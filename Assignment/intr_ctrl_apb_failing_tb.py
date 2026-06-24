# =============================================================
# Cocotb testbench for interrupt_controller_apb
#
# Tests covered:
#   T1  - Reset state verification
#   T2  - APB write transaction and interrupt source update happen concurrently.
#   T3  - Incorrect ISR due to scoreboard priority encoder modeling.
#   T4  - Faulty handling of delayed APB response.
#   T5  - Software Interrupt Race with Hardware Interrupts
#   T6  - Verify APB write data matches the actual ISR value.
#   T7  - Level Interrupt & IPR Synchronization Verification.
#   T8  - APB Error Response Handling.
#   T9  - Scoreboard prediction for level interrupt when IMR bit is cleared.
# =============================================================


import cocotb
from cocotb.clock      import Clock
from cocotb.triggers   import RisingEdge, FallingEdge, Timer, First
import logging

# ─────────────────────────────────────────────────────────────
# Register Address Map (must match RTL)
# ─────────────────────────────────────────────────────────────
ADDR_IMR      = 0x0
ADDR_IPR      = 0x1
ADDR_ISR      = 0x2
ADDR_TRIG     = 0x3
ADDR_SWIR     = 0x4
ADDR_GCR      = 0x5
ADDR_VECT     = 0x6
ADDR_STAT     = 0x7
ADDR_MICR     = 0x8
ADDR_MADDR_LO = 0x9
ADDR_MADDR_HI = 0xA

# APB FSM states (for logging)
APB_IDLE   = 0b00
APB_SETUP  = 0b01
APB_ACCESS = 0b10

# ─────────────────────────────────────────────────────────────
# APB Slave Model
# Runs as a background coroutine, responds to APB transactions
# ─────────────────────────────────────────────────────────────
class APBSlaveModel:
    """
    Lightweight APB slave model that responds to write transactions.
    Configurable: ready_latency (cycles before asserting PREADY)
                  inject_error  (assert PSLVERR on next transaction)
    """
    def __init__(self, dut, ready_latency=1, inject_error=False):
        self.dut           = dut
        self.ready_latency = ready_latency
        self.inject_error  = inject_error
        self.transactions  = []       # Log of all received transactions
        self.log           = logging.getLogger("APBSlave")
        self._running      = False

    async def start(self):
        """Start the APB slave response loop."""
        self._running = True
        dut = self.dut

        # Default outputs
        dut.PREADY.value  = 0
        dut.PSLVERR.value = 0

        while self._running:
            # Wait for PSEL to go high (SETUP phase)
            await RisingEdge(dut.clk)

            if dut.PSEL.value == 1 and dut.PENABLE.value == 0:
                # SETUP phase detected — wait for ACCESS phase
                await RisingEdge(dut.clk)

                # Now in ACCESS phase — add configured latency
                for _ in range(self.ready_latency - 1):
                    await RisingEdge(dut.clk)

                # Assert PREADY (and optionally PSLVERR)
                dut.PREADY.value  = 1
                dut.PSLVERR.value = 1 if self.inject_error else 0

                # Log the transaction
                txn = {
                    "addr"  : int(dut.PADDR.value),
                    "data"  : int(dut.PWDATA.value),
                    "write" : int(dut.PWRITE.value),
                    "error" : self.inject_error
                }
                self.transactions.append(txn)
                self.log.info(
                    f"APB TXN | ADDR=0x{txn['addr']:04X} "
                    f"DATA=0x{txn['data']:02X} "
                    f"PSLVERR={txn['error']}"
                )

                await RisingEdge(dut.clk)
                dut.PREADY.value  = 0
                dut.PSLVERR.value = 0

                # Reset inject_error after one use
                self.inject_error = False

    def stop(self):
        self._running = False

    def last_transaction(self):
        return self.transactions[-1] if self.transactions else None

    def transaction_count(self):
        return len(self.transactions)


# ─────────────────────────────────────────────────────────────
# Helper: CPU register read / write tasks
# ─────────────────────────────────────────────────────────────
async def cpu_write(dut, addr, data):
    """Perform one CPU register write."""
    await RisingEdge(dut.clk)
    dut.cpu_wr.value    = 1
    dut.cpu_rd.value    = 0
    dut.cpu_addr.value  = addr
    dut.cpu_wdata.value = data
    await RisingEdge(dut.clk)
    dut.cpu_wr.value    = 0
    dut.cpu_wdata.value = 0


async def cpu_read(dut, addr):
    """Perform one CPU register read, return the read value."""
    await RisingEdge(dut.clk)
    dut.cpu_rd.value   = 1
    dut.cpu_wr.value   = 0
    dut.cpu_addr.value = addr
    await Timer(1, units="ns")          # Let combinational settle
    val = int(dut.cpu_rdata.value)
    await RisingEdge(dut.clk)
    dut.cpu_rd.value   = 0
    return val


async def cpu_ack(dut):
    """Assert cpu_ack for one cycle."""
    await RisingEdge(dut.clk)
    dut.cpu_ack.value = 1
    await RisingEdge(dut.clk)
    dut.cpu_ack.value = 0


async def reset_dut(dut, cycles=4):
    """Apply active-low reset for N cycles."""
    dut.rst_n.value     = 0
    dut.irq.value       = 0
    dut.cpu_ack.value   = 0
    dut.cpu_wr.value    = 0
    dut.cpu_rd.value    = 0
    dut.cpu_addr.value  = 0
    dut.cpu_wdata.value = 0
    dut.PREADY.value    = 0
    dut.PSLVERR.value   = 0
    for _ in range(cycles):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def wait_apb_idle(dut, timeout=50):
    """Wait until APB is no longer busy (apb_busy == 0)."""
    for _ in range(timeout):
        await RisingEdge(dut.clk)
        if int(dut.apb_busy.value) == 0:
            return
   # raise TestFailure("Timeout waiting for APB to become idle")
    assert False,"Timeout waiting for APB to become idle"

async def fire_irq_edge(dut, irq_mask, cycles=2):
    """Assert IRQ lines for N cycles then deassert (edge mode)."""
    await RisingEdge(dut.clk)
    dut.irq.value = irq_mask
    for _ in range(cycles):
        await RisingEdge(dut.clk)
    dut.irq.value = 0


# ─────────────────────────────────────────────────────────────
# TEST 1: Reset state verification
# ─────────────────────────────────────────────────────────────
@cocotb.test()
async def test_01_reset_state(dut):
    """T1: All registers at correct reset values."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    dut._log.info("T1: Checking reset state")

    # IMR should be 0xFF (all masked)
    val = await cpu_read(dut, ADDR_IMR)
    assert val == 0xFF, f"T1 FAIL: IMR expected 0xFF, got 0x{val:02X}"

    # GCR should be 0x00 (GIE disabled)
    val = await cpu_read(dut, ADDR_GCR)
    assert val == 0x00, f"T1 FAIL: GCR expected 0x00, got 0x{val:02X}"

    # IPR should be 0x00
    val = await cpu_read(dut, ADDR_IPR)
    assert val == 0x00, f"T1 FAIL: IPR expected 0x00, got 0x{val:02X}"

    # MICR should be 0x00 (line mode, no error)
    val = await cpu_read(dut, ADDR_MICR)
    assert val == 0x00, f"T1 FAIL: MICR expected 0x00, got 0x{val:02X}"

    # APB outputs should be deasserted
    assert int(dut.PSEL.value)    == 0, "T1 FAIL: PSEL should be 0 at reset"
    assert int(dut.PENABLE.value) == 0, "T1 FAIL: PENABLE should be 0 at reset"
    assert int(dut.INT.value)     == 0, "T1 FAIL: INT should be 0 at reset"

    dut._log.info("T1 PASS: Reset state verified")



# ──────────────────────────────────────────────────────────────────────────────
# TEST 2: APB write transaction and interrupt source update happen concurrently.
# ──────────────────────────────────────────────────────────────────────────────

@cocotb.test()
async def test_2_apb_irq_data_race(dut):
    """
    BUG INSERTED
    Scoreboard incorrectly assumes that when IRQ1
    arrives during an ongoing APB transaction, the
    APB message should contain IRQ1 information.

    This is a scoreboard prediction bug.
    DUT may be correct, but test is expected to fail.
    """

    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    # Slow slave so APB remains active
    slave = APBSlaveModel(dut, ready_latency=1)
    cocotb.start_soon(slave.start())

    dut._log.info("T2: APB Data vs Interrupt Source Race Test")

    # Configure messaged mode
    await cpu_write(dut, ADDR_TRIG,     0xFF)
    await cpu_write(dut, ADDR_IMR,      0xFC)   # Unmask IRQ0 and IRQ1
    await cpu_write(dut, ADDR_GCR,      0x01)
    await cpu_write(dut, ADDR_MADDR_LO, 0x80)
    await cpu_write(dut, ADDR_MADDR_HI, 0x10)
    await cpu_write(dut, ADDR_MICR,     0x01)   # Message mode

    # --------------------------------------------------
    # Fire IRQ0
    # --------------------------------------------------
    await RisingEdge(dut.clk)
    dut.irq.value = 0x01

    await RisingEdge(dut.clk)
    dut.irq.value = 0x00

    # APB transaction should start

    await RisingEdge(dut.clk)

    # --------------------------------------------------
    # Inject IRQ1 while APB transaction is active
    # --------------------------------------------------
    dut._log.info("Injecting IRQ1 during APB transaction")

    dut.irq.value = 0x02

    await RisingEdge(dut.clk)
    dut.irq.value = 0x00

    # Wait for APB completion
    await wait_apb_idle(dut, timeout=50)

    txn = slave.last_transaction()

    assert txn is not None, \
        "T2 FAIL: No APB transaction captured"

    dut._log.info(
        f"Captured APB transaction: "
        f"ADDR=0x{txn['addr']:04X} "
        f"DATA=0x{txn['data']:02X}"
    )

    # Read ISR
    isr_val = await cpu_read(dut, ADDR_ISR)

    dut._log.info(f"Final ISR = 0x{isr_val:02X}")

    # BUG INSERTED
    # Scoreboard wrongly assumes IRQ1 overwrites IRQ0
    # during APB transaction.
    # Expected ISR is intentionally predicted wrong.

    expected_isr = 0x02

    assert isr_val == expected_isr, \
        (f"T2 FAIL: "
         f"Expected ISR=0x02 but got ISR=0x{isr_val:02X}")


    expected_apb_data = 0x02

    assert txn["data"] == expected_apb_data, \
        (f"T2 FAIL: "
         f"Expected APB_DATA=0x02 but got "
         f"0x{txn['data']:02X}")

    assert slave.transaction_count() >= 1, \
        "T2 FAIL: Expected APB transaction not observed"

    slave.stop()

    dut._log.info(
        f"T2 PASS: APB_DATA=0x{txn['data']:02X}, "
        f"ISR=0x{isr_val:02X}"
    )




# ──────────────────────────────────────────────────────────────────
# TEST 3: Incorrect ISR due to scoreboard priority encoder modeling.
# ──────────────────────────────────────────────────────────────────


@cocotb.test()
async def test_3_priority_encoder_correctness(dut):
    """
    BUG INSERTED
    Scoreboard incorrectly determines the highest-priority
    interrupt when multiple interrupts occur simultaneously.

    Correct behavior: IRQ0 has highest priority.
    Buggy scoreboard prediction:IRQ5 has highest priority.

    Expected Result: DUT may be correct, but test FAILS due to
                     incorrect scoreboard prediction.
    """

    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    dut._log.info("T3: Priority Encoder Verification")

    # Configure line interrupt mode
    await cpu_write(dut, ADDR_MICR, 0x00)

    # All interrupts edge-triggered
    await cpu_write(dut, ADDR_TRIG, 0xFF)

    # Unmask all interrupts
    await cpu_write(dut, ADDR_IMR, 0x00)

    # Enable global interrupt
    await cpu_write(dut, ADDR_GCR, 0x01)

    # --------------------------------------------------
    # Simultaneous interrupts:
    # IRQ0 + IRQ2 + IRQ5
    # --------------------------------------------------
    dut._log.info("Asserting IRQ0, IRQ2 and IRQ5 simultaneously")

    dut.irq.value = 0x25      # 0010_0101

    for _ in range(5):
        await RisingEdge(dut.clk)

    # INT should assert
    assert int(dut.INT.value) == 1, \
        "T3 FAIL: INT output not asserted"

    # Read DUT vector
    vect = int(dut.INT_VECT.value)

    
    # BUG INSERTED
    # Scoreboard incorrectly assumes IRQ5 has
    # higher priority than IRQ0.
    # Correct Winner : IRQ0
    # Buggy Winner   : IRQ5
    

    expected_vect = 5

    assert vect == expected_vect, \
        (f"T3 FAIL: "
         f"Scoreboard expected IRQ5, got IRQ{vect}")

    # Read ISR
    isr = await cpu_read(dut, ADDR_ISR)


    expected_isr = 0x20   # IRQ5 bit

    assert (isr & expected_isr), \
        (f"T3 FAIL: "
         f"Scoreboard expected ISR=0x20 "
         f"but got ISR=0x{isr:02X}")

    dut._log.info(
        f"T3 PASS: INT_VECT={vect}, ISR=0x{isr:02X}"
    )

    # Cleanup
    dut.irq.value = 0x00

    await cpu_ack(dut)

    for _ in range(3):
        await RisingEdge(dut.clk)



# ────────────────────────────────────────────────────
# TEST 4: APB Faulty handling of delayed APB response.
# ────────────────────────────────────────────────────

@cocotb.test()
async def test_4_delayed_apb_response(dut):
    """
    BUG INSERTED
    Scoreboard does not account for delayed PREADY responses.
    It assumes APB transactions complete immediately and
    therefore makes incorrect predictions when the slave
    inserts wait states.

    Expected Result:
        DUT may behave correctly,
        but test FAILS due to incorrect scoreboard assumptions.
    """

    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    # Delayed APB slave response
    slave = APBSlaveModel(dut, ready_latency=10)
    cocotb.start_soon(slave.start())

    dut._log.info("T4: Delayed APB Response Test")

    # Configure message mode
    await cpu_write(dut, ADDR_TRIG,     0xFF)
    await cpu_write(dut, ADDR_IMR,      0xFE)   # Unmask IRQ0
    await cpu_write(dut, ADDR_GCR,      0x01)   # Global enable
    await cpu_write(dut, ADDR_MADDR_LO, 0x80)
    await cpu_write(dut, ADDR_MADDR_HI, 0x10)
    await cpu_write(dut, ADDR_MICR,     0x01)   # Message mode

    dut._log.info("Generating IRQ0")

    # Trigger interrupt
    await fire_irq_edge(dut, 0x01)

    # Wait only a few cycles while slave is still delaying

    for _ in range(5):
        await RisingEdge(dut.clk)

    busy = int(dut.apb_busy.value)

    # BUG INSERTED
    # Scoreboard incorrectly assumes APB transaction
    # should already be finished.

    assert busy == 0, \
        ("T14 FAIL: "
         "Scoreboard expected APB transaction to "
         "complete immediately")

    dut._log.info(
        "Scoreboard incorrectly predicted APB completion"
    )

    # Wait for actual APB completion
    await wait_apb_idle(dut, timeout=50)

    # Scoreboard assumes delayed transaction timed out
    # and predicts that no APB transaction occurred.

    assert slave.transaction_count() == 0, \
        (f"T4 FAIL: "
         f"Scoreboard expected 0 APB transactions, "
         f"got {slave.transaction_count()}")

    txn = slave.last_transaction()

    if txn is not None:
        dut._log.info(
            f"APB Transaction Captured: "
            f"ADDR=0x{txn['addr']:04X} "
            f"DATA=0x{txn['data']:02X}"
        )

    # Scoreboard incorrectly expects APB to remain busy
    # because it never modeled delayed PREADY correctly.

    assert int(dut.apb_busy.value) == 1, \
        ("T4 FAIL: "
         "Scoreboard expected APB bus still busy")

    dut._log.info(
        "T4 PASS: Delayed APB response handled correctly"
    )

    slave.stop()



# ────────────────────────────────────────────────────────
# TEST 5:Software Interrupt Race with Hardware Interrupts.
# ────────────────────────────────────────────────────────

@cocotb.test()
async def test_5_swir_hw_irq_race(dut):
    """
    BUG INSERTED:
    Scoreboard incorrectly assumes that when a
    software interrupt arrives while a hardware
    interrupt is being processed, only the newest
    interrupt remains active.

    Correct behavior: HW IRQ0 and SW IRQ3 should both be present.

    Buggy scoreboard prediction: Only SW IRQ3 should remain.
    """

    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    dut._log.info("T5: SWIR vs HW IRQ Race Test")

    await cpu_write(dut, ADDR_TRIG, 0xFF)
    await cpu_write(dut, ADDR_IMR, 0xF6)   # IRQ0 + IRQ3 enabled
    await cpu_write(dut, ADDR_GCR, 0x01)

    # Hardware IRQ0
    
    dut._log.info("Triggering HW IRQ0")

    await fire_irq_edge(dut, 0x01)

    await RisingEdge(dut.clk)

    # Inject SW IRQ3 while HW interrupt exists

    dut._log.info("Injecting SW IRQ3")

    await cpu_write(dut, ADDR_SWIR, 0x08)

    for _ in range(3):
        await RisingEdge(dut.clk)

    ipr = await cpu_read(dut, ADDR_IPR)
    isr = await cpu_read(dut, ADDR_ISR)

    dut._log.info(f"IPR = 0x{ipr:02X}")
    dut._log.info(f"ISR = 0x{isr:02X}")

    status = ipr | isr


    # BUG INSERTED
    # Scoreboard assumes SW interrupt overwrites
    # existing HW interrupt.


    expected_status = 0x08

    assert status == expected_status, \
        (f"T5 FAIL: "
         f"Scoreboard expected only SW IRQ3 "
         f"(0x08), got 0x{status:02X}")

    dut._log.info(
        f"T5 PASS: Status=0x{status:02X}"
    )



# ───────────────────────────────────────────────────────────
# TEST 6: Verify APB write data matches the actual ISR value.
# ───────────────────────────────────────────────────────────


@cocotb.test()
async def test_6_interrupt_message_data_correctness(dut):
    """
    BUG INSERTED:
    Scoreboard incorrectly samples the interrupt vector
    and predicts the wrong ISR value.

    Actual interrupt: IRQ3 -> ISR = 0x08

    Buggy scoreboard prediction: IRQ4 -> ISR = 0x10

    Result:
        DUT may be correct, but test FAILS because
        scoreboard expects wrong interrupt message data.
    """

    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    slave = APBSlaveModel(dut, ready_latency=1)
    cocotb.start_soon(slave.start())

    dut._log.info("T6: Interrupt Message Data Verification")

    # Configure messaged mode
    await cpu_write(dut, ADDR_TRIG, 0xFF)

    # Unmask IRQ3
    await cpu_write(dut, ADDR_IMR, 0xF7)

    await cpu_write(dut, ADDR_GCR, 0x01)

    # Message address
    await cpu_write(dut, ADDR_MADDR_LO, 0x40)
    await cpu_write(dut, ADDR_MADDR_HI, 0x00)

    # Enable messaged mode
    await cpu_write(dut, ADDR_MICR, 0x01)

    dut._log.info("Triggering IRQ3")

    await fire_irq_edge(dut, 0x08)

    # Wait APB completion
    await wait_apb_idle(dut, timeout=50)

    # Give slave time to log transaction
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)

    # Read actual ISR
    isr_val = await cpu_read(dut, ADDR_ISR)

    dut._log.info(f"ISR value = 0x{isr_val:02X}")

    txn = slave.last_transaction()

    assert txn is not None, \
        "T6 FAIL: No APB transaction captured"

    apb_data = txn["data"]

    dut._log.info(
        f"APB PWDATA = 0x{apb_data:02X}"
    )

    # BUG INSERTED
    # Scoreboard incorrectly samples interrupt vector.

    predicted_isr = 0x10

    assert apb_data == predicted_isr, \
        (f"T6 FAIL: "
         f"Scoreboard expected APB_DATA=0x10 "
         f"but got 0x{apb_data:02X}")

    assert isr_val == predicted_isr, \
        (f"T6 FAIL: "
         f"Scoreboard expected ISR=0x10 "
         f"but got ISR=0x{isr_val:02X}")

    dut._log.info(
        f"T6 PASS: APB message data matches ISR "
        f"(Predicted ISR=0x{predicted_isr:02X})"
    )

    slave.stop()



# ───────────────────────────────────────────────────────────
# TEST 7: Level Interrupt & IPR Synchronization Verification.
# ───────────────────────────────────────────────────────────

@cocotb.test()
async def test_7_level_interrupt_ipr_sync(dut):
    """
    BUG INSERTED:
    Scoreboard incorrectly predicts that IPR follows
    the IRQ pin immediately in level-triggered mode.

    RTL behavior:
        IRQ asserted   -> IPR set
        IRQ removed    -> IPR remains pending
        CPU ACK        -> IPR clears

    Buggy scoreboard prediction: IRQ remove -> IPR clears immediately

    Result: RTL and scoreboard mismatch.
    """

    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    dut._log.info("T7: Level Interrupt IPR Synchronization Test")

    # Level-triggered mode
    await cpu_write(dut, ADDR_TRIG, 0x00)

    # Unmask IRQ0
    await cpu_write(dut, ADDR_IMR, 0xFE)

    # Global enable
    await cpu_write(dut, ADDR_GCR, 0x01)

    # Assert IRQ0
    
    dut._log.info("Asserting IRQ0")

    dut.irq.value = 0x01

    for _ in range(3):
        await RisingEdge(dut.clk)

    ipr_set = await cpu_read(dut, ADDR_IPR)

    dut._log.info(
        f"IPR while IRQ0 asserted = 0x{ipr_set:02X}"
    )

    assert (ipr_set & 0x01), \
        f"T7 FAIL: IRQ0 not reflected in IPR, IPR=0x{ipr_set:02X}"

    # Deassert IRQ0

    dut._log.info("Deasserting IRQ0")

    dut.irq.value = 0x00

    for _ in range(3):
        await RisingEdge(dut.clk)

    ipr_pending = await cpu_read(dut, ADDR_IPR)

    dut._log.info(
        f"IPR after IRQ0 removed = 0x{ipr_pending:02X}"
    )

    # BUG INSERTED
    # Scoreboard incorrectly assumes IPR immediately
    # follows IRQ pin state.

    expected_ipr = 0x00

    assert ipr_pending == expected_ipr, \
        (f"T7 FAIL: "
         f"Scoreboard expected IPR=0x00 after IRQ removal, "
         f"got IPR=0x{ipr_pending:02X}")

    # CPU ACK
    
    dut._log.info("CPU ACK")

    await cpu_ack(dut)

    for _ in range(3):
        await RisingEdge(dut.clk)

    ipr_after_ack = await cpu_read(dut, ADDR_IPR)

    dut._log.info(
        f"IPR after ACK = 0x{ipr_after_ack:02X}"
    )

    assert (ipr_after_ack & 0x01) == 0, \
        f"T7 FAIL: Pending interrupt not cleared after ACK, IPR=0x{ipr_after_ack:02X}"

    dut._log.info(
        f"T7 PASS: IPR synchronized correctly "
        f"(Set=0x{ipr_set:02X}, "
        f"AfterRemove=0x{ipr_pending:02X}, "
        f"AfterAck=0x{ipr_after_ack:02X})"
    )




# ────────────────────────────────────
# TEST 8: APB Error Response Handling.
# ────────────────────────────────────

@cocotb.test()
async def test_8_apb_error_response_handling(dut):
    """
    BUG INSERTED:
    Scoreboard ignores PSLVERR and incorrectly predicts
    that no APB error status should be set.
    """

    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    slave = APBSlaveModel(dut, ready_latency=1, inject_error=True)
    cocotb.start_soon(slave.start())

    dut._log.info("T8: APB Error Response Handling")

    # Configure messaged mode
    await cpu_write(dut, ADDR_TRIG, 0xFF)
    await cpu_write(dut, ADDR_IMR, 0xFE)
    await cpu_write(dut, ADDR_GCR, 0x01)
    await cpu_write(dut, ADDR_MICR, 0x01)

    # Trigger IRQ0
    await fire_irq_edge(dut, 0x01)

    for _ in range(10):
        await RisingEdge(dut.clk)

    micr = await cpu_read(dut, ADDR_MICR)

    dut._log.info(f"MICR = 0x{micr:02X}")

    # BUG INSERTED
    # Scoreboard incorrectly ignores PSLVERR.
    # It predicts apb_err remains 0.
    # RTL actually sets apb_err = 1.

    expected_apb_err = 0

    actual_apb_err = (micr >> 1) & 1

    assert actual_apb_err == expected_apb_err, \
        (f"T8 FAIL: "
         f"Scoreboard expected apb_err=0, "
         f"got apb_err={actual_apb_err}")

    slave.stop()




# ──────────────────────────────────────────────────────────────────────────
# TEST 9: Scoreboard prediction for level interrupt when IMR bit is cleared.
# ──────────────────────────────────────────────────────────────────────────

@cocotb.test()
async def test_9_masked_level_irq_becomes_pending_after_unmask(dut):
    """
    BUG INSERTED
    Scoreboard incorrectly assumes that a level interrupt
    active while masked cannot become pending after unmasking.

    RTL:IRQ0 active while masked
        IMR cleared -> IPR becomes 0x01

    Buggy Scoreboard: Expects IPR to remain 0x00
    """

    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    dut._log.info(
        "T9: Masked Level Interrupt Becomes Pending After Unmask"
    )

    # Level-triggered mode
    await cpu_write(dut, ADDR_TRIG, 0x00)

    # IRQ0 masked
    await cpu_write(dut, ADDR_IMR, 0xFF)

    # Global enable
    await cpu_write(dut, ADDR_GCR, 0x01)

    # IRQ0 active while masked
    dut.irq.value = 0x01

    for _ in range(3):
        await RisingEdge(dut.clk)

    ipr_masked = await cpu_read(dut, ADDR_IPR)

    dut._log.info(f"IPR while masked = 0x{ipr_masked:02X}")

    # Unmask IRQ0
    await cpu_write(dut, ADDR_IMR, 0xFE)

    for _ in range(3):
        await RisingEdge(dut.clk)

    ipr_unmasked = await cpu_read(dut, ADDR_IPR)

    dut._log.info(f"IPR after unmask = 0x{ipr_unmasked:02X}")

    # BUG INSERTED
    # Scoreboard forgets to re-check active level IRQ
    # after unmasking.
    # Wrong prediction - IPR = 0x00
    # RTL - IPR = 0x01

    expected_ipr = 0x00

    assert ipr_unmasked == expected_ipr, \
        (f"T9 FAIL: "
         f"Scoreboard expected IPR=0x00 after unmask, "
         f"got IPR=0x{ipr_unmasked:02X}")

    dut.irq.value = 0x00
