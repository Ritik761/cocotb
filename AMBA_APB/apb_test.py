import cocotb
import random
from cocotb.clock import Clock
from cocotb.triggers import Timer, RisingEdge

# Transaction Class

class APBTransaction:

    def __init__(self, addr=0, data=0, write=0):
        self.addr = addr
        self.data = data
        self.write = write

    def __str__(self):
        return (
            f"APBTransaction("
            f"addr={hex(self.addr)}, "
            f"data={hex(self.data)}, "
            f"write={self.write})"
        )

                   # Done By Ritik

# Driver Class

class APBDriver:
    def __init__(self, dut):
        self.dut = dut

    async def drive(self, tr):

        # Setup phase
        self.dut.psel.value = 1
        self.dut.penable.value = 0
        self.dut.pwrite.value = tr.write
        self.dut.paddr.value = tr.addr
        self.dut.pwdata.value = tr.data

        await RisingEdge(self.dut.pclk)

        # Access phase
        self.dut.penable.value = 1

        await RisingEdge(self.dut.pclk)

        # End transaction
        self.dut.psel.value = 0
        self.dut.penable.value = 0


# Monitor Class

class APBMonitor:
    def __init__(self, dut):
        self.dut = dut

    async def capture(self):

        await RisingEdge(self.dut.pclk)

        while not (self.dut.psel.value and self.dut.penable.value):
            await RisingEdge(self.dut.pclk)

        tr = APBTransaction()

        tr.addr = int(self.dut.paddr.value)
        tr.write = int(self.dut.pwrite.value)

        if tr.write:
            tr.data = int(self.dut.pwdata.value)
        else:
            await RisingEdge(self.dut.pclk)
            tr.data = int(self.dut.prdata.value)

        return tr


# Scoreboard

class APBScoreboard:

    def __init__(self, dut):
        self.dut = dut
        self.mem = {}

    def check(self, tr):

        if tr.write:

            self.mem[tr.addr] = tr.data

            self.dut._log.info(
                f"Write: Addr ={hex(tr.addr)} Data = {hex(tr.data)}"
            )

        else:

            expected = self.mem.get(tr.addr, 0)

            self.dut._log.info(
                f"Read: Addr = {hex(tr.addr)} "
                f"Exp ={hex(expected)} "
                f"Got = {hex(tr.data)}"
            )

            assert tr.data == expected, (
                f"Mismatch at {hex(tr.addr)} "
                f"Expected ={hex(expected)} "
                f"Got = {hex(tr.data)}"
            )


                          #Done By Ritik
# Environment Class

class APBEnv:

    def __init__(self, dut):
        self.dut = dut
        self.driver = APBDriver(dut)
        self.monitor = APBMonitor(dut)
        self.scoreboard = APBScoreboard(dut)

    async def run(self, num_transactions):

        for i in range(num_transactions):

            tr = APBTransaction(
                addr=random.randrange(0, 32, 4),
                data=random.getrandbits(32),
                write=random.choice([0, 1])
            )

            self.dut._log.info(
                f"TXN={i} "
                f"ADDR={hex(tr.addr)} "
                f"WRITE={tr.write}"
            )

            monitor_task = cocotb.start_soon(
                self.monitor.capture()
            )

            await self.driver.drive(tr)

            mon_tr = await monitor_task

            self.scoreboard.check(mon_tr)



# Test

@cocotb.test()
async def apb_test(dut):

    # Start clock
    cocotb.start_soon(Clock(dut.pclk, 10, units="ns").start())

    # Reset DUT
    dut.presetn.value = 0
    dut.psel.value = 0
    dut.penable.value = 0
    dut.pwrite.value = 0
    dut.paddr.value = 0
    dut.pwdata.value = 0

    # Hold reset for 2 cycles
    await RisingEdge(dut.pclk)
    await RisingEdge(dut.pclk)

    # Release reset
    dut.presetn.value = 1
    await RisingEdge(dut.pclk)

    dut._log.info("Reset Completed")

    # Create environment
    env = APBEnv(dut)

    # Run random transactions
    await env.run(num_transactions=15)

    dut._log.info("APB Test Completed")                 # Done By Ritik

