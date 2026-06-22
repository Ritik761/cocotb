module apb_slave(
    input wire pclk,
    input wire presetn,
    input wire psel,
    input wire penable,
    input wire pwrite,
    input wire [31:0] paddr,
    input wire [31:0] pwdata,

    output reg [31:0] prdata,
    output wire pready,
    output reg pslverr
);

reg [31:0] mem [0:31];
integer i;

assign pready = 1'b1;

always @(posedge pclk or negedge presetn) begin

    if(!presetn) begin

        prdata <= 0;
        pslverr <= 0;

        for(i=0;i<32;i=i+1)
            mem[i] <= 0;

    end
    else begin

        if(psel && penable) begin

            if(pwrite)
                mem[paddr[4:0]] <= pwdata;
            else
                prdata <= mem[paddr[4:0]];

        end
    end
end
 initial begin
	 $dumpfile("dump.vcd");
	 $dumpvars(1,apb_slave);
 end
endmodule
