`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Create Date:    01:32:02 06/12/2023 
// Module Name:    SC500GainTbl 
//////////////////////////////////////////////////////////////////////////////////
module SC500GainTbl(
	input 			I_clk,
	input 			I_rstn,
	input 	[6:0] 	I,
	output reg	[15:0] 	O
);

    // Preserve the original 74 gain codes and one-cycle latency. Explicit
    // ranges avoid TD's incorrect constant folding of the indexed case ROM.
    always @(posedge I_clk) begin
        if (I < 7'd32)
            O <= {8'h03, 3'b001, I[4:0]};
        else if (I < 7'd64)
            O <= {8'h07, 3'b001, I[4:0]};
        else if (I < 7'd74)
            O <= {8'h0f, 3'b001, I[4:0]};
        else
            O <= 16'h0320;
    end

endmodule
