

module data_96bit_to_128bit (
    input wire        I_clk,
    input wire        I_rst_n,
   
    input wire        I_96b_frame_start,
    input wire        I_96b_valid,
    input wire[95:0]  I_96b_data,
  
    output reg        O_128b_frame_start,
    output reg        O_128b_valid,
    output reg[127:0] O_128b_data
);



    reg[95:0]  S_rem_data;
    reg[1:0]   S_rem_bytes;

    // Treat the 96-bit input as a byte stream. Four 96-bit beats (48 bytes)
    // become three 128-bit beats (also 48 bytes), with 12 -> 8 -> 4 -> 0
    // bytes remaining after each accepted input beat.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_rem_data         <= 96'd0;
            S_rem_bytes        <= 2'd0;
            O_128b_frame_start <= 1'b0;
            O_128b_valid       <= 1'b0;
            O_128b_data        <= 128'd0;
        end
        else begin
            O_128b_frame_start <= I_96b_frame_start;
            O_128b_valid       <= 1'b0;
            O_128b_data        <= 128'd0;

            if(I_96b_frame_start) begin
                // The first beat starts a fresh stream. It cannot produce a
                // complete 128-bit word until the next input beat arrives.
                S_rem_data  <= I_96b_valid ? I_96b_data : 96'd0;
                S_rem_bytes <= I_96b_valid ? 2'd1 : 2'd0;
            end
            else if(I_96b_valid) begin
                case(S_rem_bytes)
                    2'd0: begin
                        // 12 bytes buffered.
                        S_rem_data  <= I_96b_data;
                        S_rem_bytes <= 2'd1;
                    end
                    2'd1: begin
                        // 12 buffered + 4 from the new beat.
                        O_128b_valid <= 1'b1;
                        O_128b_data  <= {S_rem_data, I_96b_data[95:64]};
                        S_rem_data   <= {32'd0, I_96b_data[63:0]};
                        S_rem_bytes  <= 2'd2;
                    end
                    2'd2: begin
                        // 8 buffered + 8 from the new beat.
                        O_128b_valid <= 1'b1;
                        O_128b_data  <= {S_rem_data[63:0], I_96b_data[95:32]};
                        S_rem_data   <= {64'd0, I_96b_data[31:0]};
                        S_rem_bytes  <= 2'd3;
                    end
                    default: begin
                        // 4 buffered + the complete new beat.
                        O_128b_valid <= 1'b1;
                        O_128b_data  <= {S_rem_data[31:0], I_96b_data};
                        S_rem_data   <= 96'd0;
                        S_rem_bytes  <= 2'd0;
                    end
                endcase
            end
        end
    end

    

endmodule







/*
module data96_128 (

    input wire [95:0] I_tdata,

    output wire [127:0] O_tdata

);

  assign O_tdata = {
    {8'd0}, I_tdata[95:72], {8'd0}, I_tdata[71:48], {8'd0}, I_tdata[47:24], {8'd0}, I_tdata[23:0]
  };
 
endmodule
*/

