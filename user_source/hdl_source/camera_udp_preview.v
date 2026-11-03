`timescale 1ns/1ps
module camera_udp_preview #(
    parameter WIDTH=160, HEIGHT=90, STEP=8, INPUT_WIDTH=1280,
    parameter FRAME_SKIP=5, GAP_CYCLES=12500, BOX_FILTER=0
)(
    input wire I_pixel_clk, I_vsync, I_de,
    input wire [23:0] I_rgb,
    input wire I_clk, I_reset, I_udp_busy,
    output reg O_udp_req,
    output wire O_udp_valid,
    output wire [15:0] O_udp_len,
    output reg [7:0] O_udp_data
);
    localparam IDLE=0, REQUEST=1, ACK=2, SEND=3, DRAIN=4, GAP=5;
    reg [15:0] pixels [0:WIDTH*HEIGHT-1];
    reg [15:0] x, y, wr_count;
    reg vs_d, de_d, capturing, ready_toggle;
    reg ack_toggle;
    (* async_reg="true" *) reg ack_meta, ack_sync;
    (* async_reg="true" *) reg ready_meta, ready_sync;
    reg [7:0] skip;
    wire start_frame=I_vsync && !vs_d;
    localparam SUM_BITS=8+2*$clog2(STEP);
    reg [SUM_BITS-1:0] h_r, h_g, h_b;
    reg [3*SUM_BITS-1:0] column_sums [0:WIDTH-1];
    reg [3*SUM_BITS-1:0] column_q;
    wire [15:0] sample_col=x/STEP;
    wire [SUM_BITS-1:0] row_r=h_r+I_rgb[23:16];
    wire [SUM_BITS-1:0] row_g=h_g+I_rgb[15:8];
    wire [SUM_BITS-1:0] row_b=h_b+I_rgb[7:0];
    wire [SUM_BITS-1:0] total_r=row_r+((y%STEP==0)?0:column_q[3*SUM_BITS-1:2*SUM_BITS]);
    wire [SUM_BITS-1:0] total_g=row_g+((y%STEP==0)?0:column_q[2*SUM_BITS-1:SUM_BITS]);
    wire [SUM_BITS-1:0] total_b=row_b+((y%STEP==0)?0:column_q[SUM_BITS-1:0]);
    wire [7:0] mean_r=total_r/(STEP*STEP);
    wire [7:0] mean_g=total_g/(STEP*STEP);
    wire [7:0] mean_b=total_b/(STEP*STEP);
    // Synchronous read has STEP pixel clocks to settle before a block ends.
    always @(posedge I_pixel_clk) begin
        if(sample_col<WIDTH) column_q<=column_sums[sample_col];
        if(BOX_FILTER && !I_reset && !start_frame && capturing && I_de &&
           x<WIDTH*STEP && y<HEIGHT*STEP && x%STEP==STEP-1)
            column_sums[sample_col]<={total_r,total_g,total_b};
    end
    // Frozen-memory mailbox: no pixel writes until the TX domain returns ACK.
    always @(posedge I_pixel_clk or posedge I_reset) begin
        if(I_reset) begin
            x<=0; y<=0; wr_count<=0; vs_d<=0; de_d<=0;
            capturing<=0; ready_toggle<=0; ack_meta<=0; ack_sync<=0; skip<=0;
            h_r<=0; h_g<=0; h_b<=0;
        end else begin
            ack_meta<=ack_toggle; ack_sync<=ack_meta;
            vs_d<=I_vsync; de_d<=I_de;
            if(start_frame) begin
                x<=0; y<=0; wr_count<=0; capturing<=0;
                h_r<=0; h_g<=0; h_b<=0;
                if(capturing && wr_count==WIDTH*HEIGHT)
                    ready_toggle<=~ready_toggle;
                else if(ready_toggle==ack_sync) begin
                    if(skip==0) begin capturing<=1; skip<=FRAME_SKIP; end
                    else skip<=skip-1'b1;
                end
            end else if(capturing) begin
                if(I_de) begin
                    x<=x+1'b1;
                    if(BOX_FILTER) begin
                        if(x%STEP==0) begin
                            h_r<=I_rgb[23:16]; h_g<=I_rgb[15:8]; h_b<=I_rgb[7:0];
                        end else begin h_r<=row_r; h_g<=row_g; h_b<=row_b; end
                        if(x<WIDTH*STEP && y<HEIGHT*STEP &&
                           x%STEP==STEP-1 && y%STEP==STEP-1 && wr_count<WIDTH*HEIGHT) begin
                            pixels[wr_count]<={mean_r[7:3],mean_g[7:2],mean_b[7:3]};
                            wr_count<=wr_count+1'b1;
                        end
                    end else if(x<INPUT_WIDTH && y<HEIGHT*STEP &&
                       x%STEP==0 && y%STEP==0 && wr_count<WIDTH*HEIGHT) begin
                        pixels[wr_count]<={I_rgb[23:19],I_rgb[15:10],I_rgb[7:3]};
                        wr_count<=wr_count+1'b1;
                    end
                end else if(de_d) begin x<=0; y<=y+1'b1; end
            end
        end
    end
    reg [2:0] state;
    reg [15:0] frame_id, row, index;
    reg [31:0] gap_count;
    reg [15:0] pixel_q;
    wire [15:0] col_prefetch=(index>=15) ? ((index-15)>>1) : 0;
    wire [31:0] rd_address=row*WIDTH+col_prefetch;
    // One-cycle RAM read; prefetch the next pixel while sending its predecessor.
    always @(posedge I_clk) begin
        if(rd_address<WIDTH*HEIGHT) pixel_q<=pixels[rd_address];
    end
    assign O_udp_valid=(state==SEND);
    assign O_udp_len=16+WIDTH*2;
    always @* begin
        case(index)
            0: O_udp_data="C"; 1: O_udp_data="A";
            2: O_udp_data="M"; 3: O_udp_data="1";
            4: O_udp_data=frame_id[15:8]; 5: O_udp_data=frame_id[7:0];
            6: O_udp_data=WIDTH>>8; 7: O_udp_data=WIDTH&255;
            8: O_udp_data=HEIGHT>>8; 9: O_udp_data=HEIGHT&255;
            10: O_udp_data=row[15:8]; 11: O_udp_data=row[7:0];
            12: O_udp_data=1; 13: O_udp_data=0;
            14: O_udp_data=(WIDTH*2)>>8; 15: O_udp_data=(WIDTH*2)&255;
            default: O_udp_data=index[0] ? pixel_q[7:0] : pixel_q[15:8];
        endcase
    end
    always @(posedge I_clk or posedge I_reset) begin
        if(I_reset) begin
            ready_meta<=0; ready_sync<=0; ack_toggle<=0; state<=IDLE;
            O_udp_req<=0; frame_id<=0; row<=0; index<=0; gap_count<=0;
        end else begin
            ready_meta<=ready_toggle; ready_sync<=ready_meta;
            case(state)
                IDLE: if(ready_sync!=ack_toggle) begin
                    frame_id<=frame_id+1'b1; row<=0; state<=REQUEST;
                end
                REQUEST: if(!I_udp_busy) begin O_udp_req<=1; state<=ACK; end
                ACK: begin
                    O_udp_req<=0;
                    if(I_udp_busy) begin index<=0; state<=SEND; end
                end
                SEND: if(index==15+WIDTH*2) begin state<=DRAIN; index<=0; end
                      else index<=index+1'b1;
                DRAIN: if(!I_udp_busy) begin gap_count<=0; state<=GAP; end
                GAP: if(gap_count>=GAP_CYCLES) begin
                    if(row==HEIGHT-1) begin ack_toggle<=ready_sync; state<=IDLE; end
                    else begin row<=row+1'b1; state<=REQUEST; end
                end else gap_count<=gap_count+1'b1;
                default: begin state<=IDLE; O_udp_req<=0; end
            endcase
        end
    end
endmodule
