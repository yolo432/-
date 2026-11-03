`timescale 1ns/1ps
// Independent DDR snapshot: capture -> drain writes -> read rows -> release.
module camera_ddr_preview #(
    parameter WIDTH=640, HEIGHT=360, BASE_ADDR=28000000, GAP_CYCLES=1250,
    parameter SOLID_TEST=0, DIRECT_SOLID_TEST=0, MEM_SOLID_TEST=0
 )(
    input wire pixel_clk, vsync, de, input wire [23:0] rgb,
    input wire clk, tx_clk, reset,
    output wire mem_wr, mem_rd, output wire [24:0] mem_addr,
    output wire [127:0] mem_wdata, input wire mem_ready, mem_valid,
    input wire [127:0] mem_rdata, input wire udp_busy,
    output reg udp_req, output wire udp_valid, output wire [15:0] udp_len,
    output reg [7:0] udp_data, output reg [31:0] dropped_frames
 );
    localparam WORDS_PER_ROW=WIDTH/8, FRAME_WORDS=WIDTH*HEIGHT/8;
    (* async_reg="true" *) reg [2:0] pixel_reset_pipe, mem_reset_pipe, tx_reset_pipe;
    always @(posedge pixel_clk or posedge reset)
        if(reset) pixel_reset_pipe<=7; else pixel_reset_pipe<={pixel_reset_pipe[1:0],1'b0};
    always @(posedge clk or posedge reset)
        if(reset) mem_reset_pipe<=7; else mem_reset_pipe<={mem_reset_pipe[1:0],1'b0};
    always @(posedge tx_clk or posedge reset)
        if(reset) tx_reset_pipe<=7; else tx_reset_pipe<={tx_reset_pipe[1:0],1'b0};
    wire pixel_reset=pixel_reset_pipe[2], mem_reset=mem_reset_pipe[2], tx_reset=tx_reset_pipe[2];
    reg arm_toggle;
    (* async_reg="true" *) reg arm_meta, arm_sync;
    reg arm_seen, vs_d, de_d, capturing, capture_done, capture_ok;
    reg [15:0] x,y;
    reg [31:0] pixel_count;
    reg [8:0] hr,hg,hb;
    reg [26:0] sums[0:WIDTH-1];
    wire [23:0] capture_rgb = SOLID_TEST ? 24'hff0000 : rgb;
    wire [15:0] column=x>>1;
    wire [9:0] row_r={1'b0,hr}+capture_rgb[23:16];
    wire [9:0] row_g={1'b0,hg}+capture_rgb[15:8];
    wire [9:0] row_b={1'b0,hb}+capture_rgb[7:0];
    wire [9:0] total_r=row_r+(y[0] ? {1'b0,sums[column][26:18]} : 10'd0);
    wire [9:0] total_g=row_g+(y[0] ? {1'b0,sums[column][17:9]} : 10'd0);
    wire [9:0] total_b=row_b+(y[0] ? {1'b0,sums[column][8:0]} : 10'd0);
    wire [15:0] averaged={total_r[9:5],total_g[9:4],total_b[9:5]};
    wire sample=capturing && de && x<WIDTH*2 && y<HEIGHT*2 && x[0] && y[0];
    reg [127:0] packed_pixels;
    reg fifo_we, overflow;
    wire fifo_full,fifo_empty,fifo_valid;
    wire [127:0] fifo_data;
    wire fifo_re;
    always @(posedge pixel_clk) begin
        if(!pixel_reset && capturing && de && !y[0] && x[0] && column<WIDTH)
            sums[column]<={hr+{1'b0,capture_rgb[23:16]},
                           hg+{1'b0,capture_rgb[15:8]},
                           hb+{1'b0,capture_rgb[7:0]}};
    end
    always @(posedge pixel_clk or posedge pixel_reset) begin
        if(pixel_reset) begin
            arm_meta<=0; arm_sync<=0; arm_seen<=0; vs_d<=0; de_d<=0;
            capturing<=0; capture_done<=0; capture_ok<=0; x<=0; y<=0;
            pixel_count<=0; hr<=0; hg<=0; hb<=0; packed_pixels<=0;
            fifo_we<=0; overflow<=0;
        end else begin
            arm_meta<=arm_toggle; arm_sync<=arm_meta; vs_d<=vsync; de_d<=de;
            fifo_we<=0;
            if(fifo_we && fifo_full) overflow<=1;
            if(vsync && !vs_d) begin
                x<=0; y<=0; pixel_count<=0; capturing<=0;
                if(capturing) begin
                    capture_ok<=!overflow && !(fifo_we && fifo_full) && pixel_count==WIDTH*HEIGHT;
                    capture_done<=~capture_done;
                end else if(arm_sync!=arm_seen) begin
                    arm_seen<=arm_sync; capturing<=1; overflow<=0;
                end
            end else if(capturing) begin
                if(de) begin
                    x<=x+1'b1;
                    if(!x[0]) begin
                        hr<=capture_rgb[23:16];
                        hg<=capture_rgb[15:8];
                        hb<=capture_rgb[7:0];
                    end
                    if(sample) begin
                        packed_pixels<={packed_pixels[111:0],averaged};
                        pixel_count<=pixel_count+1'b1;
                        if(pixel_count[2:0]==7) fifo_we<=1;
                    end
                end else if(de_d) begin x<=0; y<=y+1'b1; end
            end
        end
    end
    w128_d512_fifo capture_fifo(
        .rst(pixel_reset | mem_reset),.clkw(pixel_clk),.clkr(clk),.we(fifo_we && !fifo_full),
        .di(packed_pixels),.re(fifo_re),.dout(fifo_data),.valid(fifo_valid),
        .full_flag(fifo_full),.empty_flag(fifo_empty),.afull(),.aempty(),.wrusedw(),.rdusedw());
    (* async_reg="true" *) reg done_meta,done_sync,ok_meta,ok_sync;
    (* async_reg="true" *) reg ack_meta,ack_sync;
    reg row_ready, row_ack;
    reg [2:0] state;
    localparam ARM=0,CAPTURE=1,SETTLE=2,READ_ROW=3,WAIT_ROW=4,MEM_TEST_WRITE=5;
    reg done_seen, read_pending, fifo_pending, write_valid;
    reg [3:0] settle_count;
    reg [31:0] write_count;
    reg [15:0] row,word_index,frame_id;
    reg [127:0] write_data;
    wire row_fifo_full, row_fifo_empty, row_fifo_valid;
    wire [127:0] row_fifo_data;
    reg [15:0] direct_row, direct_frame;
    localparam [127:0] SOLID_RGB565_WORD =
        128'hF800F800F800F800F800F800F800F800;
    assign fifo_re=!DIRECT_SOLID_TEST && !MEM_SOLID_TEST &&
                   (state==CAPTURE || state==SETTLE) &&
                   !fifo_pending && !write_valid && !fifo_empty;
    assign mem_wr=write_valid;
    assign mem_rd=state==READ_ROW && !read_pending;
    assign mem_addr=(state==READ_ROW)?BASE_ADDR+(row*WORDS_PER_ROW+word_index)*8:BASE_ADDR+write_count*8;
    assign mem_wdata=write_data;
    // DDR read data is valid in the same cycle as mem_valid. Keep the FIFO
    // write enable aligned with that data instead of delaying it one cycle.
    wire row_fifo_we = mem_valid && read_pending && !row_fifo_full;
    always @(posedge clk or posedge mem_reset) begin
        if(mem_reset) begin
            arm_toggle<=0; done_meta<=0; done_sync<=0; ok_meta<=0; ok_sync<=0;
            ack_meta<=0; ack_sync<=0; row_ready<=0; state<=ARM; done_seen<=0;
            read_pending<=0; fifo_pending<=0; write_valid<=0; write_count<=0;
            row<=0; word_index<=0; frame_id<=0; dropped_frames<=0; settle_count<=0; write_data<=0;
        end else begin
            done_meta<=capture_done; done_sync<=done_meta; ok_meta<=capture_ok; ok_sync<=ok_meta;
            ack_meta<=row_ack; ack_sync<=ack_meta;
            if(fifo_re) fifo_pending<=1;
            if(MEM_SOLID_TEST && state==MEM_TEST_WRITE && !write_valid) begin
                write_valid<=1;
                write_data<=SOLID_RGB565_WORD;
            end else if(fifo_valid) begin
                fifo_pending<=0;
                write_valid<=1;
                write_data<=fifo_data;
            end
            if(mem_wr && mem_ready) begin
                write_valid<=0;
                if(MEM_SOLID_TEST && state==MEM_TEST_WRITE &&
                   write_count==FRAME_WORDS-1) begin
                    write_count<=write_count+1'b1;
                    row<=0;
                    word_index<=0;
                    frame_id<=frame_id+1'b1;
                    state<=READ_ROW;
                end else begin
                    write_count<=write_count+1'b1;
                end
            end
            case(state)
                ARM: begin
                    arm_toggle<=~arm_toggle;
                    write_count<=0;
                    if(MEM_SOLID_TEST) state<=MEM_TEST_WRITE;
                    else state<=CAPTURE;
                end
                CAPTURE: if(done_sync!=done_seen) begin
                    done_seen<=done_sync; settle_count<=0; state<=SETTLE;
                end
                SETTLE: begin
                    if(settle_count!=15) settle_count<=settle_count+1'b1;
                    else if(fifo_empty && !fifo_pending && !write_valid) begin
                        if(ok_sync && write_count==FRAME_WORDS) begin
                            row<=0; word_index<=0; frame_id<=frame_id+1'b1; state<=READ_ROW;
                        end else begin dropped_frames<=dropped_frames+1'b1; state<=ARM; end
                    end
                end
                READ_ROW: begin
                    if(mem_rd && mem_ready) read_pending<=1;
                    if(mem_valid && read_pending) begin
                        read_pending<=0;
                        if(word_index==WORDS_PER_ROW-1) begin row_ready<=~row_ready; state<=WAIT_ROW; end
                        else word_index<=word_index+1'b1;
                    end
                end
                WAIT_ROW: if(ack_sync==row_ready) begin
                    if(row==HEIGHT-1) state<=ARM;
                    else begin row<=row+1'b1; word_index<=0; state<=READ_ROW; end
                end
                MEM_TEST_WRITE: begin
                    // The write/ready block above advances this state into READ_ROW.
                end
                default: state<=ARM;
            endcase
        end
    end
    (* async_reg="true" *) reg ready_meta,ready_sync;
    reg [2:0] tx_state;
    localparam TX_IDLE=0,TX_REQ=1,TX_ACK=2,TX_SEND=3,TX_DRAIN=4,TX_GAP=5;
    reg [15:0] byte_index; reg [31:0] gap_count;
    reg [15:0] tx_row, tx_frame;
    reg [127:0] row_q;
    wire [15:0] prefetch=(byte_index>=15)?((byte_index-15)>>4):0;
    // Request each word two TX clocks before its first payload byte. The
    // vendor FIFO asserts valid one clock after the read request.
    wire row_fifo_re=!DIRECT_SOLID_TEST && (tx_state==TX_SEND) && (byte_index>=14) &&
                     (((byte_index-14) % 16)==0) && (prefetch<WORDS_PER_ROW);
    w128_d512_fifo row_fifo(
        .rst(mem_reset | tx_reset),.clkw(clk),.clkr(tx_clk),
        .we(row_fifo_we && !row_fifo_full),.di(mem_rdata),
        .re(row_fifo_re),.dout(row_fifo_data),.valid(row_fifo_valid),
        .full_flag(row_fifo_full),.empty_flag(row_fifo_empty),
        .afull(),.aempty(),.wrusedw(),.rdusedw());
    assign udp_valid=tx_state==TX_SEND;
    assign udp_len=16+WIDTH*2;
    always @* begin
        case(byte_index)
            0:udp_data="C"; 1:udp_data="A"; 2:udp_data="M"; 3:udp_data="1";
            4:udp_data=tx_frame[15:8]; 5:udp_data=tx_frame[7:0];
            6:udp_data=WIDTH>>8; 7:udp_data=WIDTH&255;
            8:udp_data=HEIGHT>>8; 9:udp_data=HEIGHT&255;
            10:udp_data=tx_row[15:8]; 11:udp_data=tx_row[7:0];
            12:udp_data=1; 13:udp_data=0;
            14:udp_data=(WIDTH*2)>>8; 15:udp_data=(WIDTH*2)&255;
            default:udp_data=row_q[127-((byte_index-16)%16)*8 -: 8];
        endcase
    end
    always @(posedge tx_clk or posedge tx_reset) begin
        if(tx_reset) begin ready_meta<=0; ready_sync<=0; row_ack<=0; tx_state<=TX_IDLE;
            udp_req<=0; byte_index<=0; gap_count<=0; tx_row<=0; tx_frame<=0;
            direct_row<=0; direct_frame<=0;
        end
        else begin
            ready_meta<=row_ready; ready_sync<=ready_meta;
            if(row_fifo_valid)
                row_q<=row_fifo_data;
            case(tx_state)
                TX_IDLE: begin
                    if(DIRECT_SOLID_TEST) begin
                        tx_row<=direct_row;
                        tx_frame<=direct_frame;
                        row_q<=SOLID_RGB565_WORD;
                        tx_state<=TX_REQ;
                    end else if(ready_sync!=row_ack) begin
                        tx_row<=row; tx_frame<=frame_id; tx_state<=TX_REQ;
                    end
                end
                TX_REQ: if(!udp_busy) begin udp_req<=1; tx_state<=TX_ACK; end
                TX_ACK: begin udp_req<=0; if(udp_busy) begin byte_index<=0; tx_state<=TX_SEND; end end
                TX_SEND: if(byte_index==15+WIDTH*2) tx_state<=TX_DRAIN; else byte_index<=byte_index+1'b1;
                TX_DRAIN: if(!udp_busy) begin gap_count<=0; tx_state<=TX_GAP; end
                TX_GAP: if(gap_count>=GAP_CYCLES) begin
                            if(DIRECT_SOLID_TEST) begin
                                if(direct_row==HEIGHT-1) begin
                                    direct_row<=0;
                                    direct_frame<=direct_frame+1'b1;
                                end else begin
                                    direct_row<=direct_row+1'b1;
                                end
                            end else begin
                                row_ack<=ready_sync;
                            end
                            tx_state<=TX_IDLE;
                        end
                        else gap_count<=gap_count+1'b1;
                default: begin tx_state<=TX_IDLE; udp_req<=0; end
            endcase
        end
    end
endmodule

// Ordered read response tags allow HDMI and snapshot reads to coexist.
module camera_ddr_arbiter(
    input wire clk,reset, old_wr,old_rd, input wire [24:0] old_addr,
    input wire [127:0] old_data, output wire old_ready,old_valid,
    input wire net_wr,net_rd, input wire [24:0] net_addr, input wire [127:0] net_data,
    output wire net_ready,net_valid, output wire wr,rd, output wire [24:0] addr,
    output wire [127:0] data, input wire ready,valid
);
    reg tags[0:1023]; reg [9:0] wp,rp; reg [10:0] outstanding;
    wire net_grant=!(old_wr || old_rd) && outstanding<1000;
    assign old_ready=ready && outstanding<1000;
    assign net_ready=ready && net_grant;
    assign wr=old_wr || (net_grant && ready && net_wr);
    assign rd=old_rd || (net_grant && ready && net_rd);
    assign addr=(old_wr || old_rd)?old_addr:net_addr;
    assign data=old_wr?old_data:net_data;
    assign old_valid=valid && outstanding!=0 && !tags[rp];
    assign net_valid=valid && outstanding!=0 && tags[rp];
    always @(posedge clk or posedge reset) begin
        if(reset) begin wp<=0; rp<=0; outstanding<=0; end
        else begin
            if(rd) begin tags[wp]<=!old_rd; wp<=wp+1'b1; end
            if(valid && outstanding!=0) rp<=rp+1'b1;
            case({rd,valid && outstanding!=0})
                2'b10:outstanding<=outstanding+1'b1;
                2'b01:outstanding<=outstanding-1'b1;
                default:outstanding<=outstanding;
            endcase
        end
    end
endmodule
