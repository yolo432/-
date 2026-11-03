`timescale 1ns/1ps
module tb_video_frame_latency;
    reg time_clk=0, cam_clk=0, ddr_clk=0, pix_clk=0, rst_n=0;
    always #5 time_clk=~time_clk;
    always #7 cam_clk=~cam_clk;
    always #3 ddr_clk=~ddr_clk;
    always #11 pix_clk=~pix_clk;
    reg capture=0, packed_sof=0, vsync=0, display_start=0;
    wire write_frame, read_frame;
    wire [1:0] write_slot, read_slot, read_pointer;
    wire [24:0] write_address, read_address;
    wire [15:0] latency;
    wire latency_valid;
    reg [15:0] expected_stamp[0:3];
    reg [3:0] expected_valid=0;
    reg have_previous=0;
    reg [1:0] prev=0, selected_slot=0;
    reg [15:0] latest_capture=0, selected_stamp=0;
    reg selected_valid=0;
    integer checks=0, i, expected_age, actual_age, previous_age;

    video_in writer (
        .I_rst_n(rst_n), .I_camera_clk(cam_clk),
        .I_camera_frame_start(packed_sof), .I_camera_valid(1'b0),
        .I_camera_data(128'd0), .I_mipi_rx_error(1'b0),
        .I_ddr_clk(ddr_clk), .I_display_pause(1'b0),
        .I_video_out_rd_busy(1'b0), .I_ddr_user_ready(1'b0),
        .O_video_out_rp(read_pointer), .O_frame_write(write_frame),
        .O_frame_slot(write_slot), .O_ddr_user_addr(write_address)
    );
    video_out reader (
        .I_rst_n(rst_n), .I_ddr_clk(ddr_clk), .I_video_in_wr_busy(1'b1),
        .I_video_out_rp(read_pointer), .I_ddr_user_ready(1'b0),
        .I_ddr_user_rd_valid(1'b0), .I_ddr_user_rd_data(128'd0),
        .I_dsi_clk(pix_clk), .I_video_vsync(vsync), .I_video_rd_en(1'b0),
        .O_frame_select(read_frame), .O_frame_slot(read_slot),
        .O_ddr_user_addr(read_address)
    );
    video_frame_latency #(.TIME_CLOCK_HZ(10000)) dut (
        .rst_n(rst_n), .time_clk(time_clk), .camera_clk(cam_clk),
        .capture_start(capture), .ddr_clk(ddr_clk),
        .frame_write(write_frame), .write_slot(write_slot),
        .frame_select(read_frame), .read_slot(read_slot),
        .display_clk(pix_clk), .display_rst_n(rst_n),
        .display_vsync(vsync), .display_start(display_start),
        .latency_ms(latency), .latency_valid(latency_valid)
    );

    always @(posedge cam_clk)
        if(rst_n && capture) begin
            // Independent common-clock reference; allow 1 ms quantization at
            // the capture and output Gray synchronizers, not a whole frame.
            latest_capture = dut.clock_ms;
        end
    always @(posedge ddr_clk) if(rst_n) begin
        if(read_frame) begin
            selected_slot = read_slot;
            selected_stamp = expected_stamp[read_slot];
            selected_valid = expected_valid[read_slot] &&
                             !(write_frame && write_slot==read_slot);
            #1;
            if(read_address !== selected_slot * 25'd7000000)
                $fatal(1,"read tag not aligned with actual DDR address");
        end
    end
    always @(posedge ddr_clk) if(rst_n && write_frame) begin
        if(have_previous) expected_valid[prev] <= 1;
        expected_valid[write_slot] <= 0;
        expected_stamp[write_slot] <= latest_capture;
        have_previous <= 1; prev <= write_slot;
        #1;
        if(write_address !== prev * 25'd7000000)
            $fatal(1,"capture tag not aligned with actual write address");
    end

    task camera_frame;
        begin
            @(negedge cam_clk); capture=1;
            @(negedge cam_clk); capture=0;
            repeat(20) @(negedge cam_clk);
            packed_sof=1;
            @(negedge cam_clk); packed_sof=0;
            repeat(80) @(negedge cam_clk);
        end
    endtask
    task start_read;
        begin
            @(negedge pix_clk); vsync=1;
            repeat(5) @(negedge pix_clk); vsync=0;
            repeat(20) @(negedge pix_clk);
        end
    endtask
    task show_frame;
        begin
            @(negedge pix_clk); display_start=1;
            @(posedge pix_clk);
            expected_age=(dut.clock_ms-selected_stamp)&65535;
            #1;
            if(latency_valid !== selected_valid)
                $fatal(1,"startup/in-progress validity wrong: %b vs %b",
                       latency_valid,selected_valid);
            if(selected_valid) begin
                actual_age=latency;
                if(actual_age<expected_age-1 || actual_age>expected_age+1)
                    $fatal(1,"wrong displayed frame age: %0d expected %0d slot %0d",
                           actual_age,expected_age,selected_slot);
            end
            checks=checks+1;
            @(negedge pix_clk); display_start=0;
        end
    endtask
    initial begin
        for(i=0;i<4;i=i+1) expected_stamp[i]=0;
        repeat(20) @(negedge cam_clk); rst_n=1;
        start_read(); show_frame();
        if(latency_valid) $fatal(1,"unwritten startup buffer reported valid");
        camera_frame(); start_read(); show_frame();
        camera_frame(); start_read(); show_frame();
        camera_frame(); start_read(); show_frame();
        if(!latency_valid || selected_slot != 0)
            $fatal(1,"completed capture zero not selected on third capture");
        previous_age=latency;
        repeat(100) @(negedge pix_clk);
        start_read(); show_frame();
        if(latency <= previous_age)
            $fatal(1,"repeated displayed frame incorrectly has constant delay");
        camera_frame(); start_read();
        // Capture pointer changes after read selection; display must keep the
        // metadata for the address selected at VS, not the latest pointer.
        camera_frame(); show_frame();
        if(selected_slot != 1) $fatal(1,"test did not retain pre-rotation read slot");
        @(negedge pix_clk); display_start=1;
        @(negedge pix_clk); display_start=0;
        if(latency_valid) $fatal(1,"metadata reused without a new DDR selection");
        // Writing slot zero is incomplete. Force only the selection request,
        // preserving the actual video_out address path being checked.
        force read_pointer=2'd0;
        start_read(); show_frame();
        if(latency_valid) $fatal(1,"in-progress capture was reported complete");
        release read_pointer;
        // Run the real counter across its modulo boundary.
        wait(dut.clock_ms==16'hfff0);
        camera_frame(); // slot 1, timestamp just before wrap
        camera_frame(); // slot 2 completes slot 1 and selects old slot 0
        camera_frame(); // slot 3 selects slot 1, timestamp now before wrap
        start_read(); show_frame();
        if(!latency_valid || latency>100 || selected_slot!=1)
            $fatal(1,"timestamp wrap or buffer rotation failed");
        rst_n=0; repeat(10) @(negedge cam_clk); rst_n=1;
        expected_valid=0; have_previous=0; selected_valid=0;
        start_read(); show_frame();
        if(latency_valid) $fatal(1,"reset retained stale timestamp");
        $display("PASS %0d latency checks: actual DDR slots, startup, repeated/skipped frames, in-progress invalidation, asynchronous clocks, 16-bit timestamp wrap, reset",checks);
        $finish;
    end
    initial begin #10000000; $fatal(1,"latency simulation timeout"); end
endmodule

// The vendor FIFO is irrelevant to metadata. Keep its occupancy empty while
// exercising the real video_in/video_out frame/address control logic.
module w128_d512_fifo (
    input rst,clkw,we,clkr,re,
    input [127:0] di,
    output [127:0] dout,
    output [8:0] wrusedw,rdusedw,
    output afull,full_flag,aempty,empty_flag,valid
);
    assign dout=0;
    assign wrusedw=0; assign rdusedw=0;
    assign afull=0; assign full_flag=0;
    assign aempty=1; assign empty_flag=1; assign valid=0;
endmodule
