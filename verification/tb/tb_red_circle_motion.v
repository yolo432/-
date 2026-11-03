`timescale 1ns/1ps
module tb_red_circle_motion;
    reg clk=0, rst_n=0, sof=0, valid=0;
    reg tracking=0, fresh=0, found=0;
    reg [8:0] x=0;
    reg [7:0] y=0;
    wire alarm, buzzer;
    integer alarms=0;
    reg heard_tone=0;

    always #1 clk=~clk;
    edge_detect_96 dut (
        .I_clk(clk), .I_rst_n(rst_n), .I_button(4'hf),
        .I_tlast(1'b0), .I_tuser(sof), .I_tdata(96'd0),
        .I_tvalid(valid), .I_tready(1'b1), .O_motion_alarm(alarm)
    );
    motion_buzzer #(.CLOCK_HZ(1000), .TONE_HZ(100), .ALARM_FRAMES(3)) speaker (
        .clk(clk), .rst_n(rst_n), .frame_start(dut.coordinate_start),
        .motion_alarm(alarm), .buzzer(buzzer)
    );
    always @(negedge clk) begin
        if(alarm) alarms=alarms+1;
        if(!buzzer && rst_n) heard_tone=1;
    end

    // Inject tracker observations at its published interface, while running
    // the actual ISP motion logic and downstream speaker without a camera.
    task observation;
        input integer px, py, is_fresh, is_found, expected;
        begin
            @(negedge clk);
            x=px; y=py; fresh=is_fresh; found=is_found;
            valid=1; sof=1;
            @(negedge clk); valid=0; sof=0;
            repeat(14) @(negedge clk);
            if(alarms != expected)
                $fatal(1,"at %0d,%0d fresh=%0d found=%0d: alarms %0d expected %0d",
                       px,py,is_fresh,is_found,alarms,expected);
        end
    endtask

    initial begin
        force dut.tracking_mode=tracking;
        force dut.track_fresh=fresh;
        force dut.track_valid=found;
        force dut.track_x=x;
        force dut.track_y=y;
        repeat(3) @(negedge clk); rst_n=1;
        observation(100,70,1,1,0);
        if(heard_tone) $fatal(1,"tone while tracking disabled");
        tracking=1;
        observation(100,70,1,1,0);
        observation(103,72,1,1,0);
        observation(98,69,1,1,0);
        observation(104,70,1,1,0);
        observation(108,70,1,1,0);
        observation(111,70,1,1,0);
        observation(112,70,1,1,1);
        if(!heard_tone) $fatal(1,"motion alarm did not reach speaker");
        if(dut.motion_hold_frames != 18 ||
           dut.status_character(3'd4,3'd0) != "M" ||
           dut.red_render_color != 24'hffff00)
            $fatal(1,"fresh displacement did not publish MOVE and yellow region");
        observation(112,70,1,1,1);
        observation(100,70,1,1,2);
        observation(100,81,1,1,2);
        observation(100,82,1,1,3);
        observation(140,110,0,1,3);
        observation(100,82,1,1,3);
        observation(0,0,0,0,3);
        observation(240,120,1,1,3);
        observation(251,120,1,1,3);
        observation(252,120,1,1,4);
        tracking=0;
        observation(20,40,1,1,4);
        tracking=1;
        observation(20,40,1,1,4);
        observation(20,40,1,1,4);
        observation(20,40,1,1,4);
        repeat(10) @(negedge clk);
        if(!buzzer) $fatal(1,"speaker did not time out");
        if(dut.motion_hold_frames != 0 ||
           dut.status_character(3'd4,3'd0) != "T")
            $fatal(1,"mode exit did not clear movement indication");
        $display("PASS ISP motion -> buzzer: first acquisition, jitter, cumulative movement, +/-X/Y threshold, held/lost track, reacquisition, mode exit, timeout");
        $finish;
    end
    initial begin
        #10000;
        $fatal(1,"motion test timeout");
    end
endmodule
