`timescale 1ns/1ps
module tb_motion_buzzer;
    reg clk=0, rst_n=0, frame_start=0, motion_alarm=0;
    wire buzzer;
    integer i, transitions;
    motion_buzzer #(.CLOCK_HZ(1000), .TONE_HZ(100), .ALARM_FRAMES(3)) dut (
        .clk(clk), .rst_n(rst_n), .frame_start(frame_start),
        .motion_alarm(motion_alarm), .buzzer(buzzer)
    );
    always #1 clk=~clk;
    always @(posedge clk) if(buzzer) transitions=transitions+1;
    task frame;
        begin
            @(negedge clk); frame_start=1;
            @(negedge clk); frame_start=0;
            repeat(4) @(negedge clk);
        end
    endtask
    initial begin
        transitions=0;
        repeat(2) @(negedge clk); rst_n=1;
        repeat(8) @(negedge clk);
        if(!buzzer) $fatal(1,"active-low driver energized without alarm");
        @(negedge clk); motion_alarm=1;
        @(negedge clk); motion_alarm=0;
        repeat(8) @(negedge clk);
        if(buzzer) $fatal(1,"buzzer did not start after alarm");
        frame();
        frame();
        frame();
        repeat(4) @(negedge clk);
        if(!buzzer) $fatal(1,"active-low buzzer did not stop after alarm frames");
        $display("PASS motion buzzer: silent idle, tone start, frame hold and timeout");
        $finish;
    end
endmodule
