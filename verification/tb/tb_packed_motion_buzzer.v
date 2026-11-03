`timescale 1ns/1ps
module tb_packed_motion_buzzer;
    reg clk=0,rst_n=0,sof=0,valid=0,alarm=0;
    wire packed_sof,packed_valid,buzzer;
    integer tones=0;
    always #1 clk=~clk;
    data_96bit_to_128bit packer (
        .I_clk(clk),.I_rst_n(rst_n),
        .I_96b_frame_start(sof),.I_96b_valid(valid),.I_96b_data(96'd0),
        .O_128b_frame_start(packed_sof),.O_128b_valid(packed_valid)
    );
    motion_buzzer #(.CLOCK_HZ(1000),.TONE_HZ(100),.ALARM_FRAMES(3)) dut (
        .clk(clk),.rst_n(rst_n),.frame_start(packed_sof),
        .motion_alarm(alarm),.buzzer(buzzer)
    );
    always @(negedge clk) if(rst_n && !buzzer) tones=tones+1;
    task frame;
        begin
            @(negedge clk); sof=1; valid=1;
            @(negedge clk); sof=0;
            if(!packed_sof || packed_valid)
                $fatal(1,"test does not exercise separate SOF/valid");
            repeat(4) @(negedge clk);
            valid=0; repeat(10) @(negedge clk);
        end
    endtask
    initial begin
        repeat(3) @(negedge clk); rst_n=1;
        @(negedge clk); alarm=1;
        @(negedge clk); alarm=0;
        frame(); frame(); frame();
        if(tones==0 || !buzzer)
            $fatal(1,"actual packer marker did not start and stop active-low tone");
        $display("PASS packed SOF -> buzzer: valid-low marker, tone, timeout, high idle");
        $finish;
    end
endmodule
