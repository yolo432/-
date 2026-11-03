`timescale 1ns/1ps
module tb_gimbal_tracking;
    reg clk=0, src_clk=0, rst_n=0, valid=0, sof=0;
    always #5 clk=~clk;
    always #7 src_clk=~src_clk;
    reg tracking=0, fresh=0, found=0;
    reg [8:0] x=220;
    reg [7:0] y=40;
    wire update,target_valid;
    wire [8:0] tx;
    wire [7:0] ty;
    integer publications=0;
    edge_detect_96 detector (
        .I_clk(src_clk),.I_rst_n(rst_n),.I_button(4'hf),
        .I_tlast(1'b0),.I_tuser(sof),.I_tdata(96'd0),.I_tvalid(valid),
        .I_tready(1'b1),.O_track_update(update),.O_track_valid(target_valid),
        .O_track_x(tx),.O_track_y(ty)
    );
    gimbal_servo_pwm #(.CLOCK_HZ(1000000)) gimbal (
        .clk(clk),.src_clk(src_clk),.rst_n(rst_n),
        .target_update(update),.target_valid(target_valid),
        .target_x(tx),.target_y(ty)
    );
    always @(posedge src_clk) if(rst_n && update) begin
        publications=publications+1;
        if(tx !== x || ty !== y ||
           target_valid !== (tracking && fresh && found))
            $fatal(1,"incoherent per-frame tracker snapshot");
    end
    task observation;
        begin
            @(negedge src_clk); valid=1; sof=1;
            @(negedge src_clk); valid=0; sof=0;
            repeat(20) @(negedge clk);
        end
    endtask
    task period;
        begin
            @(posedge clk);
            while(!gimbal.period_end) @(posedge clk);
            #1;
        end
    endtask
    initial begin
        force detector.tracking_mode=tracking;
        force detector.track_fresh=fresh;
        force detector.track_valid=found;
        force detector.track_x=x;
        force detector.track_y=y;
        repeat(8) @(negedge clk); rst_n=1;
        repeat(8) @(negedge clk);
        fresh=1; found=1; observation(); period();
        if(gimbal.pan_us != 1500) $fatal(1,"moved outside TRACK");
        tracking=1; observation(); period();
        if(gimbal.pan_us != 1500) $fatal(1,"moved before confirmation");
        observation(); period();
        if(gimbal.pan_us != 1500) $fatal(1,"moved before third stable frame");
        observation(); repeat(2) period();
        if(gimbal.pan_us != 1496 || gimbal.tilt_us != 1504)
            $fatal(1,"tracker direction did not reach PWM");
        repeat(3) period();
        if(publications != 4 || gimbal.pan_us != 1496)
            $fatal(1,"fresh level caused repeated movement");
        fresh=0; x=0; observation(); period();
        if(gimbal.pan_us != 1496) $fatal(1,"held box moved gimbal");
        fresh=1; found=0; observation(); period();
        if(gimbal.pan_us != 1496) $fatal(1,"lost target moved gimbal");
        found=1; tracking=0; observation(); period();
        if(gimbal.pan_us != 1496) $fatal(1,"mode exit moved gimbal");
        $display("PASS ISP-to-gimbal: coherent per-frame publication, fresh-level rejection, held/lost boxes, mode gate, CDC and PWM");
        $finish;
    end
    initial begin #5000000; $fatal(1,"timeout"); end
endmodule
