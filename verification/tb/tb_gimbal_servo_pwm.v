`timescale 1ns/1ps
module tb_gimbal_servo_pwm;
    reg clk=0, src_clk=0, src_run=1, rst_n=0;
    always #500 clk=~clk;
    always #337 if(src_run) src_clk=~src_clk;
    reg update=0, found=0;
    reg [8:0] x=160;
    reg [7:0] y=90;
    wire pan,tilt,reverse_pan,reverse_tilt;
    integer k;
    time rise_pan=0, fall_pan=0, rise_tilt=0;
    integer pan_width_us, tilt_width_us;
    integer pulses=0;
    gimbal_servo_pwm #(.CLOCK_HZ(1000000),.PERIOD_US(5000)) dut (
        .clk(clk),.src_clk(src_clk),.rst_n(rst_n),
        .target_update(update),.target_valid(found),.target_x(x),.target_y(y),
        .pan_pwm(pan),.tilt_pwm(tilt)
    );
    gimbal_servo_pwm #(.CLOCK_HZ(1000000),.PERIOD_US(5000),
                       .PAN_INVERT(0),.TILT_INVERT(0)) reverse (
        .clk(clk),.src_clk(src_clk),.rst_n(rst_n),
        .target_update(update),.target_valid(found),.target_x(x),.target_y(y),
        .pan_pwm(reverse_pan),.tilt_pwm(reverse_tilt)
    );
    // Check actual pins throughout movement and at both travel limits.
    always @(posedge pan) if(rst_n) begin
        if(pulses > 0 && $time-rise_pan != dut.PERIOD_US*1000)
            $fatal(1,"PWM period %0t",$time-rise_pan);
        rise_pan=$time;
        pan_width_us=dut.pan_us;
        pulses=pulses+1;
    end
    always @(negedge pan) if(rst_n && pulses>1)
        if($time-rise_pan != pan_width_us*1000)
            $fatal(1,"pan pin width %0t, expected %0d us",
                      $time-rise_pan,dut.pan_us);
    always @(posedge tilt) if(rst_n) begin
        rise_tilt=$time;
        tilt_width_us=dut.tilt_us;
    end
    always @(negedge tilt) if(rst_n && pulses>1)
        if($time-rise_tilt != tilt_width_us*1000)
            $fatal(1,"tilt pulse width %0t expected %0d us (current %0d)",
                $time-rise_tilt,tilt_width_us,dut.tilt_us);
    task publish;
        input integer px,py,valid_target;
        begin
            @(negedge src_clk);
            x=px; y=py; found=valid_target; update=1;
            @(negedge src_clk); update=0;
            repeat(10) @(negedge clk);
        end
    endtask
    task publish_stable;
        input integer px,py,valid_target;
        begin
            publish(px,py,valid_target);
            publish(px,py,valid_target);
            publish(px,py,valid_target);
        end
    endtask
    task period;
        begin
            @(posedge clk);
            while(!dut.period_end) @(posedge clk);
            #1;
        end
    endtask
    task check;
        input integer expected_pan,expected_tilt;
        begin
            if(dut.pan_us != expected_pan || dut.tilt_us != expected_tilt)
                $fatal(1,"pulse positions %0d,%0d expected %0d,%0d",
                    dut.pan_us,dut.tilt_us,expected_pan,expected_tilt);
        end
    endtask
    initial begin
        repeat(8) @(negedge clk); rst_n=1;
        repeat(8) @(negedge clk);
        period(); check(1500,1500);
        publish(220,40,1); period(); check(1500,1500);
        publish(140,100,1); period(); check(1500,1500);
        publish(220,40,1); period(); check(1500,1500);
        publish(220,40,1); period(); check(1500,1500);
        publish(220,40,1);
        // Pending commands must never alter a pulse already in progress.
        repeat(2000) @(negedge clk); check(1500,1500);
        period();
        repeat(2) period();
        check(1496,1504);
        if(reverse.pan_us != 1504 || reverse.tilt_us != 1496)
            $fatal(1,"axis inversion");
        repeat(4) period(); check(1496,1504);
        publish(168,96,1); period(); check(1496,1504);
        publish(152,84,1); period(); check(1496,1504);
        publish(140,100,1); period(); check(1496,1504);
        publish(140,100,1); period(); check(1496,1504);
        publish(140,100,1);
        repeat(3) period();
        check(1500,1500);
        publish(220,40,0); period(); check(1500,1500);
        // Invalid observations replace an unconsumed movement request.
        publish_stable(220,40,1); publish(220,40,0);
        period(); check(1500,1500);
        publish_stable(220,40,1); src_run=0;
        repeat(2) period(); check(1496,1504);
        repeat(3) period(); check(1496,1504);
        src_run=1;
        // Exercise both clamps with actual PWM output checks still running.
        for(k=0;k<140;k=k+1) begin
            publish_stable(320,0,1); repeat(3) period();
        end
        check(1000,2000);
        for(k=0;k<260;k=k+1) begin
            publish_stable(0,180,1); repeat(3) period();
        end
        check(2000,1000);
        publish(0,180,0); repeat(3) period(); check(2000,1000);
        rst_n=0; #1;
        if(pan || tilt) $fatal(1,"reset must suppress pulses");
        $display("PASS gimbal PWM: 20ms periods, pin widths, inversions, deadband, bounded single steps, invalidation, stopped camera, travel clamps, reset (%0d periods)",pulses);
        $finish;
    end
    initial begin #30000000000; $fatal(1,"timeout"); end
endmodule

module tb_gimbal_50m;
    reg clk=0, src_clk=0, rst_n=0;
    always #10 clk=~clk;
    always #8.333 src_clk=~src_clk;
    wire pan,tilt;
    time rise;
    gimbal_servo_pwm dut (
        .clk(clk),.src_clk(src_clk),.rst_n(rst_n),
        .target_update(1'b0),.target_valid(1'b0),
        .target_x(9'd160),.target_y(8'd90),.pan_pwm(pan),.tilt_pwm(tilt)
    );
    initial begin
        repeat(5) @(negedge clk); rst_n=1;
        @(posedge pan); @(posedge pan); rise=$time;
        @(negedge pan);
        if($time-rise != 1500000) $fatal(1,"50MHz 1500us pulse");
        @(posedge pan);
        if($time-rise != 20000000) $fatal(1,"50MHz 20ms period");
        if(pan !== tilt) $fatal(1,"axis startup mismatch");
        $display("PASS production 50MHz divider: 50Hz, 1500us center");
        $finish;
    end
    initial begin #50000000; $fatal(1,"timeout"); end
endmodule
