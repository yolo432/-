module gimbal_servo_pwm #(
    parameter CLOCK_HZ = 50000000,
    parameter PERIOD_US = 20000,
    parameter CENTER_US = 1500,
    parameter MIN_US = 1000,
    parameter MAX_US = 2000,
    parameter STEP_US = 8,
    parameter CENTER_X = 160,
    parameter CENTER_Y = 90,
    parameter DEAD_X = 8,
    parameter DEAD_Y = 6,
    parameter PAN_INVERT = 1,
    parameter TILT_INVERT = 1
) (
    input wire clk,
    input wire src_clk,
    input wire rst_n,
    input wire target_update,
    input wire target_valid,
    input wire [8:0] target_x,
    input wire [7:0] target_y,
    output wire pan_pwm,
    output wire tilt_pwm
);
    localparam TICKS_PER_US = CLOCK_HZ / 1000000;
    localparam DIV_BITS = (TICKS_PER_US > 1) ? $clog2(TICKS_PER_US) : 1;
    (* async_reg = "true" *) reg [1:0] src_reset, dst_reset;
    always @(posedge src_clk or negedge rst_n)
        if(!rst_n) src_reset <= 0;
        else src_reset <= {src_reset[0], 1'b1};
    always @(posedge clk or negedge rst_n)
        if(!rst_n) dst_reset <= 0;
        else dst_reset <= {dst_reset[0], 1'b1};

    // 00 holds, 01 increases pulse width, 10 decreases it.
    wire [1:0] pan_direction =
        (target_x > CENTER_X + DEAD_X) ? (PAN_INVERT ? 2'd2 : 2'd1) :
        (target_x < CENTER_X - DEAD_X) ? (PAN_INVERT ? 2'd1 : 2'd2) : 2'd0;
    wire [1:0] tilt_direction =
        (target_y > CENTER_Y + DEAD_Y) ? (TILT_INVERT ? 2'd2 : 2'd1) :
        (target_y < CENTER_Y - DEAD_Y) ? (TILT_INVERT ? 2'd1 : 2'd2) : 2'd0;

    // Hold the entire mailbox until the destination acknowledges its toggle.
    // Only per-frame observations are published, never the held-box level.
    reg [4:0] mailbox;
    reg request;
    reg acknowledge;
    (* async_reg = "true" *) reg [1:0] ack_sync, req_sync;
    always @(posedge src_clk or negedge rst_n) begin
        if(!rst_n) begin
            ack_sync <= 0;
            request <= 0;
            mailbox <= 0;
        end else if(!src_reset[1]) begin
            ack_sync <= 0;
            request <= 0;
            mailbox <= 0;
        end else begin
            ack_sync <= {ack_sync[0], acknowledge};
            if(target_update && (request == ack_sync[1])) begin
                mailbox <= {target_valid, pan_direction, tilt_direction};
                request <= ~request;
            end
        end
    end

    reg [4:0] command;
    reg pending;
    reg [DIV_BITS-1:0] divider;
    reg [14:0] period_count;
    reg [10:0] pan_us, tilt_us;
    wire us_tick = (divider == TICKS_PER_US - 1);
    wire period_end = us_tick && (period_count == PERIOD_US - 1);
    wire new_command = (req_sync[1] != acknowledge);

    function [10:0] next_pulse;
        input [10:0] current;
        input [1:0] direction;
        begin
            case(direction)
                2'd1: next_pulse = (current > MAX_US - STEP_US) ?
                                    MAX_US : current + STEP_US;
                2'd2: next_pulse = (current < MIN_US + STEP_US) ?
                                    MIN_US : current - STEP_US;
                default: next_pulse = current;
            endcase
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            req_sync <= 0;
            acknowledge <= 0;
            command <= 0;
            pending <= 0;
            divider <= 0;
            period_count <= 0;
            pan_us <= CENTER_US;
            tilt_us <= CENTER_US;
        end else if(!dst_reset[1]) begin
            req_sync <= 0;
            acknowledge <= 0;
            command <= 0;
            pending <= 0;
            divider <= 0;
            period_count <= 0;
            pan_us <= CENTER_US;
            tilt_us <= CENTER_US;
        end else begin
            req_sync <= {req_sync[0], request};
            if(us_tick) begin
                divider <= 0;
                if(period_end) period_count <= 0;
                else period_count <= period_count + 1'b1;
            end else divider <= divider + 1'b1;

            // A new observation replaces a queued one, including invalidation.
            // Each observation can cause at most one step, even if src_clk stops.
            if(new_command) begin
                command <= mailbox;
                pending <= 1'b1;
                acknowledge <= req_sync[1];
            end else if(period_end && pending) begin
                pending <= 1'b0;
                if(command[4]) begin
                    pan_us <= next_pulse(pan_us, command[3:2]);
                    tilt_us <= next_pulse(tilt_us, command[1:0]);
                end
            end
        end
    end

    assign pan_pwm = dst_reset[1] && (period_count < pan_us);
    assign tilt_pwm = dst_reset[1] && (period_count < tilt_us);
endmodule
