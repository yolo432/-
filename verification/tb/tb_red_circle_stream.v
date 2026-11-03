`timescale 1ns/1ps
module tb_red_circle_stream;
    reg clk=0;
    always #5 clk=~clk;
    reg rst=0, enable=0, valid=0, sof=0, eol=0;
    reg [8:0] x=0;
    reg [7:0] y=0;
    reg [23:0] rgb=0;
    wire box_valid, fresh;
    wire [8:0] cx,w;
    wire [7:0] cy;
    integer xx,yy,dx,dy;
    reg first_frame=1;
    red_circle_stream dut(
        .clk(clk),.rst_n(rst),.enable(enable),.valid(valid),.sof(sof),.eol(eol),
        .x(x),.y(y),.rgb(rgb),.box_valid(box_valid),.fresh(fresh),
        .center_x(cx),.center_y(cy),.diameter(w)
    );
    task frame;
        input integer tx,ty,radius,scene;
        begin
            for(yy=0;yy<180;yy=yy+1)
                for(xx=0;xx<320;xx=xx+1) begin
                    @(negedge clk);
                    valid=1; sof=(first_frame && xx==0 && yy==0); eol=(xx==319);
                    x=xx; y=yy; rgb=24'h303030;
                    dx=xx-tx; dy=yy-ty;
                    if(scene==1 && dx*dx+dy*dy<=radius*radius) rgb=24'h201614;
                    if(scene==2 && dx>=-radius && dx<=radius &&
                       dy>=-radius && dy<=radius) rgb=24'h201614;
                    if(scene==3 && dx*dx+dy*dy<=radius*radius) rgb=24'h121110;
                end
            @(negedge clk);
            valid=0; sof=0; eol=0;
            // Scan the completed profile during camera vertical blanking.
            repeat(2000) @(negedge clk);
            first_frame=0;
        end
    endtask
    task publish;
        begin
            valid=1; sof=1; x=0; y=0; rgb=24'h303030;
            @(negedge clk);
            valid=0; sof=0;
            #1;
        end
    endtask
    task location;
        input integer tx,ty,radius;
        begin
            if(!box_valid || !fresh || cx<tx-3 || cx>tx+3 || cy<ty-4 || cy>ty+4 ||
               w<2*radius-5 || w>2*radius+5)
                $fatal(1,"bad disk: valid=%b fresh=%b center=%d,%d width=%d expected %d,%d r%d",
                    box_valid,fresh,cx,cy,w,tx,ty,radius);
        end
    endtask
    initial begin
        repeat(3) @(negedge clk);
        rst=1; enable=1;
        frame(140,100,25,1); publish; location(140,100,25);
        frame(150,102,25,1); publish; location(150,102,25);
        // A spatially remote disk must not steal an existing track.
        frame(280,70,15,1); publish;
        if(!box_valid || fresh || cx<147 || cx>153) $fatal(1,"jumped to remote disk");
        frame(0,0,0,0); publish;
        if(!box_valid || fresh) $fatal(1,"second miss not held");
        frame(0,0,0,0); publish;
        if(!box_valid || fresh) $fatal(1,"third miss not held");
        frame(0,0,0,0); publish;
        if(box_valid) $fatal(1,"old box survived fourth miss");
        frame(260,85,12,1); publish; location(260,85,12);
        enable=0; repeat(3) @(negedge clk);
        if(box_valid) $fatal(1,"mode exit retained stale box");
        enable=1;
        frame(150,100,25,2); publish;
        if(box_valid) $fatal(1,"square passed circle chord test");
        frame(25,25,15,1); publish;
        if(box_valid) $fatal(1,"logo region acquired");
        frame(200,100,20,3); publish;
        if(box_valid) $fatal(1,"near-neutral weak red acquired");
        frame(310,90,25,1); publish;
        if(box_valid && (cx>=320 || cy>=180)) $fatal(1,"offscreen coordinates");
        $display("PASS actual tracking RTL: dark disk, movement, association, 3-frame coast/expiry, reacquisition, mode reset, square/logo/neutral rejection");
        $finish;
    end
    initial begin
        #20000000;
        $fatal(1,"timeout");
    end
endmodule
