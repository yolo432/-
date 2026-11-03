`timescale 1ns/1ps
module tb_red_circle_pipeline;
    reg clk=0;
    always #5 clk=~clk;
    reg rst_n=0, valid=0, sof=0, eol=0;
    reg [3:0] keys=15;
    reg [95:0] pixels=0;
    wire [95:0] result;
    wire ov,os,oe;
    integer xx,yy,lane,dx,dy,red_count,white_count;
    edge_detect_96 #(.IMG_WIDTH(320),.IMG_HEIGHT(240),
                     .KEY_DEBOUNCE_CYCLES(4)) dut (
        .I_clk(clk),.I_rst_n(rst_n),.I_button(keys),
        .I_tvalid(valid),.I_tuser(sof),.I_tlast(eol),
        .I_tready(1'b1),.I_tdata(pixels),.O_tdata(result),
        .O_tvalid(ov),.O_tuser(os),.O_tlast(oe)
    );
    task frame;
        input inspect;
        begin
            red_count=0; white_count=0;
            for(yy=0;yy<240;yy=yy+1)
                for(xx=0;xx<320;xx=xx+4) begin
                    @(negedge clk);
                    valid=1; sof=(xx==0 && yy==0); eol=(xx==316);
                    for(lane=0;lane<4;lane=lane+1) begin
                        dx=xx+lane-240; dy=yy-176;
                        pixels[95-lane*24 -:24]=(dx*dx+dy*dy<=24*24) ?
                            24'h201614 : 24'h303030;
                    end
                    @(posedge clk); #1;
                    if(!ov || os!==sof || oe!==eol) $fatal(1,"sidebands");
                    if(inspect) begin
                        for(lane=0;lane<4;lane=lane+1) begin
                            if(result[95-lane*24 -:24]==24'hff2020)
                                red_count=red_count+1;
                            if(result[95-lane*24 -:24]==24'hffffff)
                                white_count=white_count+1;
                        end
                        if(xx==240 && yy==176 && result[95:72]!==24'hffffff)
                            $fatal(1,"track center crosshair not rendered");
                        if(xx==100 && yy==100 && result!=={4{24'h484848}})
                            $fatal(1,"background brightness changed");
                    end
                end
            @(negedge clk);
            valid=0; sof=0; eol=0;
            repeat(2000) @(negedge clk);
            if(inspect && (red_count<100 || white_count<20))
                $fatal(1,"missing box/text/crosshair: red=%d white=%d",red_count,white_count);
        end
    endtask
    task press;
        input integer key;
        begin
            keys[key]=0; repeat(15) @(negedge clk);
            keys[key]=1; repeat(15) @(negedge clk);
        end
    endtask
    initial begin
        repeat(3) @(negedge clk); rst_n=1;
        press(2);
        if(dut.tracking_mode) $fatal(1,"changed without SOF");
        frame(0);
        frame(1);
        if(!dut.track_valid || !dut.track_fresh ||
           dut.track_x<58 || dut.track_x>62 ||
           dut.track_y<42 || dut.track_y>46)
            $fatal(1,"4x4 source sampling mismatch");
        press(1); frame(0);
        if(dut.tracking_mode || dut.display_mode!=3 || dut.track_valid)
            $fatal(1,"KEY2 failed to restore lamp mode/clear old track");
        $display("PASS integrated ISP: KEY3, source 4x4 sampling, frame publication, red box, white label/crosshair, natural brightness and KEY2 exit");
        $finish;
    end
endmodule
