`timescale 1ns/1ps
module tb_red_circle_video;
    reg clk=0;
    always #5 clk=~clk;
    reg rst=0, enable=0, valid=0, sof=0, eol=0;
    reg [8:0] x=0;
    reg [7:0] y=0;
    reg [23:0] rgb=0;
    reg [23:0] pixels[0:57599];
    reg [2047:0] filename, resultname;
    integer f,i,n,status,fd;
    wire box_valid,fresh;
    wire [8:0] cx,w;
    wire [7:0] cy;
    red_circle_stream dut(
        .clk(clk),.rst_n(rst),.enable(enable),.valid(valid),.sof(sof),.eol(eol),
        .x(x),.y(y),.rgb(rgb),.box_valid(box_valid),.fresh(fresh),
        .center_x(cx),.center_y(cy),.diameter(w)
    );
    initial begin
        if(!$value$plusargs("DIR=%s",filename)) $fatal(1,"DIR required");
        if(!$value$plusargs("N=%d",n)) $fatal(1,"N required");
        $sformat(resultname,"%0s/rtl_tracking.csv",filename);
        fd=$fopen(resultname,"w");
        $fdisplay(fd,"frame,valid,fresh,x,y,width");
        repeat(3) @(negedge clk);
        rst=1; enable=1;
        for(f=0;f<n;f=f+1) begin
            $sformat(resultname,"%0s/replay_%04d.hex",filename,f);
            $readmemh(resultname,pixels);
            for(i=0;i<57600;i=i+1) begin
                @(negedge clk);
                valid=1; sof=(f==0 && i==0); eol=(i%320==319);
                x=i%320; y=i/320; rgb=pixels[i];
            end
            @(negedge clk);
            valid=0; sof=0; eol=0;
            repeat(2000) @(negedge clk);
            valid=1; sof=1; x=0; y=0; rgb=24'h303030;
            @(negedge clk);
            valid=0; sof=0;
            $fdisplay(fd,"%0d,%0d,%0d,%0d,%0d,%0d",f,box_valid,fresh,cx,cy,w);
        end
        $fclose(fd);
        $display("PASS replay completed %0d frames; inspect CSV/video for detection quality",n);
        $finish;
    end
endmodule
