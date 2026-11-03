`timescale 1ns/1ps
module tb_red_circle_coordinates;
    reg clk=0;
    always #5 clk=~clk;
    reg rst_n=0,valid=0,sof=0,eol=0;
    reg [3:0] keys=15;
    wire [95:0] result;
    wire ov,os,oe;
    reg [8:0] injected_x=0;
    reg [7:0] injected_y=0;
    reg injected_valid=0,injected_fresh=0;
    reg [47:0] x_text,y_text,line_text;
    reg [23:0] expected,ink;
    reg [2047:0] output_dir,filename;
    integer xx,yy,lane,c,r,g,px,fd,checked=0;
    integer panel_left,panel_top;
    reg [34:0] bitmap;
    edge_detect_96 #(.IMG_WIDTH(1280),.IMG_HEIGHT(720),
                    .KEY_DEBOUNCE_CYCLES(4)) dut (
        .I_clk(clk),.I_rst_n(rst_n),.I_button(keys),
        .I_tvalid(valid),.I_tuser(sof),.I_tlast(eol),.I_tready(1'b1),
        .I_tdata({4{24'h303030}}),.O_tdata(result),.O_tvalid(ov),
        .O_tuser(os),.O_tlast(oe)
    );
    function [34:0] glyph;
        input [7:0] ch;
        begin
            case(ch)
                "X": glyph=35'b10001_10001_01010_00100_01010_10001_10001;
                "Y": glyph=35'b10001_10001_01010_00100_00100_00100_00100;
                ":": glyph=35'b00000_00100_00100_00000_00100_00100_00000;
                "-": glyph=35'b00000_00000_00000_11111_00000_00000_00000;
                "0": glyph=35'b01110_10001_10011_10101_11001_10001_01110;
                "1": glyph=35'b00100_01100_00100_00100_00100_00100_01110;
                "2": glyph=35'b01110_10001_00001_00010_00100_01000_11111;
                "3": glyph=35'b11110_00001_00001_01110_00001_00001_11110;
                "4": glyph=35'b00010_00110_01010_10010_11111_00010_00010;
                "5": glyph=35'b11111_10000_10000_11110_00001_00001_11110;
                "6": glyph=35'b01110_10000_10000_11110_10001_10001_01110;
                "7": glyph=35'b11111_00001_00010_00100_01000_01000_01000;
                "8": glyph=35'b01110_10001_10001_01110_10001_10001_01110;
                "9": glyph=35'b01110_10001_10001_01111_00001_00001_01110;
                default: glyph=0;
            endcase
        end
    endfunction
    task press;
        input integer key;
        begin
            keys[key]=0; repeat(15) @(negedge clk);
            keys[key]=1; repeat(15) @(negedge clk);
        end
    endtask
    task frame;
        input integer center_x,center_y;
        input seen,fresh,track;
        input integer capture;
        begin
            $sformat(x_text,"X:%04d",center_x);
            $sformat(y_text,"Y:%04d",center_y);
            if(!seen) begin x_text="X:----"; y_text="Y:----"; end
            ink=(seen && !fresh) ? 24'hffb020 : 24'hffffff;
            panel_left=(center_x>=104) ? center_x-104 : 0;
            if(panel_left>1072) panel_left=1072;
            panel_top=(center_y>=100) ? center_y-96 : 4;
            fd=0;
            if(capture>=0 && output_dir!=0) begin
                $sformat(filename,"%0s/coordinates_%0d.hex",output_dir,capture);
                fd=$fopen(filename,"w");
                if(!fd) $fatal(1,"cannot create coordinate capture");
            end
            for(yy=0;yy<720;yy=yy+1)
                for(xx=0;xx<1280;xx=xx+4) begin
                    @(negedge clk);
                    valid=1; sof=(xx==0 && yy==0); eol=(xx==1276);
                    @(posedge clk); #1;
                    // Model the tracker's publication just after the SOF
                    // edge. The HUD must use it in THIS frame, not the last.
                    if(sof) begin
                        injected_x=center_x/4; injected_y=center_y/4;
                        injected_valid=seen; injected_fresh=fresh;
                    end
                    if(!ov || os!==sof || oe!==eol) $fatal(1,"shifted sidebands");
                    if(yy>=panel_top && yy<panel_top+80 &&
                       xx>=panel_left && xx<panel_left+208 &&
                       ((track && seen) || !(xx>=1048 && yy>=12 && yy<52))) begin
                        for(lane=0;lane<4;lane=lane+1) begin
                            px=xx+lane;
                            expected=(!track || !seen) ? 24'h484848 : (px<panel_left+4) ?
                                     (fresh ? 24'hff3030 : 24'hffb020) : 24'h101820;
                            if(track && seen && px>=panel_left+8 && px<panel_left+200 &&
                               ((yy>=panel_top+4 && yy<panel_top+36) ||
                                (yy>=panel_top+44 && yy<panel_top+76))) begin
                                line_text=(yy>=panel_top+44) ? y_text : x_text;
                                g=(px-panel_left-8)/32; c=((px-panel_left-8)%32)/4;
                                r=((yy>=panel_top+44) ? yy-panel_top-44 :
                                                        yy-panel_top-4)/4;
                                bitmap=glyph(line_text[47-g*8 -:8]);
                                if(c<5 && r<7 && bitmap[34-r*5-c]) expected=ink;
                            end
                            if(result[95-lane*24 -:24]!==expected)
                                $fatal(1,"center=(%0d,%0d) seen=%b fresh=%b track=%b pixel(%0d,%0d) expected=%h got=%h panel=(%0d,%0d) active=%b bounds=(%0d,%0d)",
                                    center_x,center_y,seen,fresh,track,px,yy,expected,
                                    result[95-lane*24 -:24],dut.coordinate_panel_left,
                                    dut.coordinate_panel_top,dut.coordinate_panel_active,
                                    dut.red_render_ymin,dut.track_y);
                            checked=checked+1;
                        end
                    end
                    if(fd) $fdisplay(fd,"%024h",result);
                end
            @(negedge clk); valid=0; sof=0; eol=0;
            if(fd) $fclose(fd);
            repeat(100) @(negedge clk);
        end
    endtask
    initial begin
        if(!$value$plusargs("OUT=%s",output_dir)) output_dir=0;
        // Isolate display from detector accuracy; the ordinary integration
        // regression separately drives a real red disk through the tracker.
        force dut.track_x=injected_x;
        force dut.track_y=injected_y;
        force dut.track_width=9'd8;
        force dut.track_valid=injected_valid;
        force dut.track_fresh=injected_fresh;
        repeat(3) @(negedge clk); rst_n=1;
        press(2);
        frame(640,360,1,1,1,0);
        frame(1276,716,1,1,1,1);
        frame(0,0,1,1,1,-1);
        frame(996,96,1,1,1,-1);
        frame(1000,100,1,1,1,-1);
        frame(1000,100,1,0,1,2);
        frame(1000,100,0,0,1,3);
        press(2);
        frame(432,208,1,1,0,4);
        $display("PASS coordinate tag: %0d output pixels, tag follows box, centered/clamped, edge/top limits, held/lost and mode exit",checked);
        $finish;
    end
endmodule
