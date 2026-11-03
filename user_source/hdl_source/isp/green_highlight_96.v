module green_highlight_96 #(
    parameter [7:0] MIN_GREEN = 8'd96,
    parameter [8:0] GREEN_MARGIN = 9'd40
) (
    input  wire        I_tvalid,
    input  wire        I_tlast,
    input  wire        I_tuser,
    input  wire [95:0] I_tdata,
    output wire        O_tvalid,
    output wire        O_tlast,
    output wire        O_tuser,
    output wire [95:0] O_tdata
);
    function is_green_pixel;
        input [23:0] pixel;
        reg [8:0] r_ext;
        reg [8:0] g_ext;
        reg [8:0] b_ext;
        begin
            r_ext = {1'b0, pixel[23:16]};
            g_ext = {1'b0, pixel[15:8]};
            b_ext = {1'b0, pixel[7:0]};
            is_green_pixel = (g_ext >= {1'b0, MIN_GREEN}) &&
                             (g_ext > (r_ext + GREEN_MARGIN)) &&
                             (g_ext > (b_ext + GREEN_MARGIN));
        end
    endfunction

    function [23:0] highlight_pixel;
        input [23:0] pixel;
        input        neighbor_green;
        begin
            if (is_green_pixel(pixel) && neighbor_green)
                highlight_pixel = 24'h00ff00;
            else
                highlight_pixel = pixel;
        end
    endfunction

    wire [23:0] pixel0 = I_tdata[95:72];
    wire [23:0] pixel1 = I_tdata[71:48];
    wire [23:0] pixel2 = I_tdata[47:24];
    wire [23:0] pixel3 = I_tdata[23:0];
    wire        green0 = is_green_pixel(pixel0);
    wire        green1 = is_green_pixel(pixel1);
    wire        green2 = is_green_pixel(pixel2);
    wire        green3 = is_green_pixel(pixel3);

    assign O_tvalid = I_tvalid;
    assign O_tlast  = I_tlast;
    assign O_tuser  = I_tuser;
    assign O_tdata  = {
        highlight_pixel(pixel0, green1),
        highlight_pixel(pixel1, green0 || green2),
        highlight_pixel(pixel2, green1 || green3),
        highlight_pixel(pixel3, green2)
    };
endmodule
