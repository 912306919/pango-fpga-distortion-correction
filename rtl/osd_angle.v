`timescale 1ns / 1ps
//=============================================================================
// osd_angle.v -- 在 HDMI 输出画面上叠加三行 IMU 角度（调试/演示用）
//
//   角度格式：{sign, mag[14:0]}，单位 0.01 度，mag 范围 0..18000
//   显示格式：YAW  : +001.2 / PITCH: -012.3 / ROLL : +005.6
//   字符为 8x16 点阵，横向纵向各放大 2 倍（单字符 16x32 像素）
//
//   本模块只输出 in_box / ink 两个信号：
//     in_box = 1 表示当前像素落在文本框内
//     ink    = 1 表示该点需要画白色字，0 表示画黑色底
//   视频像素的覆盖由上层完成，本模块不接触视频数据。
//=============================================================================
module osd_angle #(
    parameter X0         = 24,      // 文本框左上角
    parameter Y0         = 24,
    parameter LINE_CHARS = 12,      // 每行字符数
    parameter SCALE      = 2        // 字符放大倍数
)(
    input  wire        clk,         // pix_clk
    input  wire        rst_n,
    input  wire        frame_start, // 帧起始脉冲（复位字符生成）
    input  wire [10:0] x,           // 当前输出像素坐标（须与视频对齐）
    input  wire [9:0]  y,
    input  wire [15:0] yaw,         // {sign, 0.01 度幅值}
    input  wire [15:0] pitch,
    input  wire [15:0] roll,
    output wire        in_box,
    output wire        ink
);

    localparam CELL_W = 8  * SCALE;             // 16
    localparam CELL_H = 16 * SCALE;             // 32
    localparam BOX_W  = LINE_CHARS * CELL_W;    // 192
    localparam BOX_H  = 3 * CELL_H;             // 96

    //=========================================================================
    // 1. 文本缓冲：36 个字符，帧起始后用 36 拍流水生成
    //=========================================================================
    reg [7:0] tbuf [0:35];
    reg [5:0] wp;

    wire [1:0] w_line = (wp < 6'd12) ? 2'd0 : (wp < 6'd24) ? 2'd1 : 2'd2;
    wire [3:0] w_col  = (w_line == 2'd0) ? wp[3:0] :
                        (w_line == 2'd1) ? (wp - 6'd12) : (wp - 6'd24);

    wire [15:0] w_val  = (w_line == 2'd0) ? yaw : (w_line == 2'd1) ? pitch : roll;
    wire [14:0] w_mag  = w_val[14:0];
    wire        w_sign = w_val[15];

    // 幅值拆成 3 位整数 + 1 位小数（0.01 度单位 -> 显示 0.1 度）
    wire [3:0]  d3 = w_mag / 15'd10000;
    wire [13:0] r3 = w_mag % 15'd10000;
    wire [3:0]  d2 = r3 / 14'd1000;
    wire [9:0]  r2 = r3 % 14'd1000;
    wire [3:0]  d1 = r2 / 10'd100;
    wire [6:0]  r1 = r2 % 10'd100;
    wire [3:0]  d0 = r1 / 7'd10;

    reg [7:0] w_char;
    always @(*) begin
        case (w_col)
        4'd0   : w_char = (w_line == 2'd0) ? 8'h59 : (w_line == 2'd1) ? 8'h50 : 8'h52; // Y/P/R
        4'd1   : w_char = (w_line == 2'd0) ? 8'h41 : (w_line == 2'd1) ? 8'h49 : 8'h4F; // A/I/O
        4'd2   : w_char = (w_line == 2'd0) ? 8'h57 : (w_line == 2'd1) ? 8'h54 : 8'h4C; // W/T/L
        4'd3   : w_char = (w_line == 2'd0) ? 8'h20 : (w_line == 2'd1) ? 8'h43 : 8'h4C; // 空格/C/L
        4'd4   : w_char = (w_line == 2'd1) ? 8'h48 : 8'h20;                           // H/空格
        4'd5   : w_char = 8'h3A;                                                      // ':'
        4'd6   : w_char = w_sign ? 8'h2D : 8'h2B;                                     // '-/+'
        4'd7   : w_char = 8'h30 + {4'd0, d3};
        4'd8   : w_char = 8'h30 + {4'd0, d2};
        4'd9   : w_char = 8'h30 + {4'd0, d1};
        4'd10  : w_char = 8'h2E;                                                      // '.'
        default: w_char = 8'h30 + {4'd0, d0};
        endcase
    end

    always @(posedge clk) begin
        if (!rst_n || frame_start)
            wp <= 6'd0;
        else if (wp < 6'd36) begin
            tbuf[wp] <= w_char;
            wp       <= wp + 6'd1;
        end
    end

    //=========================================================================
    // 2. 像素定位 + 字模查表
    //=========================================================================
    wire in_box_w = (x >= X0) && (x < (X0 + BOX_W)) &&
                    (y >= Y0) && (y < (Y0 + BOX_H));

    wire [10:0] rx = x - X0;
    wire [9:0]  ry = y - Y0;

    wire [3:0] cc   = rx[7:4];   // 字符列号 0..11
    wire [2:0] fcol = rx[3:1];   // 字模列 0..7
    wire [1:0] ln   = ry[6:5];   // 行号 0..2
    wire [3:0] frow = ry[4:1];   // 字模行 0..15
    wire       half = ry[4];     // 上半/下半
    wire [2:0] brow = ry[3:1];   // 半字内的行 0..7

    // 索引 = ln*12 + cc
    wire [5:0] ridx = ({4'd0, ln} << 3) + ({4'd0, ln} << 2) + {2'd0, cc};

    wire [7:0]  rch  = tbuf[ridx];
    wire [10:0] fidx = (({3'd0, rch} - 11'd32) << 4) +
                       (half ? 11'd8 : 11'd0) + {8'd0, fcol};

    wire [7:0] fdata;
    font_8x16 u_font_8x16 (
        .font_idx  (fidx),
        .font_data (fdata)
    );

    assign in_box = in_box_w;
    assign ink    = fdata[brow];

endmodule
