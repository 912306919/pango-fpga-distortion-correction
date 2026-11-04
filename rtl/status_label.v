`timescale 1ns / 1ps
//=============================================================================
// status_label.v -- 基础任务左上角状态字
//   mode=0 (未矫正/RAW):  显示黄色 "OPERATED"
//   mode=1 (矫正后/CORR): 显示蓝色 "NO OPERATED"
//   字符 8x16，放大 SCALE 倍；用位切片索引避免除法/模运算
//=============================================================================
module status_label #(
    parameter X0    = 24,
    parameter Y0    = 24,
    parameter SCALE = 2
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [10:0] x,
    input  wire [9:0]  y,
    input  wire        mode,
    output wire        in_box,
    output wire        ink,
    output wire [7:0]  r,
    output wire [7:0]  g,
    output wire [7:0]  b
);

    localparam CELL_W      = 8  * SCALE;     // 16
    localparam CELL_H      = 16 * SCALE;     // 32
    localparam LINE0_CHARS = 11;             // "NO OPERATED"
    localparam BOX_W       = LINE0_CHARS * CELL_W;  // 176
    localparam BOX_H       = CELL_H;         // 32

    function [7:0] char_at;
        input [3:0] idx;
        case (mode)
            1'b0: case (idx)   // "OPERATED"
                4'd0: char_at = "O";
                4'd1: char_at = "P";
                4'd2: char_at = "E";
                4'd3: char_at = "R";
                4'd4: char_at = "A";
                4'd5: char_at = "T";
                4'd6: char_at = "E";
                4'd7: char_at = "D";
                default: char_at = " ";
            endcase
            1'b1: case (idx)   // "NO OPERATED"
                4'd0:  char_at = "N";
                4'd1:  char_at = "O";
                4'd2:  char_at = " ";
                4'd3:  char_at = "O";
                4'd4:  char_at = "P";
                4'd5:  char_at = "E";
                4'd6:  char_at = "R";
                4'd7:  char_at = "A";
                4'd8:  char_at = "T";
                4'd9:  char_at = "E";
                4'd10: char_at = "D";
                default: char_at = " ";
            endcase
        endcase
    endfunction

    wire in_box_w = (x >= X0) && (x < (X0 + BOX_W)) &&
                    (y >= Y0) && (y < (Y0 + BOX_H));

    wire [10:0] rx = x - X0;
    wire [9:0]  ry = y - Y0;

    wire [3:0] cc   = rx[7:4];      // 字符列号 0..10
    wire [2:0] fcol = rx[3:1];      // 字模列 0..7
    wire       half = ry[4];        // 上半(0..15) / 下半(16..31)
    wire [2:0] brow = ry[3:1];      // 半字内行 0..7

    wire [7:0]  rch  = char_at(cc);
    wire [10:0] fidx = (({3'd0, rch} - 11'd32) << 4) +
                       (half ? 11'd8 : 11'd0) + {8'd0, fcol};

    wire [7:0] fdata;
    font_8x16 u_font_8x16 (
        .font_idx  (fidx),
        .font_data (fdata)
    );

    assign in_box = in_box_w;
    assign ink    = fdata[brow];

    // 颜色：mode=0 黄色，mode=1 蓝色
    assign r = mode ? 8'h00 : 8'hff;
    assign g = mode ? 8'h00 : 8'hff;
    assign b = mode ? 8'hff : 8'h00;

endmodule
