`timescale 1ns / 1ps
//=============================================================================
// mission_label.v -- 左上角任务标题（8x16 字模，SCALE 倍放大）
//   mode=0：黄色 "BASIC MISSION"      （基础任务）
//   mode=1：青色 "ADVANCED MISSION"   （高阶任务一）
//   用位切片索引避免除法/模运算
//=============================================================================
module mission_label #(
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

    localparam CELL_W = 8  * SCALE;    // 16
    localparam CELL_H = 16 * SCALE;    // 32
    localparam CHARS  = 16;            // "ADVANCED MISSION" 最长 16 字符
    localparam BOX_W  = CHARS * CELL_W;  // 256
    localparam BOX_H  = CELL_H;          // 32

    function [7:0] char_at;
        input [3:0] idx;
        case (mode)
            1'b0: case (idx)   // "BASIC MISSION"
                4'd0:  char_at = "B";
                4'd1:  char_at = "A";
                4'd2:  char_at = "S";
                4'd3:  char_at = "I";
                4'd4:  char_at = "C";
                4'd5:  char_at = " ";
                4'd6:  char_at = "M";
                4'd7:  char_at = "I";
                4'd8:  char_at = "S";
                4'd9:  char_at = "S";
                4'd10: char_at = "I";
                4'd11: char_at = "O";
                4'd12: char_at = "N";
                default: char_at = " ";
            endcase
            1'b1: case (idx)   // "ADVANCED MISSION"
                4'd0:  char_at = "A";
                4'd1:  char_at = "D";
                4'd2:  char_at = "V";
                4'd3:  char_at = "A";
                4'd4:  char_at = "N";
                4'd5:  char_at = "C";
                4'd6:  char_at = "E";
                4'd7:  char_at = "D";
                4'd8:  char_at = " ";
                4'd9:  char_at = "M";
                4'd10: char_at = "I";
                4'd11: char_at = "S";
                4'd12: char_at = "S";
                4'd13: char_at = "I";
                4'd14: char_at = "O";
                4'd15: char_at = "N";
                default: char_at = " ";
            endcase
        endcase
    endfunction

    wire in_box_w = (x >= X0) && (x < (X0 + BOX_W)) &&
                    (y >= Y0) && (y < (Y0 + BOX_H));

    wire [10:0] rx = x - X0;
    wire [9:0]  ry = y - Y0;

    wire [3:0] cc   = rx[7:4];      // 字符列号 0..15
    wire [2:0] fcol = rx[3:1];      // 字模列 0..7
    wire       half = ry[4];        // 上半 / 下半
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

    // 颜色：基础任务黄色，高阶任务一青色
    assign r = 8'hff;
    assign g = 8'hff;
    assign b = mode ? 8'hff : 8'h00;

endmodule
