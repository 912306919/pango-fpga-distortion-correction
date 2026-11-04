`timescale 1ns / 1ps
//=============================================================================
// mission3_label.v -- 左上角任务标题（8x16 字模，SCALE 倍放大）
//   用于 KEY2 控制的「高阶任务三」画面标识
//   mode=0：黄色 "BASIC MISSION"        （基础任务，task3_mode=0 时不叠加）
//   mode=1：品红 "ADVANCED MISSION 3"   （高阶任务三）
//   与 mission_label 结构一致，仅字符与颜色不同，便于区分两个高阶任务
//=============================================================================
module mission3_label #(
    parameter X0    = 24,
    parameter Y0    = 24,
    parameter SCALE = 2,
    parameter [7:0] MISSION_CHAR = "3"     // 末位字符：高阶三="3"，高阶二="2"
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
    localparam CHARS  = 18;            // "ADVANCED MISSION 3" 最长 18 字符
    localparam BOX_W  = CHARS * CELL_W;  // 288
    localparam BOX_H  = CELL_H;          // 32

    function [7:0] char_at;
        input [4:0] idx;
        case (mode)
            1'b0: case (idx)   // "BASIC MISSION"
                5'd0:  char_at = "B";
                5'd1:  char_at = "A";
                5'd2:  char_at = "S";
                5'd3:  char_at = "I";
                5'd4:  char_at = "C";
                5'd5:  char_at = " ";
                5'd6:  char_at = "M";
                5'd7:  char_at = "I";
                5'd8:  char_at = "S";
                5'd9:  char_at = "S";
                5'd10: char_at = "I";
                5'd11: char_at = "O";
                5'd12: char_at = "N";
                default: char_at = " ";
            endcase
            1'b1: case (idx)   // "ADVANCED MISSION 3"
                5'd0:  char_at = "A";
                5'd1:  char_at = "D";
                5'd2:  char_at = "V";
                5'd3:  char_at = "A";
                5'd4:  char_at = "N";
                5'd5:  char_at = "C";
                5'd6:  char_at = "E";
                5'd7:  char_at = "D";
                5'd8:  char_at = " ";
                5'd9:  char_at = "M";
                5'd10: char_at = "I";
                5'd11: char_at = "S";
                5'd12: char_at = "S";
                5'd13: char_at = "I";
                5'd14: char_at = "O";
                5'd15: char_at = "N";
                5'd16: char_at = " ";
                5'd17: char_at = MISSION_CHAR;
                default: char_at = " ";
            endcase
        endcase
    endfunction

    wire in_box_w = (x >= X0) && (x < (X0 + BOX_W)) &&
                    (y >= Y0) && (y < (Y0 + BOX_H));

    wire [10:0] rx = x - X0;
    wire [9:0]  ry = y - Y0;

    wire [4:0] cc   = rx[8:4];      // 字符列号 0..17
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

    // 颜色：基础任务黄色，高阶任务三品红（与高阶任务一青色区分）
    assign r = 8'hff;
    assign g = mode ? 8'h00 : 8'hff;
    assign b = mode ? 8'hff : 8'h00;

endmodule


//=============================================================================
// adv3_status_label -- 高阶任务三的场景读数（两行）
//
// 和 mission3_label 放在同一个文件里，是为了不往 .pds 的源文件列表里加条目
// （工程文件正被 PDS GUI 打开，改 .pds 有被覆盖的风险）。
//
// 位置默认 (24,56)，正好是 task3_mode 下 status_label / osd_angle 让出来的空位，
// 紧贴 "ADVANCED MISSION 3" 标题（y 24..55）下面，两者不重叠。
// 只在 enable=1（KEY2 选中高阶任务三）时叠加，其余状态完全不画。
//
//   第一行：档位
//     tier  0 NIGHT  / 1 DIM      -> 青色（暗部提亮）
//           2 NORMAL             -> 白色（恒等，不动画面）
//           3 BRIGHT / 4 GLARE   -> 橙色（高光压缩）
//   第二行：判档依据的实际数值（与第一行同色）
//     Y:128 G:05%    Y = 上一帧平均亮度(0..255)，G = 高光像素占比(0..99%)
//
// 为什么要显示这两个数字：OV5640 默认开着自动曝光，整幅平均亮度会被 AEC
// 钉在 90~145 一带，现场只看到 NORMAL 这几个字无法判断"强光为什么没触发"。
// 有了 Y 和 G，拿手电一照就能看到 G 到底升到多少，据此把 advanced_task3.v
// 里的 BR_GLARE 标定到合适值（当前 5%）。
//=============================================================================
module adv3_status_label #(
    parameter X0    = 24,
    parameter Y0    = 56,
    parameter SCALE = 2
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [10:0] x,
    input  wire [9:0]  y,
    input  wire [2:0]  tier,
    input  wire [7:0]  mean_y,       // 上一帧平均亮度 0..255
    input  wire [7:0]  glare_pct,    // 上一帧高光像素占比 0..100
    input  wire        enable,
    output wire        in_box,
    output wire        ink,
    output wire [7:0]  r,
    output wire [7:0]  g,
    output wire [7:0]  b
);

    localparam CELL_W = 8  * SCALE;      // 16
    localparam CELL_H = 16 * SCALE;      // 32
    localparam CHARS  = 11;              // 最长 "Y:255 G:99%"
    localparam LINES  = 2;               // 第一行档位，第二行读数
    localparam BOX_W  = CHARS * CELL_W;  // 176
    localparam BOX_H  = LINES * CELL_H;  // 64

    // 数字拆位（常数除/取模，综合成乘加，不进像素流水；每帧最多变一次）
    wire [7:0] m_hun = mean_y / 8'd100;              // 0..2
    wire [7:0] m_ten = (mean_y / 8'd10) % 8'd10;     // 0..9
    wire [7:0] m_one = mean_y % 8'd10;               // 0..9
    wire [7:0] gl    = (glare_pct > 8'd99) ? 8'd99 : glare_pct;
    wire [7:0] g_ten = gl / 8'd10;                   // 0..9
    wire [7:0] g_one = gl % 8'd10;                   // 0..9

    function [7:0] char_at;
        input [4:0] idx;
        begin
            case (tier)
                3'd0: case (idx)                 // "NIGHT "
                    5'd0: char_at = "N";
                    5'd1: char_at = "I";
                    5'd2: char_at = "G";
                    5'd3: char_at = "H";
                    5'd4: char_at = "T";
                    default: char_at = " ";
                endcase
                3'd1: case (idx)                 // "DIM   "
                    5'd0: char_at = "D";
                    5'd1: char_at = "I";
                    5'd2: char_at = "M";
                    default: char_at = " ";
                endcase
                3'd2: case (idx)                 // "NORMAL"
                    5'd0: char_at = "N";
                    5'd1: char_at = "O";
                    5'd2: char_at = "R";
                    5'd3: char_at = "M";
                    5'd4: char_at = "A";
                    5'd5: char_at = "L";
                    default: char_at = " ";
                endcase
                3'd3: case (idx)                 // "BRIGHT"
                    5'd0: char_at = "B";
                    5'd1: char_at = "R";
                    5'd2: char_at = "I";
                    5'd3: char_at = "G";
                    5'd4: char_at = "H";
                    5'd5: char_at = "T";
                    default: char_at = " ";
                endcase
                default: case (idx)              // "GLARE "
                    5'd0: char_at = "G";
                    5'd1: char_at = "L";
                    5'd2: char_at = "A";
                    5'd3: char_at = "R";
                    5'd4: char_at = "E";
                    default: char_at = " ";
                endcase
            endcase
        end
    endfunction

    // 第二行：判档依据的实际数值  "Y:128 G:05%"
    function [7:0] num_char;
        input [3:0] idx;
        begin
            case (idx)
                4'd0:  num_char = "Y";
                4'd1:  num_char = ":";
                4'd2:  num_char = 8'h30 + m_hun;   // 百位
                4'd3:  num_char = 8'h30 + m_ten;   // 十位
                4'd4:  num_char = 8'h30 + m_one;   // 个位
                4'd5:  num_char = " ";
                4'd6:  num_char = "G";
                4'd7:  num_char = ":";
                4'd8:  num_char = 8'h30 + g_ten;
                4'd9:  num_char = 8'h30 + g_one;
                4'd10: num_char = "%";
                default: num_char = " ";
            endcase
        end
    endfunction

    wire in_box_w = enable &&
                    (x >= X0) && (x < (X0 + BOX_W)) &&
                    (y >= Y0) && (y < (Y0 + BOX_H));

    wire [10:0] rx = x - X0;
    wire [9:0]  ry = y - Y0;

    wire [3:0]  cc   = rx[7:4];          // 字符列号 0..10
    wire [2:0]  fcol = rx[3:1];          // 字模列 0..7
    wire        line = ry[5];            // 行号 0=档位 1=读数
    wire        half = ry[4];            // 行内上半 / 下半
    wire [2:0]  brow = ry[3:1];          // 半字内行 0..7

    wire [7:0]  rch  = line ? num_char(cc) : char_at({1'b0, cc});
    wire [10:0] fidx = (({3'd0, rch} - 11'd32) << 4) +
                       (half ? 11'd8 : 11'd0) + {8'd0, fcol};

    wire [7:0] fdata;
    font_8x16 u_font_8x16_adv2 (
        .font_idx  (fidx),
        .font_data (fdata)
    );

    assign in_box = in_box_w;
    assign ink    = fdata[brow];

    // 颜色：提亮=青，正常=白，压暗=橙
    assign r = (tier <= 3'd1) ? 8'h00 : 8'hff;
    assign g = (tier <= 3'd1) ? 8'hff :
               (tier >= 3'd3) ? 8'h80 : 8'hff;
    assign b = (tier <= 3'd1) ? 8'hff :
               (tier >= 3'd3) ? 8'h00 : 8'hff;

endmodule


//=============================================================================
// perf_meter -- 高阶任务二：每帧流水线统计（HIT / MISS / BLACK）
//
//   统计对象是「与 bilinear_valid 同拍」的那个像素，所以三个输入必须同拍：
//     de       : 该像素显示有效（顶层接 de_display_pipe[DATA_LATENCY-3]）
//     hit      : bilinear_valid（行缓存 2x2 四拍全命中）
//     in_range : lb_in_range_d（源坐标落在 1280x720 内）
//
//   分类：!in_range -> BLK；in_range && !hit -> MISS；其余 -> HIT
//   三者之和每帧应等于 1280*720 = 921600，可用来检查统计门控是否对拍。
//
//   百分比用常数乘近似：pct = v * 1138 >> 20 ≈ v / 921.6（0..1000 千分比）
//   误差 < 0.1%，比除法器省资源。1138 是 2^20 / (1280*720/1000) 的整数近似。
//=============================================================================
module perf_meter #(
    parameter IMG_W = 1280,
    parameter IMG_H = 720
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        frame_start,
    input  wire        de,
    input  wire        hit,
    input  wire        in_range,
    output reg  [10:0] hit_pct,
    output reg  [10:0] miss_pct,
    output reg  [10:0] blk_pct
);
    reg [20:0] hit_acc, miss_acc, blk_acc;

    wire [31:0] hit_pct_w  = ({11'd0, hit_acc}  * 32'd1138) >> 20;
    wire [31:0] miss_pct_w = ({11'd0, miss_acc} * 32'd1138) >> 20;
    wire [31:0] blk_pct_w  = ({11'd0, blk_acc}  * 32'd1138) >> 20;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hit_acc  <= 21'd0;
            miss_acc <= 21'd0;
            blk_acc  <= 21'd0;
            hit_pct  <= 11'd0;
            miss_pct <= 11'd0;
            blk_pct  <= 11'd0;
        end else if (frame_start) begin
            hit_acc  <= 21'd0;
            miss_acc <= 21'd0;
            blk_acc  <= 21'd0;
            hit_pct  <= hit_pct_w[10:0];
            miss_pct <= miss_pct_w[10:0];
            blk_pct  <= blk_pct_w[10:0];
        end else if (de) begin
            if (!in_range)
                blk_acc <= blk_acc + 21'd1;
            else if (hit)
                hit_acc <= hit_acc + 21'd1;
            else
                miss_acc <= miss_acc + 21'd1;
        end
    end
endmodule


//=============================================================================
// perf_label -- 高阶任务二：右上角实时性能面板（5 行，8x16 字模 x2）
//
//   OUTPUT FPS:30     输出帧率（与左下角 INPUT FPS 对照，相等 = 不掉帧）
//   HIT : 99.8%       行缓存命中率
//   MISS:  0.1%       在画面内但行缓存未命中，回退原始像素
//   BLK :  0.5%       源坐标越界（黑边）
//   LAT : 13 CLK      矫正流水固定延迟（拍），13 拍 @37.125MHz = 0.35us
//
//   纯组合显示，不引入额外流水级；enable=0 时 in_box 恒 0，完全不画。
//=============================================================================
module perf_label #(
    parameter X0    = 880,
    parameter Y0    = 24,
    parameter SCALE = 2
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [10:0] x,
    input  wire [9:0]  y,
    input  wire        enable,
    input  wire [7:0]  fps_out,
    input  wire [10:0] hit_pct,
    input  wire [10:0] miss_pct,
    input  wire [10:0] blk_pct,
    input  wire [3:0]  lat_clk,
    output wire        in_box,
    output wire        ink,
    output wire [7:0]  r,
    output wire [7:0]  g,
    output wire [7:0]  b
);
    localparam CELL_W = 8  * SCALE;      // 16
    localparam CELL_H = 16 * SCALE;      // 32
    localparam CHARS  = 14;
    localparam LINES  = 5;
    localparam BOX_W  = CHARS * CELL_W;  // 224
    localparam BOX_H  = LINES * CELL_H;  // 160

    // ---------------- 数值拆位 ----------------
    wire [3:0] fps_t = (fps_out / 8'd10) % 8'd10;
    wire [3:0] fps_o = fps_out % 8'd10;

    wire [10:0] h_i = hit_pct  / 11'd10;      // 整数部分 0..100
    wire [3:0]  h_f = hit_pct  % 11'd10;      // 小数部分 0..9
    wire [10:0] m_i = miss_pct / 11'd10;
    wire [3:0]  m_f = miss_pct % 11'd10;
    wire [10:0] b_i = blk_pct  / 11'd10;
    wire [3:0]  b_f = blk_pct  % 11'd10;

    wire [3:0] h_hun = (h_i >= 11'd100) ? 4'd1 : 4'd0;
    wire [3:0] h_ten = (h_i / 11'd10) % 11'd10;
    wire [3:0] h_one = h_i % 11'd10;
    wire [3:0] m_hun = (m_i >= 11'd100) ? 4'd1 : 4'd0;
    wire [3:0] m_ten = (m_i / 11'd10) % 11'd10;
    wire [3:0] m_one = m_i % 11'd10;
    wire [3:0] b_hun = (b_i >= 11'd100) ? 4'd1 : 4'd0;
    wire [3:0] b_ten = (b_i / 11'd10) % 11'd10;
    wire [3:0] b_one = b_i % 11'd10;

    wire [3:0] lat_t = (lat_clk / 4'd10) % 4'd10;
    wire [3:0] lat_o = lat_clk % 4'd10;

    wire in_box_w = enable &&
                    (x >= X0) && (x < (X0 + BOX_W)) &&
                    (y >= Y0) && (y < (Y0 + BOX_H));

    wire [10:0] rx   = x - X0;
    wire [9:0]  ry   = y - Y0;
    wire [2:0]  ln   = ry[7:5];   // 行号 0..4
    wire [3:0]  cc   = rx[7:4];   // 字符列 0..13
    wire [2:0]  fcol = rx[3:1];   // 字模列 0..7
    wire        half = ry[4];     // 上半 / 下半
    wire [2:0]  brow = ry[3:1];   // 半字内行

    reg [7:0] ch;
    always @(*) begin
        ch = " ";
        case (ln)
            3'd0: case (cc)     // "OUTPUT FPS:30"
                4'd0:  ch = "O";
                4'd1:  ch = "U";
                4'd2:  ch = "T";
                4'd3:  ch = "P";
                4'd4:  ch = "U";
                4'd5:  ch = "T";
                4'd6:  ch = " ";
                4'd7:  ch = "F";
                4'd8:  ch = "P";
                4'd9:  ch = "S";
                4'd10: ch = ":";
                4'd11: ch = " ";
                4'd12: ch = 8'h30 + {4'd0, fps_t};
                4'd13: ch = 8'h30 + {4'd0, fps_o};
                default: ch = " ";
            endcase
            3'd1: case (cc)     // "HIT : 99.8%"
                4'd0:  ch = "H";
                4'd1:  ch = "I";
                4'd2:  ch = "T";
                4'd3:  ch = " ";
                4'd4:  ch = ":";
                4'd5:  ch = " ";
                4'd6:  ch = (h_hun != 4'd0) ? "1" : " ";
                4'd7:  ch = 8'h30 + {4'd0, h_ten};
                4'd8:  ch = 8'h30 + {4'd0, h_one};
                4'd9:  ch = ".";
                4'd10: ch = 8'h30 + {4'd0, h_f};
                4'd11: ch = "%";
                default: ch = " ";
            endcase
            3'd2: case (cc)     // "MISS:  0.1%"
                4'd0:  ch = "M";
                4'd1:  ch = "I";
                4'd2:  ch = "S";
                4'd3:  ch = "S";
                4'd4:  ch = ":";
                4'd5:  ch = " ";
                4'd6:  ch = (m_hun != 4'd0) ? "1" : " ";
                4'd7:  ch = 8'h30 + {4'd0, m_ten};
                4'd8:  ch = 8'h30 + {4'd0, m_one};
                4'd9:  ch = ".";
                4'd10: ch = 8'h30 + {4'd0, m_f};
                4'd11: ch = "%";
                default: ch = " ";
            endcase
            3'd3: case (cc)     // "BLK :  0.5%"
                4'd0:  ch = "B";
                4'd1:  ch = "L";
                4'd2:  ch = "K";
                4'd3:  ch = " ";
                4'd4:  ch = ":";
                4'd5:  ch = " ";
                4'd6:  ch = (b_hun != 4'd0) ? "1" : " ";
                4'd7:  ch = 8'h30 + {4'd0, b_ten};
                4'd8:  ch = 8'h30 + {4'd0, b_one};
                4'd9:  ch = ".";
                4'd10: ch = 8'h30 + {4'd0, b_f};
                4'd11: ch = "%";
                default: ch = " ";
            endcase
            3'd4: case (cc)     // "LAT : 13 CLK"
                4'd0:  ch = "L";
                4'd1:  ch = "A";
                4'd2:  ch = "T";
                4'd3:  ch = " ";
                4'd4:  ch = ":";
                4'd5:  ch = " ";
                4'd6:  ch = 8'h30 + {4'd0, lat_t};
                4'd7:  ch = 8'h30 + {4'd0, lat_o};
                4'd8:  ch = " ";
                4'd9:  ch = "C";
                4'd10: ch = "L";
                4'd11: ch = "K";
                default: ch = " ";
            endcase
            default: ch = " ";
        endcase
    end

    wire [10:0] fidx = (({3'd0, ch} - 11'd32) << 4) +
                       (half ? 11'd8 : 11'd0) + {8'd0, fcol};

    wire [7:0] fdata;
    font_8x16 u_font_8x16_perf (
        .font_idx  (fidx),
        .font_data (fdata)
    );

    assign in_box = in_box_w;
    assign ink    = fdata[brow];

    // 青色，与左上角品红标题区分
    assign r = 8'h00;
    assign g = 8'hff;
    assign b = 8'hff;

endmodule
