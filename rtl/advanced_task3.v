`timescale 1ns / 1ps
//=============================================================================
// advanced_task3.v -- 高阶任务三：帧级自适应 Gamma 校正（自适应色调映射）
//
// 命名说明：文件名/模块名 advanced_task3、信号 task3_mode、按键 KEY2 都是工程历史命名，
//           该功能对应赛题二的「高阶任务 3：自适应预处理」。
//
// 对应赛题二「高阶任务 3：自适应预处理」——针对强光、夜间等复杂光照环境，
// 在矫正过程中集成自适应图像增强算法，提升极端光照条件下的画面效果。
//
// ---------------------------------------------------------------------------
// 一、算法总览
//
//   本模块只做一件事：把输入像素的 R/G/B 三个通道，按同一条「随场景自适应
//   变化」的单调色调曲线重新映射。曲线由两个可调分量合成：
//
//     lut_lift[ch] —— 暗部提亮曲线（夜间 / 弱光用）
//                     ch <= 176 时 = 176*(ch/176)^0.45，ch > 176 时保持原值
//                     => 只抬暗部，176..255 的高光完全不动
//
//     lut_comp[ch] —— 高光压缩曲线（强光 / 过曝用）
//                     ch <= 96 时保持原值，
//                     ch >  96 时 = 96 + 136*((ch-96)/159)^1.15
//                     终点是 232 而不是 255：这样连最亮的"死白"也会被压下来，
//                     且全程斜率 < 1，不会出现"越靠近 255 反而被拉伸回去"的问题。
//                     => 只压高光，0..96 的暗部完全不动
//
//   两条曲线都不是"全量程 gamma"，而是带拐点的半程曲线，因此无论怎么混合，
//   都不会把画面的另一端拖灰——这是与"固定 gamma=0.5 全量程曲线"最大的区别。
//
//   合成由一个有符号强度 s ∈ [-255, +255] 控制（每帧更新一次）：
//
//     s > 0 -> 向 lut_lift 混合（数值越大，暗部提得越狠）
//     s = 0 -> 位精确恒等（输出 == 输入，一个 bit 都不改）
//     s < 0 -> 向 lut_comp 混合（数值越负，高光压得越狠）
//
//     逐像素运算（三个通道各自独立，共用同一 s）：
//         d   = (s>0 ? lut_lift[ch] : lut_comp[ch]) - ch
//         out = clamp(ch + (d * |s|) >> 8, 0, 255)
//
//   s=0 时 d*0=0 ⇒ out = ch，位精确恒等；这一点由 testbench 逐值验证。
//   三个通道共用同一 s，等于"按亮度整体缩放"，因此不会产生色偏（保色调）。
//
// ---------------------------------------------------------------------------
// 二、"自适应"从哪来（帧级场景分析）
//
//   每帧统计三个量（只在 enable=1 时累计，不修改任何其他任务的状态）：
//     sum_y    : 整帧亮度累加和         —— 反映平均亮度
//     pix_cnt  : 有效像素计数           —— 用于判断该帧是否完整
//     brt_cnt  : 亮像素（y > 215）计数  —— 反映高光过曝比例
//
//   帧起始（frame_start 上升沿）用上一帧的统计结果重新定档：
//     平均亮度 < 50        -> 0 NIGHT ，s 目标 = +255（强提亮）
//     平均亮度 < 90        -> 1 DIM   ，s 目标 = +170
//     平均亮度 < 135       -> 2 NORMAL，s 目标 =    0（恒等，不动画面）
//     平均亮度 < 170       -> 3 BRIGHT，s 目标 = -170
//     其余                 -> 4 GLARE ，s 目标 = -255（强压高光）
//     高光像素占比 >= 5%   -> 直接判强光（优先于上面的分档）
//
//   注：2026-11-03 把 BRIGHT 的门限从 150 下调到 135、GLARE 从 190 下调到 170，
//       因为实机上"强光档（橙色读数）"出现得太晚。若正常室内就被判成 BRIGHT、
//       画面发闷，把 TH_NORM 调回 145~150 即可（两行 localparam，改完重综合）。
//
//   ★ 为什么高光占比阈值是 5%（而不是最初的 20%）：
//     OV5640 默认开着自动曝光 —— reg_config.v 里写了 0x3503 = 0x00，
//     而且整份配置没有写 0x3501/0x3502（曝光）或 0x3508/0x3509（增益），
//     所以曝光完全由 AEC 闭环控制。AEC 的作用就是把「整幅平均亮度」压回
//     它的目标值附近（实测稳定在 90~145）。
//     后果：拿手电照镜头时，AEC 立刻降低曝光，平均值几乎不动，只有光斑被
//     截断成 255 —— 用平均亮度永远判不出强光（旧阈值 20% 时，普通光斑只占
//     画面 5%~15%，同样够不到）。这时唯一还留着的信息就是「被截断的高光
//     面积」，所以强光判据以它为主，阈值取 5%。
//     若现场发现「正常光照下 GL 读数就 >= 5%」（例如窗外大逆光），说明 5%
//     偏敏感，把 BR_GLARE 改到 8~10% 即可；屏幕第二行的 GL 读数就是给现场
//     标定这个阈值用的。
//
//   档位号 tier 与两个统计读数通过输出端口送到屏幕左上角（adv3_status_label）：
//     第一行 NIGHT / DIM / NORMAL / BRIGHT / GLARE
//     第二行 Y:128 G:05%   —— Y = 上一帧平均亮度，G = 高光像素占比
//   现场可以直接看到被判成哪一档、以及判档依据的实际数值。
//
//   判定阈值写成「整帧亮度和」的比较（PIX_PER_FRAME × 平均亮度），全程只有
//   比较器、没有除法器，也没有乘法器，省资源且不含长组合路径。
//   （mean_y / glare_pct 两个读数用「常数乘倒 + 移位」实现，每帧只算一次。）
//
//   为避免档位在阈值附近来回跳、画面闪，做了两层平滑：
//     1) 档位迟滞：新档位必须连续 3 帧都成立才真正切换；
//     2) s 不做阶跃，每帧向目标逼近 1/8（一阶 IIR，720P@30 下约 0.9 s 收敛）。
//
// ---------------------------------------------------------------------------
// 三、对基础任务 / 高阶任务一的隔离保证
//
//   1. enable=0（KEY2 未进入高阶任务三）时，输出寄存器直接写 pixel_in，
//      valid 直连，完全不经过任何算法逻辑，且延迟仍为 1 拍。
//   2. enable=0 时 s_cur 被强制清 0，统计计数器也清零，tier 复位到 2；
//      重新进入时不残留上一次的强度，也不会先闪一下上一帧的亮度读数。
//   3. 本模块延迟恒为 1 拍（与改造前一致），顶层 DATA_LATENCY=13 的显示
//      时序对齐不需要任何改动。
//   4. 正常光照下 s 收敛到 0，此时映射位精确恒等，即使 enable=1 画面也
//      不变——"只在极端光照下介入"。
//
// ---------------------------------------------------------------------------
// 四、时序 / 资源
//
//   组合查表 + 1 级乘法 + 1 级输出寄存，吞吐 1 像素/时钟，延迟 1 拍。
//   两张 256x8 表（LUTRAM 实现，可组合读），每表 3 个读口（R/G/B 各一个）。
//   3 个 9x9 有符号乘法器（每通道一个），无除法器。
//   统计通路：2 个 32/24 位累加器 + 几个比较器，每帧只动一次。
//   读数通路：2 个「常数乘 + 移位」（MEAN_M=291，会退化成 32x9 乘法），
//             同样每帧只算一次，不进像素流水，不影响 1 拍吞吐。
//=============================================================================
module advanced_task3 #(
    parameter PIX_WIDTH = 16,
    parameter IMG_W     = 1280,
    parameter IMG_H     = 720
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  frame_start,   // 显示域帧头（1 拍脉冲）
    input  wire                  enable,        // 1 = 高阶任务三（KEY2 选中）
    input  wire                  valid_in,
    input  wire [10:0]           x,             // 当前像素坐标（保留，本算法不需要）
    input  wire [9:0]            y,
    input  wire [PIX_WIDTH-1:0]  pixel_in,
    output reg                   valid_out,
    output reg  [PIX_WIDTH-1:0]  pixel_out,
    // 当前场景档位（只给屏幕读数用，不参与像素通路）
    //   0=NIGHT 1=DIM 2=NORMAL 3=BRIGHT 4=GLARE
    output reg  [2:0]            tier,
    // 判档依据的两个读数（只给屏幕读数用，不参与像素通路）
    //   mean_y    : 上一帧平均亮度  0..255
    //   glare_pct : 上一帧高光像素（y>215）占比 0..100（%）
    output reg  [7:0]            mean_y,
    output reg  [7:0]            glare_pct
);

    //-------------------------------------------------------------------------
    // 0. RGB565 拆分与亮度
    //-------------------------------------------------------------------------
    wire [4:0] r5 = pixel_in[15:11];
    wire [5:0] g6 = pixel_in[10:5];
    wire [4:0] b5 = pixel_in[4:0];

    // 位复制扩展到 8bit（与 bilinear_interp 一致，保证 s=0 时能原样还原）
    wire [7:0] r8 = {r5, r5[4:2]};
    wire [7:0] g8 = {g6, g6[5:4]};
    wire [7:0] b8 = {b5, b5[4:2]};

    // BT.601 亮度 Q8：0.299R + 0.587G + 0.114B（系数 ×256 取整）
    // 三项系数和恰为 256，最大值 255*256 = 65280，16 位足够
    wire [15:0] y_sum16 = 16'd77 * r8 + 16'd150 * g8 + 16'd29 * b8;
    wire [7:0]  y8      = y_sum16[15:8];

    //-------------------------------------------------------------------------
    // 1. 帧级统计（只在 enable=1 时工作）
    //-------------------------------------------------------------------------
    localparam integer PIX_PER_FRAME = IMG_W * IMG_H;

    // 判定阈值：整帧亮度和 = 像素数 × 目标平均亮度（编译期常量，无除法器）
    //   2026-11-03 下调：TH_NORM 150→135、TH_BRIGHT 190→170
    //   原因：强光档（橙色读数）在实机上出现得太晚，手电照上去还是 NORMAL。
    //   注意副作用：正常室内若 Y 读数已经到 135 以上，会提前进 BRIGHT 并开始轻度
    //   压高光（画面略闷）。屏幕第二行的 Y 就是给这里标定用的，改完重综合再看。
    localparam [31:0] TH_NIGHT  = PIX_PER_FRAME * 50;
    localparam [31:0] TH_DIM    = PIX_PER_FRAME * 90;
    localparam [31:0] TH_NORM   = PIX_PER_FRAME * 135;
    localparam [31:0] TH_BRIGHT = PIX_PER_FRAME * 170;
    // 高光像素占比阈值（5%，原因见文件头「为什么高光占比阈值是 5%」）
    // 与「帧是否完整」下限（75%）
    localparam [23:0] BR_GLARE  = (PIX_PER_FRAME * 5 + 99) / 100;
    localparam [23:0] CNT_MIN   = (PIX_PER_FRAME * 3) / 4;

    //---------------------------------------------------------------------
    // 显示用倒数常数（每帧只用一次，不参与像素通路）
    //   mean = sum_y / PIX_PER_FRAME ，用 sum_y * MEAN_M >> MEAN_S 近似
    //   MEAN_S/MEAN_M 在 elaboration 阶段由 PIX_PER_FRAME 算出，
    //   换分辨率（改 IMG_W/IMG_H）时不用动这里。
    //   1280x720 时 MEAN_M = 291，误差 0.09%，最大 255 -> 255（四舍五入）。
    //---------------------------------------------------------------------
    localparam integer MEAN_S = 28;
    localparam [31:0]  MEAN_M = ((32'd1 << MEAN_S) + PIX_PER_FRAME / 2) / PIX_PER_FRAME;
    localparam [63:0]  MEAN_R = (64'd1 << (MEAN_S - 1));

    reg [31:0] sum_y;
    reg [23:0] pix_cnt;
    reg [23:0] brt_cnt;

    wire stat_en = enable && valid_in;
    wire brt_now = (y8 > 8'd215);

    // frame_start 上升沿检测：帧头是 1 拍脉冲，这里再取一次沿，避免上游把它
    // 拉成电平（或脉冲宽度异常）时统计被清零/重载两次。
    reg  frame_start_d;
    wire frame_rise;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) frame_start_d <= 1'b0;
        else        frame_start_d <= frame_start;
    end
    assign frame_rise = frame_start && !frame_start_d;

    //-------------------------------------------------------------------------
    // 2. 场景定档 + 强度平滑
    //-------------------------------------------------------------------------
    reg signed [9:0] s_cur;              // 当前强度，-255 .. +255

    // 帧不完整（上电首批帧、或 enable 刚打开）时保持原强度，不误判
    wire frame_ok = (pix_cnt >= CNT_MIN);

    // 目标档位：高光占比优先，其次按平均亮度分档
    wire [2:0] tier_new =
        !frame_ok             ? tier :
        (brt_cnt >= BR_GLARE) ? 3'd4 :                  // GLARE（过曝面积大）
        (sum_y < TH_NIGHT)    ? 3'd0 :                  // NIGHT
        (sum_y < TH_DIM)      ? 3'd1 :                  // DIM
        (sum_y < TH_NORM)     ? 3'd2 :                  // NORMAL
        (sum_y < TH_BRIGHT)   ? 3'd3 :                  // BRIGHT
                                3'd4;                   // GLARE

    // 档位迟滞：新档位必须连续 TIER_HOLD+1 帧都成立，才真正切换。
    // 平均亮度在阈值附近抖动（149/151 来回）时，屏幕读数不会 NORMAL/BRIGHT
    // 乱跳，s 也不会跟着抖。只有「确认后」的 tier_next 才去驱动强度，
    // 因此屏幕读数与当前强度始终一致。
    localparam [1:0] TIER_HOLD  = 2'd2;          // 2 => 连续 3 帧确认
    reg  [1:0]       tier_hold;
    wire             tier_switch = (tier_new != tier) && (tier_hold >= TIER_HOLD);
    wire [2:0]       tier_next   = tier_switch ? tier_new : tier;

    // 档位 -> 目标强度（用确认后的 tier_next）
    wire signed [9:0] s_new =
        (tier_next == 3'd0) ?  10'sd255 :
        (tier_next == 3'd1) ?  10'sd170 :
        (tier_next == 3'd2) ?  10'sd0   :
        (tier_next == 3'd3) ? -10'sd170 :
                              -10'sd255;

    wire signed [10:0] s_delta = s_new - s_cur;
    // |s_delta|：判断是否已足够接近目标
    wire signed [10:0] s_abs   = s_delta[10] ? -s_delta : s_delta;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_cur     <= 10'sd0;
            tier      <= 3'd2;              // 上电先当正常光照，画面不动
            tier_hold <= 2'd0;
        end else if (!enable) begin
            // 关闭高阶任务三：强度归零、档位复位，再次进入时从头适应
            s_cur     <= 10'sd0;
            tier      <= 3'd2;
            tier_hold <= 2'd0;
        end else if (frame_rise) begin
            // 每帧向目标逼近 1/8（一阶 IIR），约 24 帧收敛，换场景平滑不闪。
            // 注意差量小于 8 时 (s_delta >>> 3) 会被截断成 0，导致强度永远停在
            // 目标附近而到不了目标（例如夜间切回正常光照会卡在 s=3~7，破坏
            // s=0 的位精确恒等）。因此接近后直接吸附到目标值，
            // 保证 s 能精确等于 0 / ±170 / ±255。
            s_cur <= (s_abs < 11'sd8) ? s_new : (s_cur + (s_delta >>> 3));
            if (tier_switch) begin
                tier      <= tier_new;
                tier_hold <= 2'd0;
            end else if (tier_new != tier) begin
                tier_hold <= tier_hold + 2'd1;      // 还在确认中，最多到 2
            end else begin
                tier_hold <= 2'd0;                  // 档位没变，重新计数
            end
        end
    end

    //-------------------------------------------------------------------------
    // 2.1 帧级亮度读数（只给屏幕，不参与像素通路）
    //     mean_y    = sum_y / PIX_PER_FRAME        —— 平均亮度 0..255
    //     glare_pct = 100 * brt_cnt / PIX_PER_FRAME —— 高光占比 0..100
    //   两者都在 frame_rise 那拍用上一帧的统计刷新一次，所以屏幕上的数字
    //   就是「刚过去那一帧」的真实数值，可以直接拿来标定阈值。
    //   归一化用的是理论像素数 PIX_PER_FRAME（行缓存 miss 的像素不计入
    //   pix_cnt），失配率 <1% 时误差 <1 个码值，可以忽略。
    //   乘法都显式扩到 64 位再做，避免 32 位表达式中间结果溢出。
    //-------------------------------------------------------------------------
    wire [31:0] brt_x100   = {8'd0, brt_cnt} * 32'd100;                 // ≤ 9.2e7
    wire [63:0] mean_prod  = ({32'd0, sum_y}   * {32'd0, MEAN_M}) + MEAN_R;
    wire [63:0] mean_int   = mean_prod  >> MEAN_S;
    wire [7:0]  mean_w     = (mean_int  > 64'd255) ? 8'd255 : mean_int[7:0];

    wire [63:0] glare_prod = ({32'd0, brt_x100} * {32'd0, MEAN_M}) + MEAN_R;
    wire [63:0] glare_int  = glare_prod >> MEAN_S;
    wire [7:0]  glare_w    = (glare_int > 64'd100) ? 8'd100 : glare_int[7:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mean_y    <= 8'd0;
            glare_pct <= 8'd0;
        end else if (!enable) begin
            mean_y    <= 8'd0;
            glare_pct <= 8'd0;
        end else if (frame_rise && frame_ok) begin
            // frame_ok 用上一帧的 pix_cnt 判断；帧不完整时保持旧读数
            mean_y    <= mean_w;
            glare_pct <= glare_w;
        end
    end

    // 强度拆成「符号 + 幅值」，供逐像素混合使用
    wire        s_pos = ~s_cur[9];
    wire [7:0]  s_mag = s_pos ? s_cur[7:0] : (~s_cur[7:0] + 8'd1);

    //-------------------------------------------------------------------------
    // 3. 色调曲线表
    //    由 tools/gen_advanced_task3_lut.py 生成，可用该脚本重新标定/重采样
    //-------------------------------------------------------------------------
    reg [7:0] lut_lift [0:255];
    reg [7:0] lut_comp [0:255];

    initial begin
    // lut_lift
        lut_lift[  0] = 8'h00; lut_lift[  1] = 8'h11; lut_lift[  2] = 8'h17; lut_lift[  3] = 8'h1C;
        lut_lift[  4] = 8'h20; lut_lift[  5] = 8'h23; lut_lift[  6] = 8'h26; lut_lift[  7] = 8'h29;
        lut_lift[  8] = 8'h2C; lut_lift[  9] = 8'h2E; lut_lift[ 10] = 8'h30; lut_lift[ 11] = 8'h33;
        lut_lift[ 12] = 8'h35; lut_lift[ 13] = 8'h36; lut_lift[ 14] = 8'h38; lut_lift[ 15] = 8'h3A;
        lut_lift[ 16] = 8'h3C; lut_lift[ 17] = 8'h3D; lut_lift[ 18] = 8'h3F; lut_lift[ 19] = 8'h41;
        lut_lift[ 20] = 8'h42; lut_lift[ 21] = 8'h44; lut_lift[ 22] = 8'h45; lut_lift[ 23] = 8'h46;
        lut_lift[ 24] = 8'h48; lut_lift[ 25] = 8'h49; lut_lift[ 26] = 8'h4A; lut_lift[ 27] = 8'h4C;
        lut_lift[ 28] = 8'h4D; lut_lift[ 29] = 8'h4E; lut_lift[ 30] = 8'h4F; lut_lift[ 31] = 8'h51;
        lut_lift[ 32] = 8'h52; lut_lift[ 33] = 8'h53; lut_lift[ 34] = 8'h54; lut_lift[ 35] = 8'h55;
        lut_lift[ 36] = 8'h56; lut_lift[ 37] = 8'h57; lut_lift[ 38] = 8'h58; lut_lift[ 39] = 8'h59;
        lut_lift[ 40] = 8'h5A; lut_lift[ 41] = 8'h5B; lut_lift[ 42] = 8'h5C; lut_lift[ 43] = 8'h5D;
        lut_lift[ 44] = 8'h5E; lut_lift[ 45] = 8'h5F; lut_lift[ 46] = 8'h60; lut_lift[ 47] = 8'h61;
        lut_lift[ 48] = 8'h62; lut_lift[ 49] = 8'h63; lut_lift[ 50] = 8'h64; lut_lift[ 51] = 8'h65;
        lut_lift[ 52] = 8'h66; lut_lift[ 53] = 8'h67; lut_lift[ 54] = 8'h67; lut_lift[ 55] = 8'h68;
        lut_lift[ 56] = 8'h69; lut_lift[ 57] = 8'h6A; lut_lift[ 58] = 8'h6B; lut_lift[ 59] = 8'h6C;
        lut_lift[ 60] = 8'h6C; lut_lift[ 61] = 8'h6D; lut_lift[ 62] = 8'h6E; lut_lift[ 63] = 8'h6F;
        lut_lift[ 64] = 8'h70; lut_lift[ 65] = 8'h70; lut_lift[ 66] = 8'h71; lut_lift[ 67] = 8'h72;
        lut_lift[ 68] = 8'h73; lut_lift[ 69] = 8'h73; lut_lift[ 70] = 8'h74; lut_lift[ 71] = 8'h75;
        lut_lift[ 72] = 8'h76; lut_lift[ 73] = 8'h76; lut_lift[ 74] = 8'h77; lut_lift[ 75] = 8'h78;
        lut_lift[ 76] = 8'h79; lut_lift[ 77] = 8'h79; lut_lift[ 78] = 8'h7A; lut_lift[ 79] = 8'h7B;
        lut_lift[ 80] = 8'h7B; lut_lift[ 81] = 8'h7C; lut_lift[ 82] = 8'h7D; lut_lift[ 83] = 8'h7D;
        lut_lift[ 84] = 8'h7E; lut_lift[ 85] = 8'h7F; lut_lift[ 86] = 8'h80; lut_lift[ 87] = 8'h80;
        lut_lift[ 88] = 8'h81; lut_lift[ 89] = 8'h81; lut_lift[ 90] = 8'h82; lut_lift[ 91] = 8'h83;
        lut_lift[ 92] = 8'h83; lut_lift[ 93] = 8'h84; lut_lift[ 94] = 8'h85; lut_lift[ 95] = 8'h85;
        lut_lift[ 96] = 8'h86; lut_lift[ 97] = 8'h87; lut_lift[ 98] = 8'h87; lut_lift[ 99] = 8'h88;
        lut_lift[100] = 8'h88; lut_lift[101] = 8'h89; lut_lift[102] = 8'h8A; lut_lift[103] = 8'h8A;
        lut_lift[104] = 8'h8B; lut_lift[105] = 8'h8B; lut_lift[106] = 8'h8C; lut_lift[107] = 8'h8D;
        lut_lift[108] = 8'h8D; lut_lift[109] = 8'h8E; lut_lift[110] = 8'h8E; lut_lift[111] = 8'h8F;
        lut_lift[112] = 8'h90; lut_lift[113] = 8'h90; lut_lift[114] = 8'h91; lut_lift[115] = 8'h91;
        lut_lift[116] = 8'h92; lut_lift[117] = 8'h92; lut_lift[118] = 8'h93; lut_lift[119] = 8'h94;
        lut_lift[120] = 8'h94; lut_lift[121] = 8'h95; lut_lift[122] = 8'h95; lut_lift[123] = 8'h96;
        lut_lift[124] = 8'h96; lut_lift[125] = 8'h97; lut_lift[126] = 8'h97; lut_lift[127] = 8'h98;
        lut_lift[128] = 8'h99; lut_lift[129] = 8'h99; lut_lift[130] = 8'h9A; lut_lift[131] = 8'h9A;
        lut_lift[132] = 8'h9B; lut_lift[133] = 8'h9B; lut_lift[134] = 8'h9C; lut_lift[135] = 8'h9C;
        lut_lift[136] = 8'h9D; lut_lift[137] = 8'h9D; lut_lift[138] = 8'h9E; lut_lift[139] = 8'h9E;
        lut_lift[140] = 8'h9F; lut_lift[141] = 8'h9F; lut_lift[142] = 8'hA0; lut_lift[143] = 8'hA0;
        lut_lift[144] = 8'hA1; lut_lift[145] = 8'hA1; lut_lift[146] = 8'hA2; lut_lift[147] = 8'hA2;
        lut_lift[148] = 8'hA3; lut_lift[149] = 8'hA3; lut_lift[150] = 8'hA4; lut_lift[151] = 8'hA4;
        lut_lift[152] = 8'hA5; lut_lift[153] = 8'hA5; lut_lift[154] = 8'hA6; lut_lift[155] = 8'hA6;
        lut_lift[156] = 8'hA7; lut_lift[157] = 8'hA7; lut_lift[158] = 8'hA8; lut_lift[159] = 8'hA8;
        lut_lift[160] = 8'hA9; lut_lift[161] = 8'hA9; lut_lift[162] = 8'hAA; lut_lift[163] = 8'hAA;
        lut_lift[164] = 8'hAA; lut_lift[165] = 8'hAB; lut_lift[166] = 8'hAB; lut_lift[167] = 8'hAC;
        lut_lift[168] = 8'hAC; lut_lift[169] = 8'hAD; lut_lift[170] = 8'hAD; lut_lift[171] = 8'hAE;
        lut_lift[172] = 8'hAE; lut_lift[173] = 8'hAF; lut_lift[174] = 8'hAF; lut_lift[175] = 8'hB0;
        lut_lift[176] = 8'hB0; lut_lift[177] = 8'hB1; lut_lift[178] = 8'hB2; lut_lift[179] = 8'hB3;
        lut_lift[180] = 8'hB4; lut_lift[181] = 8'hB5; lut_lift[182] = 8'hB6; lut_lift[183] = 8'hB7;
        lut_lift[184] = 8'hB8; lut_lift[185] = 8'hB9; lut_lift[186] = 8'hBA; lut_lift[187] = 8'hBB;
        lut_lift[188] = 8'hBC; lut_lift[189] = 8'hBD; lut_lift[190] = 8'hBE; lut_lift[191] = 8'hBF;
        lut_lift[192] = 8'hC0; lut_lift[193] = 8'hC1; lut_lift[194] = 8'hC2; lut_lift[195] = 8'hC3;
        lut_lift[196] = 8'hC4; lut_lift[197] = 8'hC5; lut_lift[198] = 8'hC6; lut_lift[199] = 8'hC7;
        lut_lift[200] = 8'hC8; lut_lift[201] = 8'hC9; lut_lift[202] = 8'hCA; lut_lift[203] = 8'hCB;
        lut_lift[204] = 8'hCC; lut_lift[205] = 8'hCD; lut_lift[206] = 8'hCE; lut_lift[207] = 8'hCF;
        lut_lift[208] = 8'hD0; lut_lift[209] = 8'hD1; lut_lift[210] = 8'hD2; lut_lift[211] = 8'hD3;
        lut_lift[212] = 8'hD4; lut_lift[213] = 8'hD5; lut_lift[214] = 8'hD6; lut_lift[215] = 8'hD7;
        lut_lift[216] = 8'hD8; lut_lift[217] = 8'hD9; lut_lift[218] = 8'hDA; lut_lift[219] = 8'hDB;
        lut_lift[220] = 8'hDC; lut_lift[221] = 8'hDD; lut_lift[222] = 8'hDE; lut_lift[223] = 8'hDF;
        lut_lift[224] = 8'hE0; lut_lift[225] = 8'hE1; lut_lift[226] = 8'hE2; lut_lift[227] = 8'hE3;
        lut_lift[228] = 8'hE4; lut_lift[229] = 8'hE5; lut_lift[230] = 8'hE6; lut_lift[231] = 8'hE7;
        lut_lift[232] = 8'hE8; lut_lift[233] = 8'hE9; lut_lift[234] = 8'hEA; lut_lift[235] = 8'hEB;
        lut_lift[236] = 8'hEC; lut_lift[237] = 8'hED; lut_lift[238] = 8'hEE; lut_lift[239] = 8'hEF;
        lut_lift[240] = 8'hF0; lut_lift[241] = 8'hF1; lut_lift[242] = 8'hF2; lut_lift[243] = 8'hF3;
        lut_lift[244] = 8'hF4; lut_lift[245] = 8'hF5; lut_lift[246] = 8'hF6; lut_lift[247] = 8'hF7;
        lut_lift[248] = 8'hF8; lut_lift[249] = 8'hF9; lut_lift[250] = 8'hFA; lut_lift[251] = 8'hFB;
        lut_lift[252] = 8'hFC; lut_lift[253] = 8'hFD; lut_lift[254] = 8'hFE; lut_lift[255] = 8'hFF;

    // lut_comp
        lut_comp[  0] = 8'h00; lut_comp[  1] = 8'h01; lut_comp[  2] = 8'h02; lut_comp[  3] = 8'h03;
        lut_comp[  4] = 8'h04; lut_comp[  5] = 8'h05; lut_comp[  6] = 8'h06; lut_comp[  7] = 8'h07;
        lut_comp[  8] = 8'h08; lut_comp[  9] = 8'h09; lut_comp[ 10] = 8'h0A; lut_comp[ 11] = 8'h0B;
        lut_comp[ 12] = 8'h0C; lut_comp[ 13] = 8'h0D; lut_comp[ 14] = 8'h0E; lut_comp[ 15] = 8'h0F;
        lut_comp[ 16] = 8'h10; lut_comp[ 17] = 8'h11; lut_comp[ 18] = 8'h12; lut_comp[ 19] = 8'h13;
        lut_comp[ 20] = 8'h14; lut_comp[ 21] = 8'h15; lut_comp[ 22] = 8'h16; lut_comp[ 23] = 8'h17;
        lut_comp[ 24] = 8'h18; lut_comp[ 25] = 8'h19; lut_comp[ 26] = 8'h1A; lut_comp[ 27] = 8'h1B;
        lut_comp[ 28] = 8'h1C; lut_comp[ 29] = 8'h1D; lut_comp[ 30] = 8'h1E; lut_comp[ 31] = 8'h1F;
        lut_comp[ 32] = 8'h20; lut_comp[ 33] = 8'h21; lut_comp[ 34] = 8'h22; lut_comp[ 35] = 8'h23;
        lut_comp[ 36] = 8'h24; lut_comp[ 37] = 8'h25; lut_comp[ 38] = 8'h26; lut_comp[ 39] = 8'h27;
        lut_comp[ 40] = 8'h28; lut_comp[ 41] = 8'h29; lut_comp[ 42] = 8'h2A; lut_comp[ 43] = 8'h2B;
        lut_comp[ 44] = 8'h2C; lut_comp[ 45] = 8'h2D; lut_comp[ 46] = 8'h2E; lut_comp[ 47] = 8'h2F;
        lut_comp[ 48] = 8'h30; lut_comp[ 49] = 8'h31; lut_comp[ 50] = 8'h32; lut_comp[ 51] = 8'h33;
        lut_comp[ 52] = 8'h34; lut_comp[ 53] = 8'h35; lut_comp[ 54] = 8'h36; lut_comp[ 55] = 8'h37;
        lut_comp[ 56] = 8'h38; lut_comp[ 57] = 8'h39; lut_comp[ 58] = 8'h3A; lut_comp[ 59] = 8'h3B;
        lut_comp[ 60] = 8'h3C; lut_comp[ 61] = 8'h3D; lut_comp[ 62] = 8'h3E; lut_comp[ 63] = 8'h3F;
        lut_comp[ 64] = 8'h40; lut_comp[ 65] = 8'h41; lut_comp[ 66] = 8'h42; lut_comp[ 67] = 8'h43;
        lut_comp[ 68] = 8'h44; lut_comp[ 69] = 8'h45; lut_comp[ 70] = 8'h46; lut_comp[ 71] = 8'h47;
        lut_comp[ 72] = 8'h48; lut_comp[ 73] = 8'h49; lut_comp[ 74] = 8'h4A; lut_comp[ 75] = 8'h4B;
        lut_comp[ 76] = 8'h4C; lut_comp[ 77] = 8'h4D; lut_comp[ 78] = 8'h4E; lut_comp[ 79] = 8'h4F;
        lut_comp[ 80] = 8'h50; lut_comp[ 81] = 8'h51; lut_comp[ 82] = 8'h52; lut_comp[ 83] = 8'h53;
        lut_comp[ 84] = 8'h54; lut_comp[ 85] = 8'h55; lut_comp[ 86] = 8'h56; lut_comp[ 87] = 8'h57;
        lut_comp[ 88] = 8'h58; lut_comp[ 89] = 8'h59; lut_comp[ 90] = 8'h5A; lut_comp[ 91] = 8'h5B;
        lut_comp[ 92] = 8'h5C; lut_comp[ 93] = 8'h5D; lut_comp[ 94] = 8'h5E; lut_comp[ 95] = 8'h5F;
        lut_comp[ 96] = 8'h60; lut_comp[ 97] = 8'h60; lut_comp[ 98] = 8'h61; lut_comp[ 99] = 8'h61;
        lut_comp[100] = 8'h62; lut_comp[101] = 8'h63; lut_comp[102] = 8'h63; lut_comp[103] = 8'h64;
        lut_comp[104] = 8'h64; lut_comp[105] = 8'h65; lut_comp[106] = 8'h66; lut_comp[107] = 8'h66;
        lut_comp[108] = 8'h67; lut_comp[109] = 8'h68; lut_comp[110] = 8'h68; lut_comp[111] = 8'h69;
        lut_comp[112] = 8'h6A; lut_comp[113] = 8'h6A; lut_comp[114] = 8'h6B; lut_comp[115] = 8'h6C;
        lut_comp[116] = 8'h6D; lut_comp[117] = 8'h6D; lut_comp[118] = 8'h6E; lut_comp[119] = 8'h6F;
        lut_comp[120] = 8'h6F; lut_comp[121] = 8'h70; lut_comp[122] = 8'h71; lut_comp[123] = 8'h72;
        lut_comp[124] = 8'h72; lut_comp[125] = 8'h73; lut_comp[126] = 8'h74; lut_comp[127] = 8'h75;
        lut_comp[128] = 8'h76; lut_comp[129] = 8'h76; lut_comp[130] = 8'h77; lut_comp[131] = 8'h78;
        lut_comp[132] = 8'h79; lut_comp[133] = 8'h79; lut_comp[134] = 8'h7A; lut_comp[135] = 8'h7B;
        lut_comp[136] = 8'h7C; lut_comp[137] = 8'h7D; lut_comp[138] = 8'h7D; lut_comp[139] = 8'h7E;
        lut_comp[140] = 8'h7F; lut_comp[141] = 8'h80; lut_comp[142] = 8'h81; lut_comp[143] = 8'h81;
        lut_comp[144] = 8'h82; lut_comp[145] = 8'h83; lut_comp[146] = 8'h84; lut_comp[147] = 8'h85;
        lut_comp[148] = 8'h86; lut_comp[149] = 8'h86; lut_comp[150] = 8'h87; lut_comp[151] = 8'h88;
        lut_comp[152] = 8'h89; lut_comp[153] = 8'h8A; lut_comp[154] = 8'h8B; lut_comp[155] = 8'h8B;
        lut_comp[156] = 8'h8C; lut_comp[157] = 8'h8D; lut_comp[158] = 8'h8E; lut_comp[159] = 8'h8F;
        lut_comp[160] = 8'h90; lut_comp[161] = 8'h91; lut_comp[162] = 8'h91; lut_comp[163] = 8'h92;
        lut_comp[164] = 8'h93; lut_comp[165] = 8'h94; lut_comp[166] = 8'h95; lut_comp[167] = 8'h96;
        lut_comp[168] = 8'h97; lut_comp[169] = 8'h98; lut_comp[170] = 8'h98; lut_comp[171] = 8'h99;
        lut_comp[172] = 8'h9A; lut_comp[173] = 8'h9B; lut_comp[174] = 8'h9C; lut_comp[175] = 8'h9D;
        lut_comp[176] = 8'h9E; lut_comp[177] = 8'h9F; lut_comp[178] = 8'hA0; lut_comp[179] = 8'hA0;
        lut_comp[180] = 8'hA1; lut_comp[181] = 8'hA2; lut_comp[182] = 8'hA3; lut_comp[183] = 8'hA4;
        lut_comp[184] = 8'hA5; lut_comp[185] = 8'hA6; lut_comp[186] = 8'hA7; lut_comp[187] = 8'hA8;
        lut_comp[188] = 8'hA8; lut_comp[189] = 8'hA9; lut_comp[190] = 8'hAA; lut_comp[191] = 8'hAB;
        lut_comp[192] = 8'hAC; lut_comp[193] = 8'hAD; lut_comp[194] = 8'hAE; lut_comp[195] = 8'hAF;
        lut_comp[196] = 8'hB0; lut_comp[197] = 8'hB1; lut_comp[198] = 8'hB2; lut_comp[199] = 8'hB3;
        lut_comp[200] = 8'hB3; lut_comp[201] = 8'hB4; lut_comp[202] = 8'hB5; lut_comp[203] = 8'hB6;
        lut_comp[204] = 8'hB7; lut_comp[205] = 8'hB8; lut_comp[206] = 8'hB9; lut_comp[207] = 8'hBA;
        lut_comp[208] = 8'hBB; lut_comp[209] = 8'hBC; lut_comp[210] = 8'hBD; lut_comp[211] = 8'hBE;
        lut_comp[212] = 8'hBF; lut_comp[213] = 8'hC0; lut_comp[214] = 8'hC1; lut_comp[215] = 8'hC1;
        lut_comp[216] = 8'hC2; lut_comp[217] = 8'hC3; lut_comp[218] = 8'hC4; lut_comp[219] = 8'hC5;
        lut_comp[220] = 8'hC6; lut_comp[221] = 8'hC7; lut_comp[222] = 8'hC8; lut_comp[223] = 8'hC9;
        lut_comp[224] = 8'hCA; lut_comp[225] = 8'hCB; lut_comp[226] = 8'hCC; lut_comp[227] = 8'hCD;
        lut_comp[228] = 8'hCE; lut_comp[229] = 8'hCF; lut_comp[230] = 8'hD0; lut_comp[231] = 8'hD1;
        lut_comp[232] = 8'hD2; lut_comp[233] = 8'hD3; lut_comp[234] = 8'hD4; lut_comp[235] = 8'hD5;
        lut_comp[236] = 8'hD5; lut_comp[237] = 8'hD6; lut_comp[238] = 8'hD7; lut_comp[239] = 8'hD8;
        lut_comp[240] = 8'hD9; lut_comp[241] = 8'hDA; lut_comp[242] = 8'hDB; lut_comp[243] = 8'hDC;
        lut_comp[244] = 8'hDD; lut_comp[245] = 8'hDE; lut_comp[246] = 8'hDF; lut_comp[247] = 8'hE0;
        lut_comp[248] = 8'hE1; lut_comp[249] = 8'hE2; lut_comp[250] = 8'hE3; lut_comp[251] = 8'hE4;
        lut_comp[252] = 8'hE5; lut_comp[253] = 8'hE6; lut_comp[254] = 8'hE7; lut_comp[255] = 8'hE8;
    end

    //-------------------------------------------------------------------------
    // 4. 逐像素混合（三通道同一强度，保色调）
    //-------------------------------------------------------------------------
    // d = 目标曲线值 - 原值（有符号 9bit，范围 -255..+255）
    wire signed [8:0] d_r = s_pos ? ($signed({1'b0, lut_lift[r8]}) - $signed({1'b0, r8}))
                                  : ($signed({1'b0, lut_comp[r8]}) - $signed({1'b0, r8}));
    wire signed [8:0] d_g = s_pos ? ($signed({1'b0, lut_lift[g8]}) - $signed({1'b0, g8}))
                                  : ($signed({1'b0, lut_comp[g8]}) - $signed({1'b0, g8}));
    wire signed [8:0] d_b = s_pos ? ($signed({1'b0, lut_lift[b8]}) - $signed({1'b0, b8}))
                                  : ($signed({1'b0, lut_comp[b8]}) - $signed({1'b0, b8}));

    wire signed [17:0] p_r = d_r * $signed({1'b0, s_mag});
    wire signed [17:0] p_g = d_g * $signed({1'b0, s_mag});
    wire signed [17:0] p_b = d_b * $signed({1'b0, s_mag});

    // ch + (d*|s| >> 8)，范围 -255..+509，用 11bit 有符号承接后再限幅
    wire signed [10:0] u_r = $signed({2'b00, r8}) + (p_r >>> 8);
    wire signed [10:0] u_g = $signed({2'b00, g8}) + (p_g >>> 8);
    wire signed [10:0] u_b = $signed({2'b00, b8}) + (p_b >>> 8);

    wire [7:0] r_o = (u_r < 0) ? 8'd0 : (u_r > 11'sd255) ? 8'd255 : u_r[7:0];
    wire [7:0] g_o = (u_g < 0) ? 8'd0 : (u_g > 11'sd255) ? 8'd255 : u_g[7:0];
    wire [7:0] b_o = (u_b < 0) ? 8'd0 : (u_b > 11'sd255) ? 8'd255 : u_b[7:0];

    // 回 RGB565：取高 5/6/5 位。注意 r8={r5,r5[4:2]}，r8[7:3] 恰好还原 r5，
    // 因此 s=0 时像素与输入逐位相同（不是"接近"，是完全相同）。
    wire [15:0] mapped = {r_o[7:3], g_o[7:2], b_o[7:3]};

    //-------------------------------------------------------------------------
    // 5. 输出寄存：延迟恒为 1 拍；enable=0 时严格直通
    //-------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_out <= 1'b0;
            pixel_out <= {PIX_WIDTH{1'b0}};
        end else begin
            valid_out <= valid_in;
            if (!enable)
                pixel_out <= pixel_in;               // 基础任务 / 高阶任务一：原样直通
            else
                pixel_out <= mapped[PIX_WIDTH-1:0];  // 高阶任务三：自适应色调映射
        end
    end

    //-------------------------------------------------------------------------
    // 6. 统计累加（纯顺序逻辑，不影响上面的组合通道）
    //-------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum_y   <= 32'd0;
            pix_cnt <= 24'd0;
            brt_cnt <= 24'd0;
        end else if (frame_rise || !enable) begin
            // 帧头清零；关闭时也保持清零，避免重新进入时用到陈旧统计
            sum_y   <= 32'd0;
            pix_cnt <= 24'd0;
            brt_cnt <= 24'd0;
        end else if (stat_en) begin
            sum_y   <= sum_y + {24'd0, y8};
            pix_cnt <= pix_cnt + 24'd1;
            if (brt_now)
                brt_cnt <= brt_cnt + 24'd1;
        end
    end

endmodule
