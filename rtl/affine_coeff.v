`timescale 1ns / 1ps
//=============================================================================
// affine_coeff.v -- 有效角度 -> 仿射系数（每帧计算一次）
//
//   输入为「已减零偏」的有效角度，16 位符号+幅值，单位 0.01 度。
//   输出 2x3 仿射系数（Q16.16），供 affine_map 使用：
//       u' = (h11*u + h12*v + h13) >> 16
//       v' = (h21*u + h22*v + h23) >> 16
//
//   映射关系（已修正）：
//       yaw   -> 水平位移 tx
//       pitch -> 垂直位移 ty
//       roll  -> 绕画面中心旋转
//
//   容量约束：行缓存只能提供「已写入、且未被下一帧预取覆盖」的源行。
//   读流领先显示 64 行，客户端可用窗口是 [y-63, y+63]，但每帧最后 64 行
//   会被下一帧的预取写入覆盖旧帧 640~703 行，形成一段"洞"；只要竖直偏移
//   |ty| + 旋转行数 ≤ 40 行，既碰不到 y+63 的未写边界，也进不了那个洞。
//   绕画面中心旋转时，源行偏移量为 640·sin|θ| + 360·(1-cosθ)，这部分
//   行数必须先扣掉，剩下的余量才留给 pitch 的垂直平移。因此 ty 的限幅
//   是「每帧动态计算」的，而不是固定常数：角度越大，可用的平移余量越小。
//
//   旋转上限取 2.0 度（此时旋转占用约 22.9 行），窗口预算 40 行里剩下的
//   余量留给 pitch。超出预算的位移直接截断（宁可少补偿，也不让画面
//   踩进缓存窗口外而出现拉伸/错位条带）。
//   水平平移（yaw）不占用行缓存，可放宽到 ±600 像素。
//
//   enable=0 时输出单位矩阵，基础任务画面完全不变。
//=============================================================================
module affine_coeff #(
    parameter [14:0] ROT_LIMIT   = 15'd500,         // 旋转限幅 5.0 度，留出缓存余量
    // 增益：1 度 = 100 个角度单位，所以 GAIN_Q16 = 每度像素数 × 65536 / 100
    //   3277  = 5.0  px/度（原值，偏保守）
    //   6554  = 10.0 px/度
    //   12596 = 19.2 px/度（= f·tan1°，fx≈1101，物理上"补满"）
    //   19661 = 30.0 px/度（演示用，过补偿约 56%）
    // 约束：水平只受 TX_LIMIT(±600 px) 限制，可以放手加大；
    //       竖直受行缓存 40 行预算限制，每度像素数越大，pitch 可补的角度越小
    //       （约 40/G 度），所以 Y 轴不要一步提到 19。
    // 必须声明为 signed：否则 tx_raw = yaw_s * GAIN 会退化成无符号乘法，负角度方向算错。
    parameter signed [31:0] GAIN_X_Q16 = 32'sd29491,  // yaw  : 45.0 像素/度（演示用）
    parameter signed [31:0] GAIN_Y_Q16 = 32'sd6554,   // pitch: 10.0 像素/度
    // 竖直窗口预算 40 像素（详见文件头）。超过 46 行就会踩进帧尾被预取
    // 覆盖的"洞"，因此这里取 40 留出余量。
    parameter signed [31:0] WIN_MARGIN_Q16 = 32'sd2621440, // 40 像素窗口预算
    parameter signed [31:0] TX_LIMIT_Q16 = 32'sd39321600,  // ±600 像素
    // 方向约定（2026-11-03 上板最终确认，三轴都按「屏幕内容移动方向」定义）：
    //   yaw   正 -> 画面右移   => tx = -yaw  * GAIN，X_INV = 1
    //   pitch 正 -> 画面下移   => ty = -pitch* GAIN，Y_INV = 1
    //   roll  正 -> 画面逆时针 => sin 不取反，      R_INV = 0
    parameter        X_INV = 1'b1,
    parameter        Y_INV = 1'b1,
    parameter        R_INV = 1'b0
)(
    input  wire                clk,
    input  wire                rst_n,
    input  wire                frame_start,
    input  wire                enable,
    input  wire [15:0]         yaw,
    input  wire [15:0]         pitch,
    input  wire [15:0]         roll,
    output reg  signed [31:0]  h11,
    output reg  signed [31:0]  h21,
    output reg  signed [31:0]  h12,
    output reg  signed [31:0]  h22,
    output reg  signed [31:0]  h13,
    output reg  signed [31:0]  h23,
    output reg  signed [31:0]  yaw_tx_q16,
    output reg  signed [31:0]  pitch_ty_q16,
    output reg  signed [31:0]  roll_c_q16,
    // sin 拆成「幅值 + 符号」两个输出。幅值恒为正（0..5718），综合时对乘法器
    // 操作数做任何位宽截断/扩展都不会改变数值；若直接把负的 Q16 sin 送进乘法器，
    // 窄位宽有符号操作数会被零扩展（-0.1° 的 -114 变成 2^N-114），整帧坐标被
    // 推出画面：roll<0 时全黑或周期性花屏，roll>0 正常。
    output reg  [15:0]         roll_s_mag,
    output reg                 roll_s_neg
);

    localparam signed [31:0] ONE_Q16 = 32'sd65536;
    localparam signed [31:0] CX_Q16  = 32'sd41943040;  // 640 * 65536
    localparam signed [31:0] CY_Q16  = 32'sd23592960;  // 360 * 65536
    localparam signed [31:0] CX_INT  = 32'sd640;
    localparam signed [31:0] CY_INT  = 32'sd360;

    // ---------------- 输入分解 ----------------
    wire        neg_r  = roll[15];
    wire [14:0] mag_r0 = roll[14:0];
    wire [14:0] mag_r  = (mag_r0 > ROT_LIMIT) ? ROT_LIMIT : mag_r0;

    wire signed [15:0] yaw_s = yaw[15]   ? -$signed({1'b0, yaw[14:0]})   : $signed({1'b0, yaw[14:0]});
    wire signed [15:0] pit_s = pitch[15] ? -$signed({1'b0, pitch[14:0]}) : $signed({1'b0, pitch[14:0]});

    // ---------------- sin / cos ----------------
    wire [31:0] sin_mag = ({17'd0, mag_r} * 32'd11712) >> 10;
    wire signed [31:0] s_q16 = neg_r ? -$signed(sin_mag) : $signed(sin_mag);
    wire [31:0] cos_corr = (sin_mag * sin_mag) >> 17;
    wire signed [31:0] c_q16 = ONE_Q16 - $signed(cos_corr);

    // ---------------- 平移量 ----------------
    // 采样偏移 = 增益 * 角度，方向可用 X_INV / Y_INV 反相
    wire signed [31:0] tx_raw = yaw_s * GAIN_X_Q16;
    wire signed [31:0] ty_raw = pit_s * GAIN_Y_Q16;
    wire signed [31:0] tx_sgn = X_INV ? -tx_raw : tx_raw;
    wire signed [31:0] ty_sgn = Y_INV ? -ty_raw : ty_raw;

    // ---------------- 垂直平移的动态限幅 ----------------
    // 旋转本身要消耗 640·|sinθ| + 360·(1-cosθ) 行源数据（绕画面中心旋转时，
    // 左右两端比中心多取这么多行），先把它从窗口预算里扣掉，剩下的才是
    // pitch 平移能用的余量。这样无论 roll 还是 pitch 单独变化都不会越过
    // 行缓存窗口，避免出现 miss 兜底像素。
    wire signed [31:0] s_abs_q16    = s_q16[31] ? -s_q16 : s_q16;
    wire signed [31:0] rot_rows_q16 = (s_abs_q16 * 32'sd640) +
                                      ((ONE_Q16 - c_q16) * 32'sd360);
    wire signed [31:0] ty_lim_q16   = WIN_MARGIN_Q16 - rot_rows_q16;
    wire signed [31:0] ty_lim_pos   = (ty_lim_q16 > 0) ? ty_lim_q16 : 32'sd0;

    wire signed [31:0] tx_q16 = (tx_sgn >  TX_LIMIT_Q16) ?  TX_LIMIT_Q16 :
                                (tx_sgn < -TX_LIMIT_Q16) ? -TX_LIMIT_Q16 : tx_sgn;

    wire signed [31:0] ty_q16 = (ty_sgn >  ty_lim_pos) ?  ty_lim_pos :
                                (ty_sgn < -ty_lim_pos) ? -ty_lim_pos : ty_sgn;

    // frame_start 是单拍脉冲。改为上升沿检测，确保每帧只锁存一次系数：
    // 组合逻辑 c_q16/s_q16/tx_q16/ty_q16 由 yaw/pitch/roll 直接算出，而
    // angle_zero_cal 在 frame_start 同一拍才更新 cal_*。若在 frame_start
    // 高电平期间持续电平采样，系数会在 cal_* 更新前后被重载两次——前半用
    // 旧系数、后半用新系数，同一帧内映射突变导致整屏黑。先打 1 拍再取
    // 上升沿，保证锁存到的是「cal_* 更新后」的新角度对应的系数。
    reg frame_start_d;
    wire frame_start_rise;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) frame_start_d <= 1'b0;
        else        frame_start_d <= frame_start;
    end
    assign frame_start_rise = frame_start && !frame_start_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            h11 <= ONE_Q16; h12 <= 32'sd0; h13 <= 32'sd0;
            h21 <= 32'sd0;  h22 <= ONE_Q16; h23 <= 32'sd0;
            yaw_tx_q16 <= 32'sd0;
            pitch_ty_q16 <= 32'sd0;
            roll_c_q16 <= ONE_Q16;
            roll_s_mag <= 16'd0;
            roll_s_neg <= 1'b0;
        end else if (frame_start_rise) begin
            if (enable) begin
                // Keep the affine stage as an identity pipeline.  All pose
                // offsets and the inverse roll are applied after distortion
                // in the top level, so yaw/pitch/roll cannot distort one
                // another and the rotation center stays fixed.
                h11 <= ONE_Q16; h12 <= 32'sd0; h13 <= 32'sd0;
                h21 <= 32'sd0;  h22 <= ONE_Q16; h23 <= 32'sd0;
                yaw_tx_q16 <= tx_q16;
                pitch_ty_q16 <= ty_q16;
                roll_c_q16 <= c_q16;
                // 正 roll → 画面逆时针（2026-11-03 上板实测方向）。
                roll_s_mag <= sin_mag[15:0];
                roll_s_neg <= neg_r ^ R_INV;
            end else begin
                h11 <= ONE_Q16; h12 <= 32'sd0; h13 <= 32'sd0;
                h21 <= 32'sd0;  h22 <= ONE_Q16; h23 <= 32'sd0;
                yaw_tx_q16 <= 32'sd0;
                pitch_ty_q16 <= 32'sd0;
                roll_c_q16 <= ONE_Q16;
                roll_s_mag <= 16'd0;
                roll_s_neg <= 1'b0;
            end
        end
    end

endmodule
