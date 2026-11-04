`timescale 1ns / 1ps
//=============================================================================
// distortion_param_ctrl.v -- 畸变参数在线动态控制（高阶任务支撑）
//
//   输出 k1/k2/p1/p2 的运行时值，供 distortion_map 使用。
//   参数只在帧起始变化，保证同一帧内坐标映射一致，不会出现画面撕裂。
//   进入高阶任务(enable=1)后，系数按三角波缓慢缩放，画面呈桶形/枕形往复变化。
//   未进入高阶任务(enable=0)时，输出与标定常量完全一致，基础任务不受影响。
//
//   后续若接上位机或 IMU：把 scale 的计算换成外部输入即可，本模块接口不变。
//=============================================================================
module distortion_param_ctrl #(
    parameter signed [31:0] K1_DEF = 32'sd5764105,
    parameter signed [31:0] K2_DEF = -32'sd147684499,
    parameter signed [31:0] P1_DEF = -32'sd3025473,
    parameter signed [31:0] P2_DEF = -32'sd329861
)(
    input  wire                clk,
    input  wire                rst_n,
    input  wire                frame_start,   // 帧起始脉冲（pix_clk 域）
    input  wire                enable,        // 1 = 高阶任务，参数动态
    output reg  signed [31:0]  k1_dyn,
    output reg  signed [31:0]  k2_dyn,
    output reg  signed [31:0]  p1_dyn,
    output reg  signed [31:0]  p2_dyn
);

    // 三角波相位：每帧 +2，256 帧一个完整周期（720P@30 约 4.3 秒）
    reg  [7:0]  phase;
    wire [7:0]  tri_val = (phase < 8'd128) ? phase : (8'd255 - phase); // 0..127

    // Q16.16 缩放系数：0.94 (61604) .. 1.06 (69478)，小幅呼吸，避免几何大幅跳动
    wire [31:0] scale_u = 32'd61604 + {24'd0, tri_val} * 32'd62;
    wire signed [31:0] scale_s = $signed({1'b0, scale_u});

    wire signed [63:0] k1_prod = K1_DEF * scale_s;
    wire signed [63:0] k2_prod = K2_DEF * scale_s;
    wire signed [63:0] p1_prod = P1_DEF * scale_s;
    wire signed [63:0] p2_prod = P2_DEF * scale_s;

    // 与 affine_coeff 同理：frame_start 高电平期间若 enable 或组合逻辑
    // 有毛刺，电平触发会让参数被重载多次；改为上升沿检测，每帧只锁存一次。
    reg frame_start_d;
    wire frame_start_rise;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) frame_start_d <= 1'b0;
        else        frame_start_d <= frame_start;
    end
    assign frame_start_rise = frame_start && !frame_start_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase   <= 8'd0;
            k1_dyn  <= K1_DEF;
            k2_dyn  <= K2_DEF;
            p1_dyn  <= P1_DEF;
            p2_dyn  <= P2_DEF;
        end else if (frame_start_rise) begin
            if (enable) begin
                phase  <= phase + 8'd2;
                k1_dyn <= k1_prod >>> 16;
                k2_dyn <= k2_prod >>> 16;
                p1_dyn <= p1_prod >>> 16;
                p2_dyn <= p2_prod >>> 16;
            end else begin
                phase  <= 8'd0;
                k1_dyn <= K1_DEF;
                k2_dyn <= K2_DEF;
                p1_dyn <= P1_DEF;
                p2_dyn <= P2_DEF;
            end
        end
    end

endmodule
