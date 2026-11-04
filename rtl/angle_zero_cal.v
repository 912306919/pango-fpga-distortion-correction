`timescale 1ns / 1ps
//=============================================================================
// angle_zero_cal.v -- IMU 零偏自动标定（运动门限 + 野值剔除 + 有效性校验）
//
//   原实现只在上电后固定窗口内做简单平均。若标定期间转动板子、yaw 跨 ±180°
//   或读数异常，零点会被标到错误位置（实测约 -118°），affine_coeff 会把平移量
//   顶到 ±600 像素上限，源坐标被 clamp 成常数，画面出现整块纯色横条。
//
//   本版本增加四层保护：
//   1) 运动门限：相邻帧变化超过 MOTION_THRESH 的帧不计入平均，改用上一帧有效
//      值，保证每个标定窗口恰好 CAL_FRAMES 帧。
//   2) 野值剔除：窗口内极差超过 SPREAD_MAX，或被替换帧超过 MAX_OUT，判该窗口
//      不是静止标定，整窗丢弃并重试（最多 RETRY_MAX 次）。
//   3) 有效性校验：零点幅值超过 OFF_LIMIT(15°) 判为无效，此时不做任何补偿，
//      输出恒 0，避免错误零点把画面推到饱和。
//   4) yaw 最短弧：差分与回绕都走 ±180° 最短路径，跨零点不产生 360° 跳变。
//
//   输入/输出均为 0.01 度、符号+幅值格式（bit15=符号，bit14:0=幅值）。
//   输出 = 原始角度 - 零点（回绕到 [-180°,180°]）；
//   标定完成前以及零点无效时输出 0（不做补偿）。
//=============================================================================
module angle_zero_cal #(
    parameter [7:0]  SKIP_FRAMES   = 8'd30,    // 上电后跳过帧数（30 帧 ≈ 1 秒）
    parameter [7:0]  CAL_FRAMES    = 8'd128,   // 每个标定窗口帧数（2 的幂）
    parameter [2:0]  CAL_SHIFT     = 3'd7,     // log2(CAL_FRAMES)
    parameter [15:0] MOTION_THRESH = 16'd40,   // 帧间变化门限 0.40°（≈12°/s）
    parameter [15:0] SPREAD_MAX    = 16'd300,  // 窗口内极差上限 3.00°（pitch/roll）
    parameter [15:0] SPREAD_MAX_YAW = 16'd1000,// yaw 极差上限 10.00°（6 轴 yaw 有缓慢漂移）
    parameter [7:0]  MAX_OUT       = 8'd64,    // 窗口内可替换帧数上限
    parameter [1:0]  RETRY_MAX     = 2'd2,     // 丢弃窗口后的重试次数
    parameter [15:0] OFF_LIMIT     = 16'd1500  // 零点幅值上限 15.00°
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        frame_start,
    input  wire [15:0] yaw_raw,
    input  wire [15:0] pitch_raw,
    input  wire [15:0] roll_raw,
    output reg  [15:0] yaw_cal,
    output reg  [15:0] pitch_cal,
    output reg  [15:0] roll_cal,
    output reg         cal_done
);

    // 符号+幅值 -> 有符号补码
    function signed [15:0] sm2s;
        input [15:0] v;
        begin
            sm2s = v[15] ? -$signed({1'b0, v[14:0]}) : $signed({1'b0, v[14:0]});
        end
    endfunction

    // 补码 -> 符号+幅值
    function [15:0] s2sm;
        input signed [15:0] v;
        begin
            s2sm = v[15] ? {1'b1, (~v[14:0] + 15'd1)} : {1'b0, v[14:0]};
        end
    endfunction

    // 最短弧差分 a-b，结果落在 [-18000, 18000]（单位 0.01°）
    function signed [16:0] ang_diff;
        input signed [16:0] a;
        input signed [16:0] b;
        reg signed [16:0] d;
        begin
            d = a - b;
            if (d >  17'sd18000) d = d - 17'sd36000;
            if (d < -17'sd18000) d = d + 17'sd36000;
            ang_diff = d;
        end
    endfunction

    // 回绕到 [-18000, 18000]（输入为残差，范围有限，一次校正足够）
    function signed [15:0] ang_wrap;
        input signed [16:0] a;
        reg signed [16:0] d;
        begin
            d = a;
            if (d >  17'sd18000) d = d - 17'sd36000;
            if (d < -17'sd18000) d = d + 17'sd36000;
            ang_wrap = d[15:0];
        end
    endfunction

    reg [7:0]  skip_cnt;
    reg [7:0]  win_cnt;
    reg [7:0]  out_cnt;     // 窗口内被「替换」的帧数
    reg [1:0]  retry_cnt;
    reg signed [31:0] acc_y, acc_p, acc_r;
    reg signed [15:0] hold_y, hold_p, hold_r;   // 最近一个有效（静止）采样
    reg signed [15:0] prev_y, prev_p, prev_r;   // 上一帧采样，用于运动判据
    reg signed [16:0] min_y, min_p, min_r;
    reg signed [16:0] max_y, max_p, max_r;
    reg signed [15:0] off_y, off_p, off_r;
    reg        off_valid;


    wire signed [15:0] y_s = sm2s(yaw_raw);
    wire signed [15:0] p_s = sm2s(pitch_raw);
    wire signed [15:0] r_s = sm2s(roll_raw);

    // ---------------- 运动判据 ----------------
    wire signed [16:0] d_yaw = ang_diff(y_s, prev_y);
    wire signed [16:0] d_pit = $signed(p_s) - $signed(prev_p);
    wire signed [16:0] d_rol = $signed(r_s) - $signed(prev_r);
    wire signed [16:0] mt    = $signed({1'b0, MOTION_THRESH});

    wire still = (d_yaw <= mt) && (d_yaw >= -mt) &&
                 (d_pit <= mt) && (d_pit >= -mt) &&
                 (d_rol <= mt) && (d_rol >= -mt);

    // yaw 以 hold 为参考解开 ±180° 回绕，得到连续角度
    wire signed [16:0] y_cont = $signed(hold_y) + ang_diff(y_s, hold_y);
    wire signed [16:0] p_cont = $signed(p_s);
    wire signed [16:0] r_cont = $signed(r_s);

    // 静止帧用当前采样，运动帧用上一有效值替换（保证窗口长度固定）
    wire signed [16:0] val_y = still ? y_cont : $signed(hold_y);
    wire signed [16:0] val_p = still ? p_cont : $signed(hold_p);
    wire signed [16:0] val_r = still ? r_cont : $signed(hold_r);

    wire signed [31:0] acc_n_y = acc_y + val_y;
    wire signed [31:0] acc_n_p = acc_p + val_p;
    wire signed [31:0] acc_n_r = acc_r + val_r;

    wire signed [16:0] spread_y = max_y - min_y;
    wire signed [16:0] spread_p = max_p - min_p;
    wire signed [16:0] spread_r = max_r - min_r;
    wire signed [16:0] spread_lim     = $signed({1'b0, SPREAD_MAX});
    wire signed [16:0] spread_lim_yaw = $signed({1'b0, SPREAD_MAX_YAW});

    // 本窗口是否判定为「非静止」→ 丢弃重试
    // yaw 用较宽的极差门限：6 轴模式下 yaw 由陀螺积分得到，本身会缓慢漂移，
    // 用 3° 会把正常静止窗口误判成运动；pitch/roll 由重力参考，保持 3°。
    wire win_bad = (out_cnt >= MAX_OUT) ||
                   (spread_y > spread_lim_yaw) ||
                   (spread_p > spread_lim) ||
                   (spread_r > spread_lim);

    wire signed [31:0] mean_y = acc_n_y >>> CAL_SHIFT;
    wire signed [31:0] mean_p = acc_n_p >>> CAL_SHIFT;
    wire signed [31:0] mean_r = acc_n_r >>> CAL_SHIFT;

    wire signed [31:0] off_lim = $signed({16'd0, OFF_LIMIT});
    wire mean_bad = (mean_y > off_lim) || (mean_y < -off_lim) ||
                    (mean_p > off_lim) || (mean_p < -off_lim) ||
                    (mean_r > off_lim) || (mean_r < -off_lim);

    // 与 affine_coeff / distortion_param_ctrl 同理：frame_start 是单拍脉冲，
    // 电平触发会让 warmup 状态机在 1650 拍内被反复复位（skip_cnt/win_cnt/acc_*
    // 在脉冲期间一直清零），标定永远完不成；上升沿检测保证每帧只推进一步。
    reg frame_start_d;
    wire frame_start_rise;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) frame_start_d <= 1'b0;
        else        frame_start_d <= frame_start;
    end
    assign frame_start_rise = frame_start && !frame_start_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            skip_cnt  <= 8'd0;
            win_cnt   <= 8'd0;
            out_cnt   <= 8'd0;
            retry_cnt <= 2'd0;
            acc_y     <= 32'sd0;
            acc_p     <= 32'sd0;
            acc_r     <= 32'sd0;
            hold_y    <= 16'sd0;
            hold_p    <= 16'sd0;
            hold_r    <= 16'sd0;
            prev_y    <= 16'sd0;
            prev_p    <= 16'sd0;
            prev_r    <= 16'sd0;
            min_y     <= 17'sd0; max_y <= 17'sd0;
            min_p     <= 17'sd0; max_p <= 17'sd0;
            min_r     <= 17'sd0; max_r <= 17'sd0;
            off_y     <= 16'sd0;
            off_p     <= 16'sd0;
            off_r     <= 16'sd0;
            off_valid <= 1'b0;
            cal_done <= 1'b0;
            yaw_cal   <= 16'd0;
            pitch_cal <= 16'd0;
            roll_cal  <= 16'd0;
        end else if (frame_start_rise) begin
            prev_y <= y_s;
            prev_p <= p_s;
            prev_r <= r_s;

            if (!cal_done) begin
                // 标定进行中：不做补偿
                yaw_cal   <= 16'd0;
                pitch_cal <= 16'd0;
                roll_cal  <= 16'd0;

                if (skip_cnt != SKIP_FRAMES) begin
                    // 上电等待阶段，同时用当前值初始化 hold
                    skip_cnt <= skip_cnt + 8'd1;
                    hold_y   <= y_s;
                    hold_p   <= p_s;
                    hold_r   <= r_s;
                    win_cnt  <= 8'd0;
                    out_cnt  <= 8'd0;
                end else if (win_cnt == 8'd0) begin
                    // 窗口首帧：无条件计入，建立 hold / 极值基准
                    win_cnt <= 8'd1;
                    out_cnt <= 8'd0;
                    acc_y   <= y_cont;
                    acc_p   <= p_cont;
                    acc_r   <= r_cont;
                    hold_y  <= y_s;
                    hold_p  <= p_s;
                    hold_r  <= r_s;
                    min_y   <= y_cont; max_y <= y_cont;
                    min_p   <= p_cont; max_p <= p_cont;
                    min_r   <= r_cont; max_r <= r_cont;
                end else begin
                    // 窗口常规帧：静止才更新 hold，否则用旧值替换
                    if (still) begin
                        hold_y <= y_s;
                        hold_p <= p_s;
                        hold_r <= r_s;
                    end
                    acc_y <= acc_n_y;
                    acc_p <= acc_n_p;
                    acc_r <= acc_n_r;
                    if (!still)
                        out_cnt <= out_cnt + 8'd1;

                    if (val_y < min_y) min_y <= val_y;
                    if (val_y > max_y) max_y <= val_y;
                    if (val_p < min_p) min_p <= val_p;
                    if (val_p > max_p) max_p <= val_p;
                    if (val_r < min_r) min_r <= val_r;
                    if (val_r > max_r) max_r <= val_r;

                    if (win_cnt == CAL_FRAMES - 8'd1) begin
                        // 窗口结束：先做质量判定，再取平均作为零点
                        if (win_bad) begin
                            // 非静止窗口：丢弃并重试
                            win_cnt <= 8'd0;
                            out_cnt <= 8'd0;
                            acc_y   <= 32'sd0;
                            acc_p   <= 32'sd0;
                            acc_r   <= 32'sd0;
                            if (retry_cnt == RETRY_MAX) begin
                                off_y      <= 16'sd0;
                                off_p      <= 16'sd0;
                                off_r      <= 16'sd0;
                                off_valid  <= 1'b0;
                                cal_done <= 1'b1;   // 放弃补偿，保持原图
                            end else begin
                                retry_cnt <= retry_cnt + 2'd1;
                            end
                        end else if (mean_bad) begin
                            // 零点超出 15°，说明标定条件不成立，判无效
                            off_y      <= 16'sd0;
                            off_p      <= 16'sd0;
                            off_r      <= 16'sd0;
                            off_valid  <= 1'b0;
                            cal_done <= 1'b1;
                        end else begin
                            off_y      <= mean_y[15:0];
                            off_p      <= mean_p[15:0];
                            off_r      <= mean_r[15:0];
                            off_valid  <= 1'b1;
                            cal_done <= 1'b1;
                        end
                    end else begin
                        win_cnt <= win_cnt + 8'd1;
                    end
                end
            end else begin
                // 标定完成：输出残差（yaw 走 ±180° 最短弧回绕）
                if (off_valid) begin
                    yaw_cal   <= s2sm(ang_wrap($signed(y_s) - $signed(off_y)));
                    pitch_cal <= s2sm(ang_wrap($signed(p_s) - $signed(off_p)));
                    roll_cal  <= s2sm(ang_wrap($signed(r_s) - $signed(off_r)));
                end else begin
                    yaw_cal   <= 16'd0;
                    pitch_cal <= 16'd0;
                    roll_cal  <= 16'd0;
                end
            end
        end
    end

endmodule
