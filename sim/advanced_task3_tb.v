`timescale 1ns / 1ps
//=============================================================================
// advanced_task3_tb.v -- 高阶任务三（帧级自适应 Gamma）单元测试
//
// 用缩小帧尺寸（8x8 = 64 像素/帧）驱动。判定阈值全部由 IMG_W*IMG_H 参数化，
// 因此本 tb 验证的定档逻辑与 1280x720 实机完全相同，只是帧短、仿真快。
//
// 时序约定：所有激励在 negedge 驱动、在 posedge 之后 #1 采样，
//           避免与 DUT 的 always @(posedge clk) 产生竞争。
//
// 覆盖项：
//   1. 穷举一致性：把 s 强制成 0 / +170 / ±255，遍历全部 65536 个 RGB565
//      码字，与 tb 内参考模型逐位比对。其中 s=0 必须与输入逐位相同 ——
//      这是「不影响基础任务/高阶任务一」最直接的证据。
//   2. 直通隔离：enable=0 时 300 个随机像素原样输出，valid 延迟 1 拍。
//   3. 正常光照：s 收敛到 0，输出位精确恒等。
//   4. 夜间：s 收敛到 +255，暗部提亮、高光不动；亮度读数≈34、高光占比 0。
//   5. 强光：s 收敛到 -255，高光压缩、暗部不动；亮度读数≈236、占比 100%。
//   6. 高光占比判据：平均亮度落在正常区间但 31% 像素过曝时仍判强光。
//   6b. 小面积强光：3.1% 过曝不判、6.25% 过曝判强光（5% 阈值两侧），
//       对应现场「手电照镜头却一直显示 NORMAL」的回归用例。
//   7. 关闭即直通：从夜间(enable=1)切到 enable=0，输出立刻恢复原样。
//
// 运行：
//   iverilog -g2012 -o sim_adv3.vvp source/rtl/advanced_task3.v \
//            source/sim/advanced_task3_tb.v
//   vvp sim_adv3.vvp
//=============================================================================
module advanced_task3_tb;

    localparam IMG_W = 8;
    localparam IMG_H = 8;
    localparam NPIX  = IMG_W * IMG_H;
    localparam DEBUG = 0;

    // 精确命中目标亮度的 RGB565 码字（脚本穷举得到）
    // 括号里的 y8 是按本 tb 的 luma8() 复算出来的实际值
    localparam [15:0] CODE_NIGHT  = 16'h20E4;  // y8 =  34
    localparam [15:0] CODE_DARK   = 16'h2945;  // y8 =  43
    localparam [15:0] CODE_MID120 = 16'h7BCE;  // y8 = 122
    localparam [15:0] CODE_NORMAL = 16'h7C10;  // y8 = 114
    localparam [15:0] CODE_BRT    = 16'hD6BB;  // y8 = 213
    localparam [15:0] CODE_GLARE  = 16'hDF5C;  // y8 = 236

    reg clk = 0;
    reg rst_n = 0;
    reg frame_start = 0;
    reg enable = 0;
    reg valid_in = 0;
    reg [10:0] x = 0;
    reg [9:0]  y = 0;
    reg [15:0] pixel_in = 0;

    wire        valid_out;
    wire [15:0] pixel_out;
    wire [2:0]  tier_out;
    wire [7:0]  mean_out;
    wire [7:0]  glare_out;

    integer errors = 0;
    integer checks = 0;

    always #5 clk = ~clk;   // 100 MHz

    advanced_task3 #(
        .PIX_WIDTH (16),
        .IMG_W     (IMG_W),
        .IMG_H     (IMG_H)
    ) dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .frame_start (frame_start),
        .enable      (enable),
        .valid_in    (valid_in),
        .x           (x),
        .y           (y),
        .pixel_in    (pixel_in),
        .valid_out   (valid_out),
        .pixel_out   (pixel_out),
        .tier        (tier_out),
        .mean_y      (mean_out),
        .glare_pct   (glare_out)
    );

    //-------------------------------------------------------------------------
    // 参考模型：与 RTL 相同的定点运算，直接复用 DUT 内部的查找表
    //-------------------------------------------------------------------------
    function [15:0] ref_map(input [15:0] c, input signed [9:0] s);
        reg [7:0] r8, g8, b8, ro, go, bo, mag;
        reg signed [8:0]  dr, dg, db;
        reg signed [17:0] pr, pg, pb;
        reg signed [10:0] ur, ug, ub;
        begin
            r8 = {c[15:11], c[15:13]};
            g8 = {c[10:5],  c[10:9]};
            b8 = {c[4:0],   c[4:2]};
            mag = s[9] ? ((~s[7:0]) + 8'd1) : s[7:0];
            if (s[9]) begin
                dr = $signed({1'b0, dut.lut_comp[r8]}) - $signed({1'b0, r8});
                dg = $signed({1'b0, dut.lut_comp[g8]}) - $signed({1'b0, g8});
                db = $signed({1'b0, dut.lut_comp[b8]}) - $signed({1'b0, b8});
            end else begin
                dr = $signed({1'b0, dut.lut_lift[r8]}) - $signed({1'b0, r8});
                dg = $signed({1'b0, dut.lut_lift[g8]}) - $signed({1'b0, g8});
                db = $signed({1'b0, dut.lut_lift[b8]}) - $signed({1'b0, b8});
            end
            pr = dr * $signed({1'b0, mag});
            pg = dg * $signed({1'b0, mag});
            pb = db * $signed({1'b0, mag});
            ur = $signed({2'b00, r8}) + (pr >>> 8);
            ug = $signed({2'b00, g8}) + (pg >>> 8);
            ub = $signed({2'b00, b8}) + (pb >>> 8);
            ro = (ur < 0) ? 8'd0 : (ur > 11'sd255) ? 8'd255 : ur[7:0];
            go = (ug < 0) ? 8'd0 : (ug > 11'sd255) ? 8'd255 : ug[7:0];
            bo = (ub < 0) ? 8'd0 : (ub > 11'sd255) ? 8'd255 : ub[7:0];
            ref_map = {ro[7:3], go[7:2], bo[7:3]};
        end
    endfunction

    function [7:0] luma8(input [15:0] c);
        reg [7:0] r8, g8, b8;
        begin
            r8 = {c[15:11], c[15:13]};
            g8 = {c[10:5],  c[10:9]};
            b8 = {c[4:0],   c[4:2]};
            luma8 = (77*r8 + 150*g8 + 29*b8) >> 8;
        end
    endfunction

    if (DEBUG) begin : g_dbg
        always @(posedge clk) begin
            if (dut.frame_rise)
                $display("    [dbg] rise pix=%0d sum=%0d brt=%0d ok=%0d s_new=%0d s_cur=%0d",
                         dut.pix_cnt, dut.sum_y, dut.brt_cnt, dut.frame_ok, dut.s_new, dut.s_cur);
        end
    end

    //-------------------------------------------------------------------------
    // 激励基础件（negedge 驱动）
    //-------------------------------------------------------------------------
    task do_reset;
        begin
            rst_n = 0; enable = 0; frame_start = 0; valid_in = 0; pixel_in = 0;
            repeat (5) @(posedge clk);
            #1 rst_n = 1;
        end
    endtask

    // 一帧：1 拍 frame_start + NPIX 个像素
    task send_frame(input [15:0] code);
        integer i;
        begin
            @(negedge clk); frame_start = 1;
            @(negedge clk); frame_start = 0;
            for (i = 0; i < NPIX; i = i + 1) begin
                valid_in = 1; pixel_in = code; x = i; y = 0;
                @(negedge clk);
            end
            valid_in = 0;
        end
    endtask

    // 通用「小面积强光」帧：前 nb 个像素过曝（y=236），其余中灰（y=122）。
    //   8x8 帧下 BR_GLARE = ceil(64*5%) = 4 个像素，
    //   所以 nb=4 -> 6.25% 判强光，nb=2 -> 3.1% 不判。
    task send_spot_frame(input integer nb);
        integer i;
        begin
            @(negedge clk); frame_start = 1;
            @(negedge clk); frame_start = 0;
            for (i = 0; i < NPIX; i = i + 1) begin
                valid_in = 1;
                pixel_in = (i < nb) ? CODE_GLARE : CODE_MID120;
                x = i; y = 0;
                @(negedge clk);
            end
            valid_in = 0;
        end
    endtask

    // 送一个像素并取回对应输出（1 拍延迟）
    task send_get(input [15:0] v, output [15:0] o);
        begin
            @(negedge clk); valid_in = 1; pixel_in = v; x = 0; y = 0;
            @(posedge clk); #1; o = pixel_out;
        end
    endtask

    //-------------------------------------------------------------------------
    // 1. 全码字穷举比对
    //-------------------------------------------------------------------------
    task sweep_codes(input signed [9:0] sval);
        integer c;
        integer local_err;
        begin
            local_err = 0;
            force dut.s_cur = sval;
            enable = 1;
            for (c = 0; c < 65536; c = c + 1) begin
                @(negedge clk); valid_in = 1; pixel_in = c[15:0];
                @(posedge clk); #1;
                if (pixel_out !== ref_map(c[15:0], sval)) begin
                    local_err = local_err + 1;
                    errors = errors + 1;
                    if (local_err <= 3)
                        $display("  FAIL: s=%0d code=%h out=%h exp=%h",
                                 sval, c[15:0], pixel_out, ref_map(c[15:0], sval));
                end
            end
            @(negedge clk); valid_in = 0;
            release dut.s_cur;
            checks = checks + 1;
            if (local_err == 0)
                $display("  OK: s=%4d 全 65536 码字与参考模型逐位一致", sval);
            else
                $display("  FAIL: s=%4d 有 %0d 个码字不一致", sval, local_err);
        end
    endtask

    //-------------------------------------------------------------------------
    // 2. enable=0 直通隔离
    //-------------------------------------------------------------------------
    task test_isolation;
        integer i;
        reg [15:0] v;
        integer local_err;
        begin
            local_err = 0;
            do_reset;
            enable = 0;
            for (i = 0; i < 300; i = i + 1) begin
                v = $random;
                @(negedge clk); valid_in = 1; pixel_in = v; x = i; y = 0;
                @(posedge clk); #1;
                if (pixel_out !== v) begin
                    local_err = local_err + 1;
                    errors = errors + 1;
                    if (local_err <= 3) $display("  FAIL: i=%0d in=%h out=%h", i, v, pixel_out);
                end
                if (valid_out !== 1'b1) begin
                    local_err = local_err + 1;
                    errors = errors + 1;
                    if (local_err <= 3) $display("  FAIL: valid 未随 valid_in 拉高 i=%0d", i);
                end
            end
            @(negedge clk); valid_in = 0;
            @(posedge clk); #1;
            if (valid_out !== 1'b0) begin
                errors = errors + 1;
                $display("  FAIL: valid_in 撤销后 valid_out 仍为 1");
            end
            checks = checks + 1;
            if (local_err == 0) $display("  OK: 300 个随机像素原样直通，valid 延迟 1 拍");
        end
    endtask

    //-------------------------------------------------------------------------
    // 3. 正常光照 -> s 收敛到 0
    //-------------------------------------------------------------------------
    task test_normal;
        integer f, i;
        integer local_err;
        begin
            local_err = 0;
            do_reset;
            enable = 1;
            for (f = 0; f < 20; f = f + 1) begin
                @(negedge clk); frame_start = 1;
                @(negedge clk); frame_start = 0;
                for (i = 0; i < NPIX; i = i + 1) begin
                    @(negedge clk); valid_in = 1; pixel_in = CODE_NORMAL; x = i; y = 0;
                    @(posedge clk); #1;
                    if (pixel_out !== CODE_NORMAL) begin
                        local_err = local_err + 1;
                        errors = errors + 1;
                        if (local_err <= 3)
                            $display("  FAIL: 正常光照非恒等 f=%0d i=%0d in=%h out=%h",
                                     f, i, CODE_NORMAL, pixel_out);
                    end
                end
                valid_in = 0;
            end
            checks = checks + 1;
            if (tier_out !== 3'd2) begin
                errors = errors + 1;
                $display("  FAIL: 正常光照档位=%0d，应为 2(NORMAL)", tier_out);
            end
            if (dut.s_cur !== 10'sd0) begin
                errors = errors + 1;
                $display("  FAIL: 正常光照 s_cur=%0d，应精确等于 0", dut.s_cur);
            end else if (local_err == 0) begin
                $display("  OK: 正常光照 s_cur=0，20 帧输出位精确恒等");
            end
        end
    endtask

    //-------------------------------------------------------------------------
    // 4. 夜间 -> s 收敛到 +255，暗部提亮、高光不动
    //-------------------------------------------------------------------------
    task test_night;
        integer f;
        integer local_err;
        reg [15:0] o_dark, o_high;
        begin
            local_err = 0;
            do_reset;
            enable = 1;
            for (f = 0; f < 60; f = f + 1) send_frame(CODE_NIGHT);
            // 高光像素：y8≈236，三通道都 >176，提亮曲线在拐点之上应为恒等
            send_get(CODE_GLARE, o_high);
            // 暗部像素：y8≈34，应被显著提亮
            send_get(CODE_NIGHT, o_dark);
            @(negedge clk); valid_in = 0;
            checks = checks + 1;

            if (dut.s_cur < 10'sd250) begin
                errors = errors + 1;
                $display("  FAIL: 夜间 s_cur=%0d，应收敛到 +255", dut.s_cur);
            end
            if (tier_out !== 3'd0) begin
                errors = errors + 1;
                $display("  FAIL: 夜间档位=%0d，应为 0(NIGHT)", tier_out);
            end
            // 屏幕读数：平均亮度应≈CODE_NIGHT 的 34，高光占比 0
            if (mean_out < 8'd30 || mean_out > 8'd38) begin
                errors = errors + 1;
                $display("  FAIL: 夜间平均亮度读数=%0d，应≈34", mean_out);
            end
            if (glare_out !== 8'd0) begin
                errors = errors + 1;
                $display("  FAIL: 夜间高光占比读数=%0d，应为 0", glare_out);
            end
            if (o_high !== CODE_GLARE) begin
                local_err = local_err + 1; errors = errors + 1;
                $display("  FAIL: 夜间误改高光 in=%h out=%h", CODE_GLARE, o_high);
            end
            if (luma8(o_dark) < luma8(CODE_NIGHT) + 20) begin
                local_err = local_err + 1; errors = errors + 1;
                $display("  FAIL: 夜间暗部未提亮 luma %0d -> %0d", luma8(CODE_NIGHT), luma8(o_dark));
            end
            if (local_err == 0)
                $display("  OK: s_cur=%0d，暗部 y %0d->%0d 提亮，高光保持 %h",
                         dut.s_cur, luma8(CODE_NIGHT), luma8(o_dark), o_high);
        end
    endtask

    //-------------------------------------------------------------------------
    // 5. 强光 -> s 收敛到 -255，高光压缩、暗部不动
    //-------------------------------------------------------------------------
    task test_glare;
        integer f;
        integer local_err;
        reg [15:0] o_high, o_mid, o_dark;
        begin
            local_err = 0;
            do_reset;
            enable = 1;
            for (f = 0; f < 60; f = f + 1) send_frame(CODE_GLARE);
            send_get(CODE_GLARE, o_high);
            // y8≈213 落在压缩最明显的区间（压缩曲线 96 以下恒等、255 压到 232）
            send_get(CODE_BRT, o_mid);
            // 暗部像素：三通道都 <=96，压缩曲线在拐点之下应为恒等
            send_get(CODE_DARK, o_dark);
            @(negedge clk); valid_in = 0;
            checks = checks + 1;

            if (dut.s_cur > -10'sd250) begin
                errors = errors + 1;
                $display("  FAIL: 强光 s_cur=%0d，应收敛到 -255", dut.s_cur);
            end
            if (tier_out !== 3'd4) begin
                errors = errors + 1;
                $display("  FAIL: 强光档位=%0d，应为 4(GLARE)", tier_out);
            end
            // 屏幕读数：全画面过曝 -> 平均亮度≈236、高光占比 100%
            if (mean_out < 8'd232 || mean_out > 8'd240) begin
                errors = errors + 1;
                $display("  FAIL: 强光平均亮度读数=%0d，应≈236", mean_out);
            end
            if (glare_out !== 8'd100) begin
                errors = errors + 1;
                $display("  FAIL: 强光高光占比读数=%0d，应为 100", glare_out);
            end
            if (o_dark !== CODE_DARK) begin
                local_err = local_err + 1; errors = errors + 1;
                $display("  FAIL: 强光误改暗部 in=%h out=%h", CODE_DARK, o_dark);
            end
            if (luma8(o_high) >= luma8(CODE_GLARE)) begin
                local_err = local_err + 1; errors = errors + 1;
                $display("  FAIL: 强光高光未下降 luma %0d -> %0d", luma8(CODE_GLARE), luma8(o_high));
            end
            if (luma8(o_mid) > luma8(CODE_BRT) - 15) begin
                local_err = local_err + 1; errors = errors + 1;
                $display("  FAIL: 强光高光压缩不足 luma %0d -> %0d", luma8(CODE_BRT), luma8(o_mid));
            end
            if (local_err == 0)
                $display("  OK: s_cur=%0d，高光 y %0d->%0d / %0d->%0d 压缩，暗部保持 %h",
                         dut.s_cur, luma8(CODE_GLARE), luma8(o_high),
                         luma8(CODE_BRT), luma8(o_mid), o_dark);
        end
    endtask

    //-------------------------------------------------------------------------
    // 6. 高光占比判据（大面积：31%）
    //-------------------------------------------------------------------------
    task test_bright_ratio;
        integer f;
        begin
            do_reset;
            enable = 1;
            for (f = 0; f < 60; f = f + 1) send_spot_frame(20);
            checks = checks + 1;
            if (dut.s_cur > -10'sd200) begin
                errors = errors + 1;
                $display("  FAIL: 过曝占比 31%% 未判强光，s_cur=%0d", dut.s_cur);
            end else if (tier_out !== 3'd4) begin
                errors = errors + 1;
                $display("  FAIL: 过曝占比 31%% 档位=%0d，应为 4(GLARE)", tier_out);
            end else begin
                $display("  OK: 平均亮度 %0d（只够 BRIGHT 档）但 31%% 过曝 -> s_cur=%0d 判为 GLARE",
                         mean_out, dut.s_cur);
            end
        end
    endtask

    //-------------------------------------------------------------------------
    // 6b. 小面积强光（5% 阈值两侧）—— 对应现场「手电照摄像头一直是 NORMAL」
    //
    //   8x8 帧：BR_GLARE = ceil(64*5%) = 4 个像素
    //     nb=2 -> 3.1% <  5%  -> 必须保持 NORMAL（不能过敏）
    //     nb=4 -> 6.25% >= 5% -> 必须判 GLARE（AEC 压不住截断面积）
    //   两种情况平均亮度都只有 126~129（按亮度只会判 NORMAL），
    //   所以这组用例专门验证「高光面积」这条判据真的接管了强光判定。
    //-------------------------------------------------------------------------
    task test_glare_ratio;
        integer f;
        begin
            do_reset;
            enable = 1;

            for (f = 0; f < 30; f = f + 1) send_spot_frame(2);
            checks = checks + 1;
            if (tier_out !== 3'd2 || dut.s_cur !== 10'sd0) begin
                errors = errors + 1;
                $display("  FAIL: 高光 3.1%%（<5%%）误判 tier=%0d s=%0d", tier_out, dut.s_cur);
            end else begin
                $display("  OK: 高光 3.1%% < 5%% -> 保持 NORMAL（Y=%0d G=%0d%%）",
                         mean_out, glare_out);
            end

            for (f = 0; f < 30; f = f + 1) send_spot_frame(4);
            checks = checks + 1;
            if (tier_out !== 3'd4) begin
                errors = errors + 1;
                $display("  FAIL: 高光 6.25%% 未判强光 tier=%0d s=%0d", tier_out, dut.s_cur);
            end else if (dut.s_cur > -10'sd200) begin
                errors = errors + 1;
                $display("  FAIL: 高光 6.25%% 判了档但强度没压下去 s=%0d", dut.s_cur);
            end else begin
                $display("  OK: 高光 6.25%%（平均亮度仅 %0d）-> GLARE，s=%0d G=%0d%%",
                         mean_out, dut.s_cur, glare_out);
            end
        end
    endtask

    //-------------------------------------------------------------------------
    // 7. 从夜间直接关闭 -> 立刻直通
    //-------------------------------------------------------------------------
    task test_disable_passthrough;
        integer f, i;
        reg [15:0] v;
        integer local_err;
        begin
            local_err = 0;
            do_reset;
            enable = 1;
            for (f = 0; f < 40; f = f + 1) send_frame(CODE_NIGHT);
            if (dut.s_cur < 10'sd200) begin
                errors = errors + 1;
                $display("  FAIL: 关闭前 s_cur 未升上去 (%0d)", dut.s_cur);
            end
            enable = 0;
            for (i = 0; i < 80; i = i + 1) begin
                v = $random;
                @(negedge clk); valid_in = 1; pixel_in = v; x = i; y = 0;
                @(posedge clk); #1;
                if (pixel_out !== v) begin
                    local_err = local_err + 1; errors = errors + 1;
                    if (local_err <= 3) $display("  FAIL: 关闭后未直通 in=%h out=%h", v, pixel_out);
                end
            end
            @(negedge clk); valid_in = 0;
            checks = checks + 1;
            if (local_err == 0) $display("  OK: enable 1->0 后立即恢复原样输出");
        end
    endtask

    //-------------------------------------------------------------------------
    initial begin
        $display("==== advanced_task3 (高阶任务三 / 自适应 Gamma) 单元测试 ====");
        $display("帧尺寸 %0dx%0d，%0d 像素/帧", IMG_W, IMG_H, NPIX);

        $display("[1] 全码字穷举一致性");
        do_reset;
        sweep_codes(10'sd0);
        sweep_codes(10'sd255);
        sweep_codes(10'sd170);
        sweep_codes(-10'sd170);
        sweep_codes(-10'sd255);

        $display("[2] enable=0 直通隔离");
        test_isolation;

        $display("[3] 正常光照");
        test_normal;

        $display("[4] 夜间");
        test_night;

        $display("[5] 强光");
        test_glare;

        $display("[6] 高光占比判据（31%）");
        test_bright_ratio;

        $display("[6b] 小面积强光 / 5% 阈值两侧");
        test_glare_ratio;

        $display("[7] 关闭即直通");
        test_disable_passthrough;

        $display("--------------------------------------------------------");
        if (errors == 0)
            $display("ALL PASS: %0d 组检查全部通过", checks);
        else
            $display("FAILED: %0d 处错误 / %0d 组检查", errors, checks);
        $finish;
    end

    initial begin
        #50000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
