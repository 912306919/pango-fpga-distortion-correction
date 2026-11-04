module distortion_map #(
    parameter IMG_W = 1280, parameter IMG_H = 720,
    parameter signed [31:0] FX_Q16 = 32'sd72165667,
    parameter signed [31:0] FY_Q16 = 32'sd71973804,
    parameter signed [31:0] CX_Q16 = 32'sd40786220,
    parameter signed [31:0] CY_Q16 = 32'sd25353105,
    parameter signed [31:0] K1_Q30 = 32'sd5764105,
    parameter signed [31:0] K2_Q30 = -32'sd147684499,
    parameter signed [31:0] P1_Q30 = -32'sd3025473,
    parameter signed [31:0] P2_Q30 = -32'sd329861,
    // INVs are derived from the matching FX/FY above (2^32 / fx), not the
    // older 1100.7-pixel values used by the previous calibration.
    parameter signed [63:0] INV_FX_Q32 = 64'sd3900400,
    parameter signed [63:0] INV_FY_Q32 = 64'sd3910798
)(
    input clk, input rst_n, input valid_in,
    input [10:0] u_ideal, input [9:0] v_ideal,
    // 动态参数输入（高阶任务用）：dyn_en=1 时使用 k1_dyn..p2_dyn，
    // dyn_en=0 时完全使用下面的标定常量，基础任务行为不变。
    input dyn_en,
    input signed [31:0] k1_dyn, input signed [31:0] k2_dyn,
    input signed [31:0] p1_dyn, input signed [31:0] p2_dyn,
    // Q12.4 source coordinates: [14:4] integer pixels, [3:0] 1/16 pixel.
    // in_range 同时指示 u/v 是否都落在图像内：为 0 表示该像素的源坐标在
    // 1280×720 之外（移动 IMU 把画面推出边界），上层据此画黑边，
    // 而不是让边界行列被重复采样成拉伸条带。
    output reg valid_out, output reg [14:0] u_source,
    output reg [13:0] v_source, output reg in_range
);
    localparam signed [31:0] ONE_Q16 = 32'sd65536;

    // 参数选择：基础任务走常量，高阶任务走运行时寄存器
    wire signed [31:0] K1_EFF = dyn_en ? k1_dyn : K1_Q30;
    wire signed [31:0] K2_EFF = dyn_en ? k2_dyn : K2_Q30;
    wire signed [31:0] P1_EFF = dyn_en ? p1_dyn : P1_Q30;
    wire signed [31:0] P2_EFF = dyn_en ? p2_dyn : P2_Q30;

    // Seven registered stages, one coordinate accepted each clock.
    reg valid_p0, valid_p1, valid_p2, valid_p3, valid_p4, valid_p5;
    reg signed [31:0] du_p0, dv_p0;
    reg signed [31:0] xn_p1, yn_p1;
    reg signed [31:0] xn_p2, yn_p2, x2_p2, y2_p2, xy_p2, r2_p2;
    reg signed [31:0] xn_p3, yn_p3, xy_p3, r4_p3, k1r2_p3, tx_p3, ty_p3;
    reg signed [31:0] xn_p4, yn_p4, k1r2_p4, k2r4_p4, tang_x_p4, tang_y_p4;
    reg signed [31:0] xd_p5, yd_p5;
    wire signed [31:0] radial_p4;
    wire signed [63:0] radial_ext_p4;
    wire signed [63:0] xd_ext_p5, yd_ext_p5, fx_ext, fy_ext, cx_ext, cy_ext;
    wire signed [63:0] final_u_wide, final_v_wide;
    wire signed [31:0] final_u, final_v;

    assign radial_p4 = ONE_Q16 + k1r2_p4 + k2r4_p4;
    assign radial_ext_p4 = {{32{radial_p4[31]}}, radial_p4};
    assign xd_ext_p5 = {{32{xd_p5[31]}}, xd_p5};
    assign yd_ext_p5 = {{32{yd_p5[31]}}, yd_p5};
    assign fx_ext = {{32{FX_Q16[31]}}, FX_Q16};
    assign fy_ext = {{32{FY_Q16[31]}}, FY_Q16};
    assign cx_ext = {{32{CX_Q16[31]}}, CX_Q16};
    assign cy_ext = {{32{CY_Q16[31]}}, CY_Q16};
    // final_*_wide is Q32.32.  Shift by 28 to keep 4 fractional bits (Q4),
    // adding 2^27 for round-to-nearest at the 1/16-pixel LSB.
    assign final_u_wide = xd_ext_p5 * fx_ext + (cx_ext <<< 16) + 64'sd134217728;
    assign final_v_wide = yd_ext_p5 * fy_ext + (cy_ext <<< 16) + 64'sd134217728;
    assign final_u = final_u_wide >>> 28;
    assign final_v = final_v_wide >>> 28;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_p0 <= 0; valid_p1 <= 0; valid_p2 <= 0;
            valid_p3 <= 0; valid_p4 <= 0; valid_p5 <= 0; valid_out <= 0;
            du_p0 <= 0; dv_p0 <= 0;
            xn_p1 <= 0; yn_p1 <= 0;
            xn_p2 <= 0; yn_p2 <= 0; x2_p2 <= 0; y2_p2 <= 0;
            xy_p2 <= 0; r2_p2 <= 0;
            xn_p3 <= 0; yn_p3 <= 0; xy_p3 <= 0; r4_p3 <= 0;
            k1r2_p3 <= 0; tx_p3 <= 0; ty_p3 <= 0;
            xn_p4 <= 0; yn_p4 <= 0; k1r2_p4 <= 0; k2r4_p4 <= 0;
            tang_x_p4 <= 0; tang_y_p4 <= 0;
            xd_p5 <= 0; yd_p5 <= 0;
            u_source <= 0; v_source <= 0;
            in_range <= 1'b0;
        end else begin
            // Stage 0: Q16.16 pixel deltas.
            valid_p0 <= valid_in && (FX_Q16 != 0) && (FY_Q16 != 0) &&
                        (INV_FX_Q32 != 0) && (INV_FY_Q32 != 0);
            if (valid_in && (FX_Q16 != 0) && (FY_Q16 != 0) &&
                (INV_FX_Q32 != 0) && (INV_FY_Q32 != 0)) begin
                du_p0 <= ($signed({21'd0, u_ideal}) <<< 16) - CX_Q16;
                dv_p0 <= ($signed({22'd0, v_ideal}) <<< 16) - CY_Q16;
            end else begin
                du_p0 <= 0; dv_p0 <= 0;
            end

            // Stage 1: normalized coordinates.
            valid_p1 <= valid_p0;
            xn_p1 <= ($signed({{32{du_p0[31]}},du_p0}) * $signed(INV_FX_Q32)) >>> 32;
            yn_p1 <= ($signed({{32{dv_p0[31]}},dv_p0}) * $signed(INV_FY_Q32)) >>> 32;

            // Stage 2: x^2, y^2, xy, and r^2.
            valid_p2 <= valid_p1;
            xn_p2 <= xn_p1; yn_p2 <= yn_p1;
            x2_p2 <= ($signed({{32{xn_p1[31]}},xn_p1}) * $signed({{32{xn_p1[31]}},xn_p1})) >>> 16;
            y2_p2 <= ($signed({{32{yn_p1[31]}},yn_p1}) * $signed({{32{yn_p1[31]}},yn_p1})) >>> 16;
            xy_p2 <= ($signed({{32{xn_p1[31]}},xn_p1}) * $signed({{32{yn_p1[31]}},yn_p1})) >>> 16;
            r2_p2 <= (($signed({{32{xn_p1[31]}},xn_p1}) * $signed({{32{xn_p1[31]}},xn_p1})) >>> 16) +
                      (($signed({{32{yn_p1[31]}},yn_p1}) * $signed({{32{yn_p1[31]}},yn_p1})) >>> 16);

            // Stage 3: r^4, k1*r^2, and tangential polynomial arguments.
            valid_p3 <= valid_p2;
            xn_p3 <= xn_p2; yn_p3 <= yn_p2; xy_p3 <= xy_p2;
            r4_p3 <= ($signed({{32{r2_p2[31]}},r2_p2}) * $signed({{32{r2_p2[31]}},r2_p2})) >>> 16;
            k1r2_p3 <= ($signed({{32{K1_EFF[31]}},K1_EFF}) * $signed({{32{r2_p2[31]}},r2_p2})) >>> 30;
            tx_p3 <= r2_p2 + (x2_p2 <<< 1);
            ty_p3 <= r2_p2 + (y2_p2 <<< 1);

            // Stage 4: k2*r^4 and tangential products.
            valid_p4 <= valid_p3;
            xn_p4 <= xn_p3; yn_p4 <= yn_p3; k1r2_p4 <= k1r2_p3;
            k2r4_p4 <= ($signed({{32{K2_EFF[31]}},K2_EFF}) * $signed({{32{r4_p3[31]}},r4_p3})) >>> 30;
            tang_x_p4 <= (($signed({{32{P1_EFF[31]}},P1_EFF}) * $signed({{32{xy_p3[31]}},xy_p3})) >>> 29) +
                         (($signed({{32{P2_EFF[31]}},P2_EFF}) * $signed({{32{tx_p3[31]}},tx_p3})) >>> 30);
            tang_y_p4 <= (($signed({{32{P1_EFF[31]}},P1_EFF}) * $signed({{32{ty_p3[31]}},ty_p3})) >>> 30) +
                         (($signed({{32{P2_EFF[31]}},P2_EFF}) * $signed({{32{xy_p3[31]}},xy_p3})) >>> 29);

            // Stage 5: radial and tangential distortion.
            valid_p5 <= valid_p4;
            xd_p5 <= (($signed({{32{xn_p4[31]}},xn_p4}) * radial_ext_p4) >>> 16) + tang_x_p4;
            yd_p5 <= (($signed({{32{yn_p4[31]}},yn_p4}) * radial_ext_p4) >>> 16) + tang_y_p4;

            // Stage 6: convert to Q12.4 source pixels.  Clamp the warped
            // coordinate at the image edge instead of declaring it invalid.
            // Otherwise a few out-of-range samples at the bottom edge become
            // black pixels and make the whole corrected image look shifted.
            valid_out <= valid_p5;
            in_range  <= valid_p5 && (final_u >= 0) && (final_u < (IMG_W * 16)) &&
                                    (final_v >= 0) && (final_v < (IMG_H * 16));
            if (valid_p5) begin
                if (final_u < 0)
                    u_source <= 15'd0;
                else if (final_u >= (IMG_W * 16))
                    u_source <= (IMG_W * 16 - 1);
                else
                    u_source <= final_u[14:0];
                if (final_v < 0)
                    v_source <= 14'd0;
                else if (final_v >= (IMG_H * 16))
                    v_source <= (IMG_H * 16 - 1);
                else
                    v_source <= final_v[13:0];
            end else begin
                u_source <= 0; v_source <= 0;
            end
        end
    end
endmodule

