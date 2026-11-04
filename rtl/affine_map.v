`timescale 1ns / 1ps
//=============================================================================
// affine_map.v -- 仿射坐标变换（2 级流水）
//
//   输入理想输出坐标 (u,v)，输出反向映射后的坐标（整数像素，限幅到画面内）。
//   与 distortion_map 串联：仿射在前、畸变在后，符合「先补偿姿态、再补偿镜头」
//   的物理顺序。
//
//   系数由 affine_coeff 每帧更新；基础任务下为单位矩阵，输出等于输入延后 2 拍，
//   不影响画面内容。
//=============================================================================
module affine_map #(
    parameter W_ACT = 1280,
    parameter H_ACT = 720
)(
    input  wire                clk,
    input  wire                rst_n,
    input  wire                valid_in,
    input  wire [10:0]         u_in,
    input  wire [9:0]          v_in,
    input  wire signed [31:0]  h11,
    input  wire signed [31:0]  h12,
    input  wire signed [31:0]  h13,
    input  wire signed [31:0]  h21,
    input  wire signed [31:0]  h22,
    input  wire signed [31:0]  h23,
    output reg                 valid_out,
    output reg  [10:0]         u_out,
    output reg  [9:0]          v_out
);

    // 第 1 级：乘加项
    reg signed [43:0] pu, pv, qu, qv;
    reg               valid_s1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pu <= 44'sd0; pv <= 44'sd0; qu <= 44'sd0; qv <= 44'sd0;
            valid_s1 <= 1'b0;
        end else begin
            pu <= h11 * $signed({1'b0, u_in});
            pv <= h12 * $signed({1'b0, v_in});
            qu <= h21 * $signed({1'b0, u_in});
            qv <= h22 * $signed({1'b0, v_in});
            valid_s1 <= valid_in;
        end
    end

    // 第 2 级：求和、定点化、限幅
    wire signed [63:0] acc_u = $signed({{20{pu[43]}}, pu}) + $signed({{20{pv[43]}}, pv}) + $signed({{32{h13[31]}}, h13});
    wire signed [63:0] acc_v = $signed({{20{qu[43]}}, qu}) + $signed({{20{qv[43]}}, qv}) + $signed({{32{h23[31]}}, h23});

    wire signed [63:0] u_shift = acc_u >>> 16;
    wire signed [63:0] v_shift = acc_v >>> 16;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_out <= 1'b0;
            u_out     <= 11'd0;
            v_out     <= 10'd0;
        end else begin
            valid_out <= valid_s1;
            if (u_shift < 0)
                u_out <= 11'd0;
            else if (u_shift >= W_ACT)
                u_out <= W_ACT - 1;
            else
                u_out <= u_shift[10:0];

            if (v_shift < 0)
                v_out <= 10'd0;
            else if (v_shift >= H_ACT)
                v_out <= H_ACT - 1;
            else
                v_out <= v_shift[9:0];
        end
    end

endmodule
