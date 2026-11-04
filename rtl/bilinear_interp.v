`timescale 1ns / 1ps

// Bilinear interpolation for RGB565 pixels.
//
// p00..p11 are the 2x2 source pixels around the Q12.4 source coordinate;
// fu/fv are the 4-bit fractional parts.  The output is registered, one clock
// after the input.  NEAREST=1 bypasses the interpolation for A/B testing.
module bilinear_interp #(
    parameter PIX_WIDTH = 16,
    parameter FRAC_BITS = 4,
    parameter NEAREST   = 0
)(
    input                       clk,
    input                       rst_n,
    input                       valid_in,
    input      [PIX_WIDTH-1:0]  p00,
    input      [PIX_WIDTH-1:0]  p01,
    input      [PIX_WIDTH-1:0]  p10,
    input      [PIX_WIDTH-1:0]  p11,
    input      [FRAC_BITS-1:0]  fu,
    input      [FRAC_BITS-1:0]  fv,
    output reg                  valid_out,
    output reg [PIX_WIDTH-1:0]  pixel_out
);

    localparam integer W = (1 << FRAC_BITS);   // 16 for Q4

    // Unpack RGB565 to 8-bit with bit replication.
    wire [7:0] r00 = {p00[15:11], p00[15:13]};
    wire [7:0] g00 = {p00[10:5],  p00[10:9]};
    wire [7:0] b00 = {p00[4:0],   p00[4:2]};
    wire [7:0] r01 = {p01[15:11], p01[15:13]};
    wire [7:0] g01 = {p01[10:5],  p01[10:9]};
    wire [7:0] b01 = {p01[4:0],   p01[4:2]};
    wire [7:0] r10 = {p10[15:11], p10[15:13]};
    wire [7:0] g10 = {p10[10:5],  p10[10:9]};
    wire [7:0] b10 = {p10[4:0],   p10[4:2]};
    wire [7:0] r11 = {p11[15:11], p11[15:13]};
    wire [7:0] g11 = {p11[10:5],  p11[10:9]};
    wire [7:0] b11 = {p11[4:0],   p11[4:2]};

    // Weights; the four weights sum to W*W = 256 for Q4.
    wire [FRAC_BITS:0] wu0 = W - fu;
    wire [FRAC_BITS:0] wu1 = fu;
    wire [FRAC_BITS:0] wv0 = W - fv;
    wire [FRAC_BITS:0] wv1 = fv;

    wire [2*FRAC_BITS:0] w00 = wu0 * wv0;
    wire [2*FRAC_BITS:0] w01 = wu1 * wv0;
    wire [2*FRAC_BITS:0] w10 = wu0 * wv1;
    wire [2*FRAC_BITS:0] w11 = wu1 * wv1;

    localparam integer ROUND = (1 << (2*FRAC_BITS - 1)); // 128

    wire [23:0] r_sum = r00*w00 + r01*w01 + r10*w10 + r11*w11 + ROUND;
    wire [23:0] g_sum = g00*w00 + g01*w01 + g10*w10 + g11*w11 + ROUND;
    wire [23:0] b_sum = b00*w00 + b01*w01 + b10*w10 + b11*w11 + ROUND;

    wire [7:0] r_out = r_sum[2*FRAC_BITS + 7 : 2*FRAC_BITS];
    wire [7:0] g_out = g_sum[2*FRAC_BITS + 7 : 2*FRAC_BITS];
    wire [7:0] b_out = b_sum[2*FRAC_BITS + 7 : 2*FRAC_BITS];

    wire [PIX_WIDTH-1:0] interp_pixel = {r_out[7:3], g_out[7:2], b_out[7:3]};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_out <= 1'b0;
            pixel_out <= {PIX_WIDTH{1'b0}};
        end else begin
            valid_out <= valid_in;
            if (NEAREST)
                pixel_out <= p00;
            else
                pixel_out <= interp_pixel;
        end
    end

endmodule
