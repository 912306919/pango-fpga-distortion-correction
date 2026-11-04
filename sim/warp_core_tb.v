`timescale 1ns / 1ps

// Integrated testbench: identity map -> four-bank line buffer -> bilinear.
// A 16x8 source image is written into the line buffer; the map is driven with
// the same 16x8 raster coordinates.  The output stream must reproduce the
// source image exactly (fu=fv=0, so bilinear must select p00).
module warp_core_tb;
    localparam IMG_W = 16;
    localparam IMG_H = 8;
    localparam FRAC_BITS = 4;
    localparam X_BITS = 4;
    localparam Y_BITS = 4;
    localparam SLOT_BITS = 4;

    reg clk, rst_n;
    reg frame_start, in_valid, display_de;
    reg [15:0] in_pixel;
    reg [10:0] u_ideal;
    reg [9:0] v_ideal;

    wire map_valid;
    wire [14:0] map_u;
    wire [13:0] map_v;

    wire lb_valid;
    wire [15:0] p00, p01, p10, p11;
    wire [FRAC_BITS-1:0] fu, fv;

    wire bl_valid;
    wire [15:0] pixel_out;

    integer r, c, out_count, errors, expected;

    distortion_map #(
        .IMG_W(IMG_W), .IMG_H(IMG_H),
        .FX_Q16(32'sd65536), .FY_Q16(32'sd65536),
        .CX_Q16(32'sd0), .CY_Q16(32'sd0),
        .K1_Q30(32'sd0), .K2_Q30(32'sd0),
        .P1_Q30(32'sd0), .P2_Q30(32'sd0),
        .INV_FX_Q32(64'sd4294967296),
        .INV_FY_Q32(64'sd4294967296)
    ) u_map (
        .clk(clk), .rst_n(rst_n), .valid_in(display_de),
        .u_ideal(u_ideal), .v_ideal(v_ideal),
        // 动态参数端口显式接 0，避免悬空 x 污染流水线
        .dyn_en(1'b0),
        .k1_dyn(32'sd0), .k2_dyn(32'sd0), .p1_dyn(32'sd0), .p2_dyn(32'sd0),
        .valid_out(map_valid), .u_source(map_u), .v_source(map_v),
        .in_range()
    );

    distortion_line_buffer #(
        .IMG_W(IMG_W), .IMG_H(IMG_H), .PIX_WIDTH(16),
        .FRAC_BITS(FRAC_BITS), .X_BITS(X_BITS), .Y_BITS(Y_BITS),
        .SLOT_BITS(SLOT_BITS), .ADDR_BITS(6)
    ) u_lb (
        .clk(clk), .rst_n(rst_n), .frame_start(frame_start),
        .in_valid(in_valid), .in_pixel(in_pixel),
        .map_valid(map_valid),
        .u_source(map_u[X_BITS+FRAC_BITS-1:0]),
        .v_source(map_v[Y_BITS+FRAC_BITS-1:0]),
        .out_valid(lb_valid),
        .p00(p00), .p01(p01), .p10(p10), .p11(p11),
        .fu(fu), .fv(fv)
    );

    bilinear_interp #(
        .PIX_WIDTH(16), .FRAC_BITS(FRAC_BITS), .NEAREST(0)
    ) u_bl (
        .clk(clk), .rst_n(rst_n), .valid_in(lb_valid),
        .p00(p00), .p01(p01), .p10(p10), .p11(p11),
        .fu(fu), .fv(fv),
        .valid_out(bl_valid), .pixel_out(pixel_out)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    // Collect the output stream and compare it with the source image raster.
    always @(posedge clk) begin
        if (bl_valid) begin
            expected = ((out_count / IMG_W) << 8) | (out_count % IMG_W);
            if (pixel_out !== expected) begin
                $display("FAIL out[%0d] got=%h expected=%h", out_count, pixel_out, expected);
                errors = errors + 1;
            end
            out_count = out_count + 1;
        end
    end

    initial begin
        rst_n = 1'b0;
        frame_start = 1'b0;
        in_valid = 1'b0;
        display_de = 1'b0;
        in_pixel = 0;
        u_ideal = 0;
        v_ideal = 0;
        out_count = 0;
        errors = 0;

        repeat (2) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;

        // Start of source frame.
        @(negedge clk);
        frame_start = 1'b1;
        @(negedge clk);
        frame_start = 1'b0;

        // Write the 16x8 source image.
        for (r = 0; r < IMG_H; r = r + 1) begin
            for (c = 0; c < IMG_W; c = c + 1) begin
                @(negedge clk);
                in_valid = 1'b1;
                in_pixel = (r << 8) | c;
            end
        end
        @(negedge clk);
        in_valid = 1'b0;
        repeat (4) @(posedge clk);

        // Drive the identity map with the same raster coordinates.
        for (r = 0; r < IMG_H; r = r + 1) begin
            for (c = 0; c < IMG_W; c = c + 1) begin
                @(negedge clk);
                display_de = 1'b1;
                u_ideal = c;
                v_ideal = r;
            end
        end
        @(negedge clk);
        display_de = 1'b0;
        u_ideal = 0;
        v_ideal = 0;

        // Drain the pipeline.
        repeat (30) @(posedge clk);

        if ((out_count != IMG_W*IMG_H) || (errors != 0)) begin
            $display("FAIL warp_core: out_count=%0d errors=%0d", out_count, errors);
        end else begin
            $display("All warp_core checks passed: %0d pixels.", out_count);
        end
        $finish;
    end
endmodule
