`timescale 1ns / 1ps

// Small-image testbench for the four-bank line buffer.
// The source image is 16x8; pixel value = (row << 8) | column.
module distortion_line_buffer_tb;
    localparam IMG_W = 16;
    localparam IMG_H = 8;
    localparam PIX_WIDTH = 16;
    localparam FRAC_BITS = 4;
    localparam X_BITS = 4;
    localparam Y_BITS = 4;
    localparam SLOT_BITS = 4;

    reg clk, rst_n, frame_start, in_valid, map_valid;
    reg [PIX_WIDTH-1:0] in_pixel;
    reg [X_BITS+FRAC_BITS-1:0] u_source;
    reg [Y_BITS+FRAC_BITS-1:0] v_source;

    wire out_valid;
    wire [PIX_WIDTH-1:0] p00, p01, p10, p11;
    wire [FRAC_BITS-1:0] fu, fv;

    integer r, c, errors;

    distortion_line_buffer #(
        .IMG_W(IMG_W), .IMG_H(IMG_H), .PIX_WIDTH(PIX_WIDTH),
        .FRAC_BITS(FRAC_BITS), .X_BITS(X_BITS), .Y_BITS(Y_BITS),
        .SLOT_BITS(SLOT_BITS), .ADDR_BITS(6)
    ) dut (
        .clk(clk), .rst_n(rst_n), .frame_start(frame_start),
        .in_valid(in_valid), .in_pixel(in_pixel),
        .map_valid(map_valid), .u_source(u_source), .v_source(v_source),
        .out_valid(out_valid),
        .p00(p00), .p01(p01), .p10(p10), .p11(p11),
        .fu(fu), .fv(fv)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    task check_sample;
        input [X_BITS-1:0] ui;
        input [Y_BITS-1:0] vi;
        input [FRAC_BITS-1:0] fui;
        input [FRAC_BITS-1:0] fvi;
        input [PIX_WIDTH-1:0] e00, e01, e10, e11;
        begin
            @(negedge clk);
            u_source = {ui, fui};
            v_source = {vi, fvi};
            map_valid = 1'b1;
            @(posedge clk);
            #1;
            if (out_valid !== 1'b1) begin
                $display("FAIL valid u=%0d v=%0d got valid=%b", ui, vi, out_valid);
                errors = errors + 1;
            end
            if ((p00 !== e00) || (p01 !== e01) ||
                (p10 !== e10) || (p11 !== e11)) begin
                $display("FAIL u=%0d v=%0d got (%h,%h,%h,%h) expected (%h,%h,%h,%h)",
                         ui, vi, p00, p01, p10, p11, e00, e01, e10, e11);
                errors = errors + 1;
            end
            if ((fu !== fui) || (fv !== fvi)) begin
                $display("FAIL frac u=%0d v=%0d got (%h,%h) expected (%h,%h)",
                         ui, vi, fu, fv, fui, fvi);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        rst_n = 1'b0;
        frame_start = 1'b0;
        in_valid = 1'b0;
        map_valid = 1'b0;
        in_pixel = 0;
        u_source = 0;
        v_source = 0;
        errors = 0;

        repeat (2) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;

        // Reset the write counters at the start of a frame.
        @(negedge clk);
        frame_start = 1'b1;
        @(negedge clk);
        frame_start = 1'b0;

        // Write the 16x8 source image in raster order.
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

        // Center sample with fractions 0.5 / 0.25.
        check_sample(4'd4, 4'd3, 4'd8, 4'd4,
                     (3 << 8) | 4, (3 << 8) | 5,
                     (4 << 8) | 4, (4 << 8) | 5);

        // Top-left corner.
        check_sample(4'd0, 4'd0, 4'd0, 4'd0,
                     16'h0000, 16'h0001, 16'h0100, 16'h0101);

        // Bottom-right corner: u+1 and v+1 are clamped.
        check_sample(4'd15, 4'd7, 4'd0, 4'd0,
                     (7 << 8) | 15, (7 << 8) | 15,
                     (7 << 8) | 15, (7 << 8) | 15);

        // Right edge with a non-clamped row.
        check_sample(4'd15, 4'd6, 4'd0, 4'd0,
                     (6 << 8) | 15, (6 << 8) | 15,
                     (7 << 8) | 15, (7 << 8) | 15);

        // Bottom edge with a non-clamped column.
        check_sample(4'd14, 4'd7, 4'd0, 4'd0,
                     (7 << 8) | 14, (7 << 8) | 15,
                     (7 << 8) | 14, (7 << 8) | 15);

        // Nonaligned sample.
        check_sample(4'd5, 4'd2, 4'd12, 4'd3,
                     (2 << 8) | 5, (2 << 8) | 6,
                     (3 << 8) | 5, (3 << 8) | 6);

        if (errors == 0)
            $display("All line-buffer checks passed.");
        else
            $display("Line-buffer checks failed: %0d errors.", errors);
        $finish;
    end
endmodule
