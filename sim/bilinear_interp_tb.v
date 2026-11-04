`timescale 1ns / 1ps

module bilinear_interp_tb;
    localparam PIX_WIDTH = 16;
    localparam FRAC_BITS = 4;

    reg clk, rst_n, valid_in;
    reg [PIX_WIDTH-1:0] p00, p01, p10, p11;
    reg [FRAC_BITS-1:0] fu, fv;

    wire valid_interp, valid_nearest;
    wire [PIX_WIDTH-1:0] pix_interp, pix_nearest;

    integer errors;

    bilinear_interp #(
        .PIX_WIDTH(PIX_WIDTH), .FRAC_BITS(FRAC_BITS), .NEAREST(0)
    ) dut_interp (
        .clk(clk), .rst_n(rst_n), .valid_in(valid_in),
        .p00(p00), .p01(p01), .p10(p10), .p11(p11),
        .fu(fu), .fv(fv),
        .valid_out(valid_interp), .pixel_out(pix_interp)
    );

    bilinear_interp #(
        .PIX_WIDTH(PIX_WIDTH), .FRAC_BITS(FRAC_BITS), .NEAREST(1)
    ) dut_nearest (
        .clk(clk), .rst_n(rst_n), .valid_in(valid_in),
        .p00(p00), .p01(p01), .p10(p10), .p11(p11),
        .fu(fu), .fv(fv),
        .valid_out(valid_nearest), .pixel_out(pix_nearest)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    task check;
        input [15:0] a00, a01, a10, a11;
        input [3:0] afu, afv;
        input [15:0] expected;
        begin
            @(negedge clk);
            p00 = a00; p01 = a01; p10 = a10; p11 = a11;
            fu = afu; fv = afv;
            valid_in = 1'b1;
            @(posedge clk);
            #1;
            if ((valid_interp !== 1'b1) || (pix_interp !== expected)) begin
                $display("FAIL interp p=(%h,%h,%h,%h) fu=%0d fv=%0d got=%h expected=%h valid=%b",
                         a00, a01, a10, a11, afu, afv, pix_interp, expected, valid_interp);
                errors = errors + 1;
            end
            if ((valid_nearest !== 1'b1) || (pix_nearest !== a00)) begin
                $display("FAIL nearest got=%h expected=%h valid=%b",
                         pix_nearest, a00, valid_nearest);
                errors = errors + 1;
            end
            @(negedge clk);
            valid_in = 1'b0;
            @(posedge clk);
            #1;
            if ((valid_interp !== 1'b0) || (valid_nearest !== 1'b0)) begin
                $display("FAIL valid should clear");
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        rst_n = 1'b0;
        valid_in = 1'b0;
        p00 = 0; p01 = 0; p10 = 0; p11 = 0;
        fu = 0; fv = 0;
        errors = 0;

        repeat (2) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;

        // A constant image must pass through regardless of the fractions.
        check(16'h1234, 16'h1234, 16'h1234, 16'h1234, 4'd0,  4'd0,  16'h1234);
        check(16'h1234, 16'h1234, 16'h1234, 16'h1234, 4'd15, 4'd15, 16'h1234);

        // fu=fv=0 selects p00 exactly.
        check(16'h0000, 16'hFFFF, 16'hFFFF, 16'hFFFF, 4'd0, 4'd0, 16'h0000);

        // Halfway vertically between black and white.
        check(16'h0000, 16'h0000, 16'hFFFF, 16'hFFFF, 4'd0, 4'd8, 16'h8410);

        // Halfway in both directions: 1/4 black + 3/4 white.
        check(16'h0000, 16'hFFFF, 16'hFFFF, 16'hFFFF, 4'd8, 4'd8, 16'hBDF7);

        // Integer pixels with a fractional coordinate (0.5,0.25).
        check(16'h0000, 16'hFFFF, 16'h0000, 16'hFFFF, 4'd8, 4'd4, 16'h8410);

        if (errors == 0)
            $display("All bilinear checks passed.");
        else
            $display("Bilinear checks failed: %0d errors.", errors);
        $finish;
    end
endmodule
