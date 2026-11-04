`timescale 1ns / 1ps

module distortion_map_tb;
    reg clk;
    reg rst_n;
    reg valid_in;
    reg [10:0] u_ideal;
    reg [9:0] v_ideal;

    wire valid_identity;
    wire [14:0] u_identity;   // Q12.4
    wire [13:0] v_identity;   // Q12.4
    wire valid_calibrated;
    wire [14:0] u_source;     // Q12.4
    wire [13:0] v_source;     // Q12.4

    integer error_u;
    integer error_v;
    localparam integer MAP_LATENCY = 7;

    distortion_map #(
        .IMG_W(1280), .IMG_H(720),
        .FX_Q16(32'sd65536), .FY_Q16(32'sd65536),
        .CX_Q16(32'sd0), .CY_Q16(32'sd0),
        .K1_Q30(32'sd0), .K2_Q30(32'sd0),
        .P1_Q30(32'sd0), .P2_Q30(32'sd0),
        .INV_FX_Q32(64'sd4294967296),
        .INV_FY_Q32(64'sd4294967296)
    ) dut_identity (
        .clk(clk), .rst_n(rst_n), .valid_in(valid_in),
        .u_ideal(u_ideal), .v_ideal(v_ideal),
        // 动态参数端口必须显式接 0：悬空会变成 x，经 dyn_en?dyn:const 选择后
        // 把整条流水线污染成 x。
        .dyn_en(1'b0),
        .k1_dyn(32'sd0), .k2_dyn(32'sd0), .p1_dyn(32'sd0), .p2_dyn(32'sd0),
        .valid_out(valid_identity),
        .u_source(u_identity), .v_source(v_identity),
        .in_range()
    );

    distortion_map #(
        .IMG_W(1280), .IMG_H(720),
        .FX_Q16(32'sd72165667), .FY_Q16(32'sd71973804),
        .CX_Q16(32'sd40786220), .CY_Q16(32'sd25353105),
        .K1_Q30(32'sd5764105), .K2_Q30(-32'sd147684499),
        .P1_Q30(-32'sd3025473), .P2_Q30(-32'sd329861),
        .INV_FX_Q32(64'sd3900400), .INV_FY_Q32(64'sd3910798)
    ) dut_calibrated (
        .clk(clk), .rst_n(rst_n), .valid_in(valid_in),
        .u_ideal(u_ideal), .v_ideal(v_ideal),
        .valid_out(valid_calibrated),
        .u_source(u_source), .v_source(v_source)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    task check_point;
        input [10:0] test_u;
        input [9:0] test_v;
        input integer expected_u;
        input integer expected_v;
        begin
            @(negedge clk);
            u_ideal = test_u;
            v_ideal = test_v;
            valid_in = 1'b1;
            repeat (MAP_LATENCY) @(posedge clk);
            #1;

            if ((valid_identity !== 1'b1) ||
                (u_identity[14:4] !== test_u) || (v_identity[13:4] !== test_v) ||
                (u_identity[3:0] !== 4'd0) || (v_identity[3:0] !== 4'd0)) begin
                $display("FAIL identity ideal=(%0d,%0d) source=(%0d.%0d,%0d.%0d) valid=%b",
                         test_u, test_v,
                         u_identity[14:4], u_identity[3:0],
                         v_identity[13:4], v_identity[3:0], valid_identity);
                $finish;
            end

            error_u = u_source[14:4] - expected_u;
            if (error_u < 0) error_u = -error_u;
            error_v = v_source[13:4] - expected_v;
            if (error_v < 0) error_v = -error_v;

            if ((valid_calibrated !== 1'b1) ||
                (error_u > 1) || (error_v > 1)) begin
                $display("FAIL ideal=(%0d,%0d) source=(%0d.%0d,%0d.%0d) expected~=(%0d,%0d) error=(%0d,%0d) valid=%b",
                         test_u, test_v,
                         u_source[14:4], u_source[3:0],
                         v_source[13:4], v_source[3:0],
                         expected_u, expected_v, error_u, error_v, valid_calibrated);
                $finish;
            end else begin
                $display("PASS ideal=(%0d,%0d) identity=(%0d,%0d) source=(%0d.%0d,%0d.%0d) MATLAB~=(%0d,%0d) error=(%0d,%0d)",
                         test_u, test_v,
                         u_identity[14:4], v_identity[13:4],
                         u_source[14:4], u_source[3:0],
                         v_source[13:4], v_source[3:0],
                         expected_u, expected_v,
                         error_u, error_v);
            end
        end
    endtask

    initial begin
        rst_n = 1'b0;
        valid_in = 1'b0;
        u_ideal = 0;
        v_ideal = 0;
        repeat (2) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;

        check_point(11'd100,  10'd100, 104, 102);
        check_point(11'd640,  10'd100, 640,  99);
        check_point(11'd1180, 10'd100, 1173, 102);
        check_point(11'd100,  10'd360, 103, 359);
        check_point(11'd640,  10'd360, 640, 360);
        check_point(11'd1180, 10'd360, 1176, 359);
        check_point(11'd100,  10'd620, 105, 617);
        check_point(11'd640,  10'd620, 640, 620);
        check_point(11'd1180, 10'd620, 1173, 616);

        @(negedge clk);
        valid_in = 1'b0;
        repeat (MAP_LATENCY) @(posedge clk);
        #1;
        if ((valid_identity !== 1'b0) || (valid_calibrated !== 1'b0)) begin
            $display("FAIL valid_out should clear when valid_in is low");
            $finish;
        end

        $display("All identity and inverse-mapping checks passed.");
        $finish;
    end
endmodule







