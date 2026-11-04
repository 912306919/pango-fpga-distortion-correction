`timescale 1ns / 1ps

// Four-bank circular source-image cache for bilinear backward distortion mapping.
//
// Each bank stores every other row and every other column, so the four pixels
// needed by bilinear interpolation (u,v), (u+1,v), (u,v+1), (u+1,v+1) live in
// four different banks.  Each bank is a separate simple-dual-port RAM instance,
// which lets the synthesizer infer block RAM reliably.
//
// A row is marked available only after all IMG_W pixels have arrived, so a
// partially overwritten row is never reported as valid.  The Q12.4 fractional
// bits of the source coordinate are passed through to the interpolator.
module distortion_line_buffer #(
    parameter IMG_W       = 1280,
    parameter IMG_H       = 720,
    parameter PIX_WIDTH   = 16,
    parameter FRAC_BITS   = 4,
    parameter X_BITS      = 11,
    parameter Y_BITS      = 10,
    parameter SLOT_BITS   = 6,     // total cached lines = 2^SLOT_BITS
    parameter ADDR_BITS   = 15     // bank address width
)(
    input                       clk,
    input                       rst_n,
    input                       frame_start,

    input                       in_valid,
    input      [PIX_WIDTH-1:0]  in_pixel,

    input                       map_valid,
    input                       map_in_range,
    input      [X_BITS+FRAC_BITS-1:0] u_source,
    input      [Y_BITS+FRAC_BITS-1:0] v_source,

    output reg                  out_valid,
    output reg                  out_in_range,
    output wire [PIX_WIDTH-1:0] p00,
    output wire [PIX_WIDTH-1:0] p01,
    output wire [PIX_WIDTH-1:0] p10,
    output wire [PIX_WIDTH-1:0] p11,
    output reg [FRAC_BITS-1:0]  fu,
    output reg [FRAC_BITS-1:0]  fv
);

    localparam BANK_SLOTS = (1 << (SLOT_BITS - 1));
    localparam BANK_DEPTH = BANK_SLOTS * (IMG_W / 2);
    localparam TAG_DEPTH  = 4 * BANK_SLOTS;
    localparam TAG_BITS   = SLOT_BITS + 1;

    reg [Y_BITS-1:0]    line_tag [0:TAG_DEPTH-1];
    reg [TAG_DEPTH-1:0] line_valid;

    reg [X_BITS-1:0]    in_x;
    reg [Y_BITS-1:0]    in_y;

    // ---------------------------------------------------------------- write
    // 关键不变量：写侧 slot 必须与读侧 slot = v[SLOT_BITS-1:1] 同坐标系。
    // 读侧直接用「全局行号 v」的高 SLOT_BITS-1 位寻址；写侧也必须用
    // 「写入行的全局行号 in_y」的高位，而不能用「本帧递推到第几行」的
    // 独立计数器——后者在 frame_start 清零瞬间会与读侧相位错开若干行，
    // 此后 line_tag[idx] == v0 永远失败，整帧 4 个 hit 全 0 → 整屏黑。
    // in_y 本身就是 0..IMG_H-1 的全局行号（rd_buf 每帧从第 0 行起输出，
    // in_valid 有效期间 in_y 恰好等于全局行号），直接取高位即可。
    wire [1:0]                 wr_bank      = {in_y[0], in_x[0]};
    wire [SLOT_BITS-2:0]       wr_bank_slot = in_y[SLOT_BITS-1:1];
    wire [ADDR_BITS-1:0]  wr_addr      = wr_bank_slot * (IMG_W/2) +
                                             in_x[X_BITS-1:1];
    wire [TAG_BITS-1:0]        wr_idx_even  = {in_y[0], 1'b0} * BANK_SLOTS +
                                             wr_bank_slot;
    wire [TAG_BITS-1:0]        wr_idx_odd   = {in_y[0], 1'b1} * BANK_SLOTS +
                                             wr_bank_slot;

    wire wr_active = in_valid && (in_y < IMG_H) && !frame_start;
    wire we0 = wr_active && (wr_bank == 2'd0);
    wire we1 = wr_active && (wr_bank == 2'd1);
    wire we2 = wr_active && (wr_bank == 2'd2);
    wire we3 = wr_active && (wr_bank == 2'd3);

    // Row tags/valid.  A row is invalid while it is being overwritten and
    // becomes valid only when its last pixel arrives.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            line_valid <= {TAG_DEPTH{1'b0}};
        end else if (wr_active) begin
            if (in_x == 0) begin
                line_valid[wr_idx_even] <= 1'b0;
                line_valid[wr_idx_odd]  <= 1'b0;
            end

            if (in_x == IMG_W-1) begin
                line_tag[wr_idx_even]   <= in_y;
                line_tag[wr_idx_odd]    <= in_y;
                line_valid[wr_idx_even] <= 1'b1;
                line_valid[wr_idx_odd]  <= 1'b1;
            end
        end
    end

    // 与其他三个模块同理：frame_start 是单拍脉冲，电平触发会让
    // in_x/in_y 在 1650 拍内被持续清零，写指针在帧头无法推进，
    // 行缓存写不进数据 → miss → 黑屏。上升沿检测保证每帧只清零一次。
    reg frame_start_d;
    wire frame_start_rise;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) frame_start_d <= 1'b0;
        else        frame_start_d <= frame_start;
    end
    assign frame_start_rise = frame_start && !frame_start_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_x    <= {X_BITS{1'b0}};
            in_y    <= {Y_BITS{1'b0}};
        end else if (frame_start_rise) begin
            in_x    <= {X_BITS{1'b0}};
            in_y    <= {Y_BITS{1'b0}};
        end else if (in_valid && (in_y < IMG_H)) begin
            if (in_x == IMG_W-1) begin
                in_x    <= {X_BITS{1'b0}};
                in_y    <= in_y + 1'b1;
            end else begin
                in_x <= in_x + 1'b1;
            end
        end
    end

    // ---------------------------------------------------------------- read
    wire [X_BITS-1:0] u_int = u_source[X_BITS+FRAC_BITS-1:FRAC_BITS];
    wire [Y_BITS-1:0] v_int = v_source[Y_BITS+FRAC_BITS-1:FRAC_BITS];
    wire [FRAC_BITS-1:0] fu_in = u_source[FRAC_BITS-1:0];
    wire [FRAC_BITS-1:0] fv_in = v_source[FRAC_BITS-1:0];

    wire [X_BITS-1:0] u0 = u_int;
    wire [X_BITS-1:0] u1 = (u_int == IMG_W-1) ? u_int : u_int + 1'b1;
    wire [Y_BITS-1:0] v0 = v_int;
    wire [Y_BITS-1:0] v1 = (v_int == IMG_H-1) ? v_int : v_int + 1'b1;

    wire [1:0] bank00 = {v0[0], u0[0]};
    wire [1:0] bank01 = {v0[0], u1[0]};
    wire [1:0] bank10 = {v1[0], u0[0]};
    wire [1:0] bank11 = {v1[0], u1[0]};

    wire [SLOT_BITS-2:0] slot00 = v0[SLOT_BITS-1:1];
    wire [SLOT_BITS-2:0] slot01 = v0[SLOT_BITS-1:1];
    wire [SLOT_BITS-2:0] slot10 = v1[SLOT_BITS-1:1];
    wire [SLOT_BITS-2:0] slot11 = v1[SLOT_BITS-1:1];

    wire [ADDR_BITS-1:0] addr00 = slot00 * (IMG_W/2) + u0[X_BITS-1:1];
    wire [ADDR_BITS-1:0] addr01 = slot01 * (IMG_W/2) + u1[X_BITS-1:1];
    wire [ADDR_BITS-1:0] addr10 = slot10 * (IMG_W/2) + u0[X_BITS-1:1];
    wire [ADDR_BITS-1:0] addr11 = slot11 * (IMG_W/2) + u1[X_BITS-1:1];

    wire [TAG_BITS-1:0] idx00 = bank00 * BANK_SLOTS + slot00;
    wire [TAG_BITS-1:0] idx01 = bank01 * BANK_SLOTS + slot01;
    wire [TAG_BITS-1:0] idx10 = bank10 * BANK_SLOTS + slot10;
    wire [TAG_BITS-1:0] idx11 = bank11 * BANK_SLOTS + slot11;

    wire same00 = wr_active && (wr_bank == bank00) && (wr_bank_slot == slot00);
    wire same01 = wr_active && (wr_bank == bank01) && (wr_bank_slot == slot01);
    wire same10 = wr_active && (wr_bank == bank10) && (wr_bank_slot == slot10);
    wire same11 = wr_active && (wr_bank == bank11) && (wr_bank_slot == slot11);

    wire hit00 = line_valid[idx00] && (line_tag[idx00] == v0) && !same00;
    wire hit01 = line_valid[idx01] && (line_tag[idx01] == v0) && !same01;
    wire hit10 = line_valid[idx10] && (line_tag[idx10] == v1) && !same10;
    wire hit11 = line_valid[idx11] && (line_tag[idx11] == v1) && !same11;

    // Route each sample's read address to its own bank.  The four samples are
    // in four distinct banks, so each RAM receives exactly one address.
    reg [ADDR_BITS-1:0] raddr0, raddr1, raddr2, raddr3;
    always @(*) begin
        raddr0 = {ADDR_BITS{1'b0}};
        raddr1 = {ADDR_BITS{1'b0}};
        raddr2 = {ADDR_BITS{1'b0}};
        raddr3 = {ADDR_BITS{1'b0}};

        case (bank00)
            2'd0: raddr0 = addr00;
            2'd1: raddr1 = addr00;
            2'd2: raddr2 = addr00;
            2'd3: raddr3 = addr00;
        endcase
        case (bank01)
            2'd0: raddr0 = addr01;
            2'd1: raddr1 = addr01;
            2'd2: raddr2 = addr01;
            2'd3: raddr3 = addr01;
        endcase
        case (bank10)
            2'd0: raddr0 = addr10;
            2'd1: raddr1 = addr10;
            2'd2: raddr2 = addr10;
            2'd3: raddr3 = addr10;
        endcase
        case (bank11)
            2'd0: raddr0 = addr11;
            2'd1: raddr1 = addr11;
            2'd2: raddr2 = addr11;
            2'd3: raddr3 = addr11;
        endcase
    end

    wire [PIX_WIDTH-1:0] r0, r1, r2, r3;

    distortion_line_ram #(
        .PIX_WIDTH(PIX_WIDTH), .DEPTH(BANK_DEPTH), .ADDR_BITS(ADDR_BITS)
    ) ram0 (
        .clk(clk), .we(we0), .waddr(wr_addr), .wdata(in_pixel),
        .raddr(raddr0), .rdata(r0)
    );
    distortion_line_ram #(
        .PIX_WIDTH(PIX_WIDTH), .DEPTH(BANK_DEPTH), .ADDR_BITS(ADDR_BITS)
    ) ram1 (
        .clk(clk), .we(we1), .waddr(wr_addr), .wdata(in_pixel),
        .raddr(raddr1), .rdata(r1)
    );
    distortion_line_ram #(
        .PIX_WIDTH(PIX_WIDTH), .DEPTH(BANK_DEPTH), .ADDR_BITS(ADDR_BITS)
    ) ram2 (
        .clk(clk), .we(we2), .waddr(wr_addr), .wdata(in_pixel),
        .raddr(raddr2), .rdata(r2)
    );
    distortion_line_ram #(
        .PIX_WIDTH(PIX_WIDTH), .DEPTH(BANK_DEPTH), .ADDR_BITS(ADDR_BITS)
    ) ram3 (
        .clk(clk), .we(we3), .waddr(wr_addr), .wdata(in_pixel),
        .raddr(raddr3), .rdata(r3)
    );

    // The RAM data is registered, so the bank select must be registered too;
    // otherwise the mux uses the next request's bank with the current data.
    reg [1:0] bank00_d, bank01_d, bank10_d, bank11_d;
    reg [PIX_WIDTH-1:0] p00_mux, p01_mux, p10_mux, p11_mux;
    always @(*) begin
        case (bank00_d)
            2'd0: p00_mux = r0;
            2'd1: p00_mux = r1;
            2'd2: p00_mux = r2;
            2'd3: p00_mux = r3;
        endcase
        case (bank01_d)
            2'd0: p01_mux = r0;
            2'd1: p01_mux = r1;
            2'd2: p01_mux = r2;
            2'd3: p01_mux = r3;
        endcase
        case (bank10_d)
            2'd0: p10_mux = r0;
            2'd1: p10_mux = r1;
            2'd2: p10_mux = r2;
            2'd3: p10_mux = r3;
        endcase
        case (bank11_d)
            2'd0: p11_mux = r0;
            2'd1: p11_mux = r1;
            2'd2: p11_mux = r2;
            2'd3: p11_mux = r3;
        endcase
    end

    assign p00 = p00_mux;
    assign p01 = p01_mux;
    assign p10 = p10_mux;
    assign p11 = p11_mux;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            out_in_range <= 1'b0;
            fu        <= {FRAC_BITS{1'b0}};
            fv        <= {FRAC_BITS{1'b0}};
        end else begin
            fu        <= fu_in;
            fv        <= fv_in;
            bank00_d  <= bank00;
            bank01_d  <= bank01;
            bank10_d  <= bank10;
            bank11_d  <= bank11;
            out_in_range <= map_in_range;
            out_valid <= map_valid && hit00 && hit01 && hit10 && hit11;
        end
    end

endmodule

// Simple dual-port RAM: one synchronous write port, one synchronous read port.
module distortion_line_ram #(
    parameter PIX_WIDTH = 16,
    parameter DEPTH     = 20480,
    parameter ADDR_BITS = 15
)(
    input                   clk,
    input                   we,
    input  [ADDR_BITS-1:0]  waddr,
    input  [PIX_WIDTH-1:0]  wdata,
    input  [ADDR_BITS-1:0]  raddr,
    output reg [PIX_WIDTH-1:0] rdata
);
    reg [PIX_WIDTH-1:0] mem [0:DEPTH-1];

    always @(posedge clk) begin
        if (we)
            mem[waddr] <= wdata;
        rdata <= mem[raddr];
    end
endmodule
