`timescale 1ns / 1ps
//=============================================================================
// oled_imu_ctrl.v
// SSD1306（0.96 寸 128x64 I2C OLED）初始化 + JY61P 角度实时显示
//
// 屏幕布局（每行 16 个 8x16 字符，占 2 页）：
//   行0 (page0-1): YAW:    ±xxx.xx
//   行1 (page2-3): PITCH:  ±xxx.xx
//   行2 (page4-5): ROLL:   ±xxx.xx
//
// 工作流程：
//   PH_PWR  上电等待 PWR_CYCLES（默认 100ms）
//   PH_CMD  I2C 事务1：0x78 + 0x00 + 32 条初始化/寻址命令
//   PH_DAT  I2C 事务2：0x78 + 0x40 + 1024 字节显存（整屏）
//   PH_WAIT 等待 REFRESH_CYCLES（默认 200ms）后回到 PH_DAT 周期性刷新
//
// 显存不占 RAM：按 (page, col) 实时查 font_8x16 ROM 流式发出。
// 每屏开始时把角度 BCD 锁存到 d*，避免刷屏中途数据更新造成画面撕裂。
//=============================================================================
module oled_imu_ctrl #(
    parameter integer PWR_CYCLES     = 2700000,   // 100ms @27MHz
    parameter integer REFRESH_CYCLES = 5400000,   // 200ms @27MHz
    parameter [7:0]   OLED_ADDR      = 8'h78      // 0x3C<<1；若模块是 0x3D 改 8'h7A
)(
    input  wire               clk,
    input  wire               rst_n,

    input  wire signed [15:0] yaw,
    input  wire signed [15:0] pitch,
    input  wire signed [15:0] roll,
    input  wire               upd,        // angle_valid 脉冲

    output wire        tx_valid,
    output wire [7:0]  tx_data,
    output wire        tx_last,
    input  wire        tx_ready,

    output wire        done               // 首屏已完成
);

    //------------------------------------------------------------------
    // 阶段
    //------------------------------------------------------------------
    localparam [1:0] PH_PWR  = 2'd0,
                     PH_CMD  = 2'd1,
                     PH_DAT  = 2'd2,
                     PH_WAIT = 2'd3;

    reg [1:0]  phase;
    reg [31:0] cnt;
    reg [10:0] idx;

    //------------------------------------------------------------------
    // 角度原始值 -> 符号 + 5 位 BCD（单位 0.01 度）
    //   deg*100 = raw * 18000 / 32768 = raw * 1125 / 2048
    // PDS 综合器不支持除法/取模运算符；若按组合逻辑展开减法链，
    // 综合/布线都很慢。这里改用一个共享减法器分时逐位转换：
    // upd 脉冲后约 100 个时钟（<4us）完成三路换算，远小于 10ms 数据周期。
    //------------------------------------------------------------------
    localparam [2:0] CV_IDLE = 3'd0,
                     CV_MUL  = 3'd1,
                     CV_D4   = 3'd2,
                     CV_D3   = 3'd3,
                     CV_D2   = 3'd4,
                     CV_D1   = 3'd5,
                     CV_D0   = 3'd6,
                     CV_NEXT = 3'd7;

    reg [2:0]  cv_state;
    reg [1:0]  cv_ch;            // 0=yaw 1=pitch 2=roll
    reg        cv_sign;
    reg [31:0] cv_q;
    reg [3:0]  cv_dig;
    reg [31:0] cv_base;          // 当前位权：10000/1000/100/10
    reg [19:0] cv_acc;           // 已换算出的 BCD 位（左移拼接）

    reg [20:0] byaw, bpitch, broll;   // 换算完成后更新
    reg [20:0] dyaw, dpitch, droll;   // 每屏开始时锁存

    wire signed [15:0] cv_in = (cv_ch == 2'd0) ? yaw :
                               (cv_ch == 2'd1) ? pitch : roll;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cv_state <= CV_IDLE;
            cv_ch    <= 2'd0;
            cv_sign  <= 1'b0;
            cv_q     <= 32'd0;
            cv_dig   <= 4'd0;
            cv_base  <= 32'd0;
            cv_acc   <= 20'd0;
            byaw     <= 21'd0;
            bpitch   <= 21'd0;
            broll    <= 21'd0;
        end else begin
            case (cv_state)
                CV_IDLE: begin
                    if (upd) begin
                        cv_ch    <= 2'd0;
                        cv_state <= CV_MUL;
                    end
                end

                // 幅度 * 1125 >> 11 -> 0.01 度整数（0..18000）
                CV_MUL: begin
                    cv_sign  <= (cv_in < 0);
                    cv_q     <= (((cv_in < 0) ? -cv_in : cv_in) * 1125) >> 11;
                    cv_dig   <= 4'd0;
                    cv_base  <= 32'd10000;
                    cv_acc   <= 20'd0;
                    cv_state <= CV_D4;
                end

                // 每一位：q >= base 就减 base 并计 1，否则收下该位进入下一位
                CV_D4, CV_D3, CV_D2, CV_D1: begin
                    if (cv_q >= cv_base) begin
                        cv_q   <= cv_q - cv_base;
                        cv_dig <= cv_dig + 4'd1;
                    end else begin
                        cv_acc <= {cv_acc[15:0], cv_dig};
                        cv_dig <= 4'd0;
                        case (cv_state)
                            CV_D4: begin cv_base <= 32'd1000; cv_state <= CV_D3; end
                            CV_D3: begin cv_base <= 32'd100;  cv_state <= CV_D2; end
                            CV_D2: begin cv_base <= 32'd10;   cv_state <= CV_D1; end
                            default:                            cv_state <= CV_D0;
                        endcase
                    end
                end

                // 余数即最后一位，连同符号写入对应通道
                CV_D0: begin
                    case (cv_ch)
                        2'd0:    byaw   <= {cv_sign, cv_acc[15:0], cv_q[3:0]};
                        2'd1:    bpitch <= {cv_sign, cv_acc[15:0], cv_q[3:0]};
                        default: broll  <= {cv_sign, cv_acc[15:0], cv_q[3:0]};
                    endcase
                    cv_state <= CV_NEXT;
                end

                CV_NEXT: begin
                    if (cv_ch == 2'd2) begin
                        cv_state <= CV_IDLE;
                    end else begin
                        cv_ch    <= cv_ch + 2'd1;
                        cv_state <= CV_MUL;
                    end
                end

                default: cv_state <= CV_IDLE;
            endcase
        end
    end

    //------------------------------------------------------------------
    // 字符布局：pos0-7 标签，8 符号，9-11 整数（高位 blank），12 '.'，13-14 小数
    //------------------------------------------------------------------
    function automatic [7:0] line_char(input [1:0] line, input [3:0] pos);
        reg [20:0] b;
        reg [7:0]  r;
        begin
            case (line)
                2'd0:    b = dyaw;
                2'd1:    b = dpitch;
                default: b = droll;
            endcase
            case (pos)
                4'd8:    r = b[20] ? 8'd45 : 8'd32;                          // '-' / ' '
                4'd9:    r = (b[19:16] == 4'd0) ? 8'd32
                             : (8'd48 + {4'd0, b[19:16]});                   // 百位
                4'd10:   r = (b[19:16] == 4'd0 && b[15:12] == 4'd0) ? 8'd32
                             : (8'd48 + {4'd0, b[15:12]});                   // 十位
                4'd11:   r = 8'd48 + {4'd0, b[11:8]};                        // 个位
                4'd12:   r = 8'd46;                                          // '.'
                4'd13:   r = 8'd48 + {4'd0, b[7:4]};                         // 十分位
                4'd14:   r = 8'd48 + {4'd0, b[3:0]};                         // 百分位
                default: begin                                               // 标签
                    case (line)
                        2'd0: case (pos)
                                  4'd0: r = 8'd89;   // Y
                                  4'd1: r = 8'd65;   // A
                                  4'd2: r = 8'd87;   // W
                                  4'd3: r = 8'd58;   // :
                                  default: r = 8'd32;
                              endcase
                        2'd1: case (pos)
                                  4'd0: r = 8'd80;   // P
                                  4'd1: r = 8'd73;   // I
                                  4'd2: r = 8'd84;   // T
                                  4'd3: r = 8'd67;   // C
                                  4'd4: r = 8'd72;   // H
                                  4'd5: r = 8'd58;   // :
                                  default: r = 8'd32;
                              endcase
                        default: case (pos)
                                  4'd0: r = 8'd82;   // R
                                  4'd1: r = 8'd79;   // O
                                  4'd2: r = 8'd76;   // L
                                  4'd3: r = 8'd76;   // L
                                  4'd4: r = 8'd58;   // :
                                  default: r = 8'd32;
                              endcase
                    endcase
                end
            endcase
            line_char = r;
        end
    endfunction

    //------------------------------------------------------------------
    // 命令表（idx=0 地址，idx=1 控制字节 0x00，idx=2..33 共 32 条命令）
    //------------------------------------------------------------------
    reg [7:0] cmd_byte;

    always @(*) begin
        case (idx)
            11'd0:  cmd_byte = OLED_ADDR;
            11'd1:  cmd_byte = 8'h00;
            11'd2:  cmd_byte = 8'hAE;       // 关显示
            11'd3:  cmd_byte = 8'hD5;       // 时钟分频
            11'd4:  cmd_byte = 8'h80;
            11'd5:  cmd_byte = 8'hA8;       // 复用率 64
            11'd6:  cmd_byte = 8'h3F;
            11'd7:  cmd_byte = 8'hD3;       // 显示偏移 0
            11'd8:  cmd_byte = 8'h00;
            11'd9:  cmd_byte = 8'h40;       // 起始行 0
            11'd10: cmd_byte = 8'h8D;       // 电荷泵开
            11'd11: cmd_byte = 8'h14;
            11'd12: cmd_byte = 8'h20;       // 水平寻址
            11'd13: cmd_byte = 8'h00;
            11'd14: cmd_byte = 8'hA1;       // 段重映射
            11'd15: cmd_byte = 8'hC8;       // COM 扫描方向
            11'd16: cmd_byte = 8'hDA;       // COM 引脚配置
            11'd17: cmd_byte = 8'h12;
            11'd18: cmd_byte = 8'h81;       // 对比度
            11'd19: cmd_byte = 8'hCF;
            11'd20: cmd_byte = 8'hD9;       // 预充电
            11'd21: cmd_byte = 8'hF1;
            11'd22: cmd_byte = 8'hDB;       // VCOMH
            11'd23: cmd_byte = 8'h40;
            11'd24: cmd_byte = 8'hA4;       // 按显存显示
            11'd25: cmd_byte = 8'hA6;       // 正常显示
            11'd26: cmd_byte = 8'h2E;       // 关滚动
            11'd27: cmd_byte = 8'h21;       // 列 0..127
            11'd28: cmd_byte = 8'h00;
            11'd29: cmd_byte = 8'h7F;
            11'd30: cmd_byte = 8'h22;       // 页 0..7
            11'd31: cmd_byte = 8'h00;
            11'd32: cmd_byte = 8'h07;
            11'd33: cmd_byte = 8'hAF;       // 开显示
            default: cmd_byte = 8'h00;
        endcase
    end

    //------------------------------------------------------------------
    // 显存数据（idx=0 地址，idx=1 控制字节 0x40，idx=2..1025 为 1024 像素字节）
    //------------------------------------------------------------------
    wire [10:0] dat_idx = idx - 11'd2;          // 0..1023
    wire [2:0]  page    = dat_idx[9:7];         // 0..7
    wire [6:0]  col     = dat_idx[6:0];         // 0..127
    wire [1:0]  line    = page[2:1];            // 0..2
    wire        half    = page[0];              // 字符上半/下半
    wire [3:0]  cpos    = {1'b0, col[6:3]};     // 字符列 0..15
    wire [2:0]  ccol    = col[2:0];             // 字符内列 0..7

    wire [7:0]  code = (page < 3'd6) ? line_char(line, cpos) : 8'd32;
    wire [7:0]  cofs = code - 8'd32;
    wire [10:0] fidx = {cofs, 4'b0000} + {6'd0, half, ccol};

    wire [7:0]  font_d;

    font_8x16 u_font_8x16 (
        .font_idx  (fidx),
        .font_data (font_d)
    );

    reg [7:0] dat_byte;

    always @(*) begin
        if (idx == 11'd0)
            dat_byte = OLED_ADDR;
        else if (idx == 11'd1)
            dat_byte = 8'h40;
        else
            dat_byte = font_d;
    end

    //------------------------------------------------------------------
    // 输出到 I2C 主机
    //------------------------------------------------------------------
    assign tx_valid = (phase == PH_CMD) || (phase == PH_DAT);
    assign tx_data  = (phase == PH_CMD) ? cmd_byte : dat_byte;
    assign tx_last  = (phase == PH_CMD) ? (idx == 11'd33) : (idx == 11'd1025);
    assign done     = (phase == PH_WAIT);

    //------------------------------------------------------------------
    // 主状态机
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase  <= PH_PWR;
            cnt    <= 32'd0;
            idx    <= 11'd0;
            dyaw   <= 21'd0;
            dpitch <= 21'd0;
            droll  <= 21'd0;
        end else begin
            case (phase)
                PH_PWR: begin
                    if (cnt >= PWR_CYCLES) begin
                        phase <= PH_CMD;
                        idx   <= 11'd0;
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                PH_CMD, PH_DAT: begin
                    if (tx_valid && tx_ready) begin
                        if (tx_last) begin
                            if (phase == PH_CMD) begin
                                phase  <= PH_DAT;
                                idx    <= 11'd0;
                                dyaw   <= byaw;
                                dpitch <= bpitch;
                                droll  <= broll;
                            end else begin
                                phase <= PH_WAIT;
                                cnt   <= 32'd0;
                            end
                        end else begin
                            idx <= idx + 1'b1;
                        end
                    end
                end

                PH_WAIT: begin
                    if (cnt >= REFRESH_CYCLES) begin
                        phase  <= PH_DAT;
                        idx    <= 11'd0;
                        dyaw   <= byaw;
                        dpitch <= bpitch;
                        droll  <= broll;
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                default: phase <= PH_PWR;
            endcase
        end
    end

endmodule
