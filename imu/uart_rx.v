`timescale 1ns / 1ps
//=============================================================================
// uart_rx.v
// 通用异步串口接收（16 倍过采样，起始位中点校验，停止位采样）
// 用途：接收 JY61P 陀螺仪的串口输出（8N1，TTL 电平）
//
// 参数：
//   CLK_FREQ  : 输入时钟频率，盘古 100Pro+ 板载 27MHz 单端晶振
//   BAUD_RATE : JY61P 出厂默认 115200，若模块被改成 9600 则改这里
//
// 说明：
//   分频采用相位累加，平均采样率严格等于 BAUD_RATE*16，
//   不会因为 27MHz/115200 = 234.375 不是整数而产生累计误差。
//=============================================================================
module uart_rx #(
    parameter integer CLK_FREQ  = 27000000,
    parameter integer BAUD_RATE = 115200
)(
    input  wire       clk,
    input  wire       rst_n,        // 低有效复位
    input  wire       rx,           // 串口输入（空闲为高）
    output reg  [7:0] rx_data,      // 收到的 1 字节
    output reg        rx_valid,     // 1 个时钟周期的有效脉冲
    output reg        frame_err     // 停止位不是高电平（帧错误）
);

    //------------------------------------------------------------------
    // 16 倍波特率采样脉冲：acc 每个时钟加 BAUD_RATE*16
    // acc 溢出 CLK_FREQ 时产生一个 tick，tick 频率 = 16 * BAUD_RATE
    //------------------------------------------------------------------
    localparam integer STEP = BAUD_RATE * 16;

    reg [31:0] baud_acc;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            baud_acc <= 32'd0;
        else if (baud_acc + STEP >= CLK_FREQ)
            baud_acc <= baud_acc + STEP - CLK_FREQ;
        else
            baud_acc <= baud_acc + STEP;
    end

    wire baud_tick = (baud_acc + STEP >= CLK_FREQ);

    //------------------------------------------------------------------
    // 输入同步 + 下降沿检测
    //------------------------------------------------------------------
    reg rx_meta, rx_sync, rx_last;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_meta <= 1'b1;
            rx_sync <= 1'b1;
            rx_last <= 1'b1;
        end else begin
            rx_meta <= rx;
            rx_sync <= rx_meta;
            rx_last <= rx_sync;
        end
    end

    wire rx_fall = rx_last & ~rx_sync;

    //------------------------------------------------------------------
    // 接收状态机
    //------------------------------------------------------------------
    localparam [1:0] S_IDLE  = 2'd0,
                     S_START = 2'd1,
                     S_DATA  = 2'd2,
                     S_STOP  = 2'd3;

    reg [1:0] state;
    reg [3:0] os;        // 0..15 过采样计数，7 = 位中点
    reg [2:0] bit_idx;
    reg [7:0] shift;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            os        <= 4'd0;
            bit_idx   <= 3'd0;
            shift     <= 8'd0;
            rx_data   <= 8'd0;
            rx_valid  <= 1'b0;
            frame_err <= 1'b0;
        end else begin
            rx_valid  <= 1'b0;
            frame_err <= 1'b0;

            // 空闲时检测起始位下降沿，立即对齐到起始位起点
            if (state == S_IDLE && rx_fall) begin
                state <= S_START;
                os    <= 4'd0;
            end

            if (baud_tick) begin
                case (state)
                    S_IDLE: ;

                    S_START: begin
                        if (os == 4'd15) begin          // 起始位结束，进入数据位
                            state   <= S_DATA;
                            bit_idx <= 3'd0;
                            os      <= 4'd0;
                        end else if ((os == 4'd7) && rx_sync) begin
                            state <= S_IDLE;             // 不是起始位（毛刺）
                        end else begin
                            os <= os + 1'b1;
                        end
                    end

                    S_DATA: begin
                        if (os == 4'd7)                // 每一位的中点采样（低位先收）
                            shift <= {rx_sync, shift[7:1]};

                        if (os == 4'd15) begin
                            os <= 4'd0;
                            if (bit_idx == 3'd7)
                                state <= S_STOP;
                            else
                                bit_idx <= bit_idx + 1'b1;
                        end else begin
                            os <= os + 1'b1;
                        end
                    end

                    S_STOP: begin
                        if (os == 4'd7) begin
                            os        <= 4'd0;
                            state     <= S_IDLE;
                            rx_data   <= shift;
                            rx_valid  <= 1'b1;
                            frame_err <= ~rx_sync;
                        end else begin
                            os <= os + 1'b1;
                        end
                    end

                    default: state <= S_IDLE;
                endcase
            end
        end
    end

endmodule
