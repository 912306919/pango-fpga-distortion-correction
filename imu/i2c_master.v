`timescale 1ns / 1ps
//=============================================================================
// i2c_master.v
// 字节流 I2C 主机（只写，开漏输出，支持从机 ACK 采样）
//
// 用法：
//   外部把待发送字节放在 tx_data 上并保持 tx_valid；tx_ready 为单周期脉冲，
//   为高表示该字节已被锁存，可切换为下一个字节。tx_last 为 1 表示该字节
//   发完后产生 STOP。
//   主机在 S_IDLE / S_LOAD 检测到 tx_valid 时自动产生 START 或继续发送，
//   不需要额外 start 信号。
//
// SCL_FREQ = 100kHz（提升到 400kHz 只需改参数，需确认 OLED 模块与上拉电阻支持）
//=============================================================================
module i2c_master #(
    parameter integer CLK_FREQ = 27000000,
    parameter integer SCL_FREQ = 100000
)(
    input  wire       clk,
    input  wire       rst_n,

    input  wire       tx_valid,
    input  wire [7:0] tx_data,
    input  wire       tx_last,
    output reg        tx_ready,

    output reg        busy,
    output reg        ack_err,

    inout  wire       scl,
    inout  wire       sda
);

    // 半个 SCL 周期的时钟数
    localparam integer QN = CLK_FREQ / (SCL_FREQ * 2);

    //------------------------------------------------------------------
    // 状态机
    //------------------------------------------------------------------
    localparam [3:0] S_IDLE   = 4'd0,
                     S_START  = 4'd1,
                     S_SEND   = 4'd2,
                     S_SCLH   = 4'd3,
                     S_SAMPLE = 4'd4,
                     S_ACKREL = 4'd5,
                     S_ACKH   = 4'd6,
                     S_ACKS   = 4'd7,
                     S_LOAD   = 4'd8,
                     S_STOPA  = 4'd9,
                     S_STOPB  = 4'd10,
                     S_STOPC  = 4'd11;

    reg [3:0]  state;
    reg [15:0] wait_cnt;
    reg [7:0]  byte;
    reg [2:0]  bit_idx;
    reg        last_flag;

    // 开漏驱动：1 = 拉低，0 = 释放（由上拉电阻拉高）
    reg scl_l;
    reg sda_l;

    assign scl = scl_l ? 1'b0 : 1'bz;
    assign sda = sda_l ? 1'b0 : 1'bz;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            wait_cnt  <= 16'd0;
            byte      <= 8'd0;
            bit_idx   <= 3'd0;
            last_flag <= 1'b0;
            scl_l     <= 1'b0;
            sda_l     <= 1'b0;
            tx_ready  <= 1'b0;
            busy      <= 1'b0;
            ack_err   <= 1'b0;
        end else begin
            // tx_ready 是单周期脉冲：只在真正锁存一个字节的那一拍为高，
            // 上游据此把索引加一（电平式 ready 会让上游连加多次）。
            tx_ready <= 1'b0;

            if (wait_cnt != 16'd0) begin
                wait_cnt <= wait_cnt - 1'b1;
            end else begin
                case (state)
                    // 空闲：等待第一个字节，SDA 拉低（SCL 此时为高）产生 START
                    S_IDLE: begin
                        scl_l    <= 1'b0;
                        sda_l    <= 1'b0;
                        busy     <= 1'b0;
                        if (tx_valid) begin
                            byte      <= tx_data;
                            last_flag <= tx_last;
                            busy      <= 1'b1;
                            tx_ready  <= 1'b1;
                            sda_l     <= 1'b1;
                            state     <= S_START;
                            wait_cnt  <= QN - 1;
                        end
                    end

                    // START 建立后把 SCL 拉低
                    S_START: begin
                        scl_l    <= 1'b1;
                        state    <= S_SEND;
                        wait_cnt <= QN - 1;
                    end

                    // 输出最高位（SCL 此时为低，数据在 SCL 上升前已稳定）
                    S_SEND: begin
                        sda_l    <= ~byte[7];
                        bit_idx  <= 3'd7;
                        state    <= S_SCLH;
                        wait_cnt <= QN - 1;
                    end

                    // 释放 SCL（上升沿，从机在此采样）
                    S_SCLH: begin
                        scl_l    <= 1'b0;
                        state    <= S_SAMPLE;
                        wait_cnt <= QN - 1;
                    end

                    // 拉低 SCL，准备下一位
                    S_SAMPLE: begin
                        scl_l <= 1'b1;
                        if (bit_idx == 3'd0) begin
                            state    <= S_ACKREL;
                        end else begin
                            bit_idx  <= bit_idx - 1'b1;
                            sda_l    <= ~byte[bit_idx - 1'b1];
                            state    <= S_SCLH;
                        end
                        wait_cnt <= QN - 1;
                    end

                    // 释放 SDA，准备接收 ACK
                    S_ACKREL: begin
                        sda_l    <= 1'b0;
                        state    <= S_ACKH;
                        wait_cnt <= QN - 1;
                    end

                    S_ACKH: begin
                        scl_l    <= 1'b0;
                        state    <= S_ACKS;
                        wait_cnt <= QN - 1;
                    end

                    // 第 9 个时钟：采样 ACK，然后决定继续下一个字节还是 STOP
                    S_ACKS: begin
                        ack_err <= sda;
                        scl_l   <= 1'b1;
                        if (last_flag) begin
                            state <= S_STOPA;
                        end else begin
                            state <= S_LOAD;
                        end
                        wait_cnt <= QN - 1;
                    end

                    // 字节间保持 SCL 低，等待下一个字节
                    S_LOAD: begin
                        scl_l <= 1'b1;
                        if (tx_valid) begin
                            byte      <= tx_data;
                            last_flag <= tx_last;
                            sda_l     <= ~tx_data[7];
                            bit_idx   <= 3'd7;
                            tx_ready  <= 1'b1;      // 本拍已锁存，通知上游
                            state     <= S_SCLH;
                            wait_cnt  <= QN - 1;
                        end
                    end

                    // STOP：SCL 低时 SDA 拉低 -> SCL 高 -> SDA 释放
                    S_STOPA: begin
                        sda_l    <= 1'b1;
                        state    <= S_STOPB;
                        wait_cnt <= QN - 1;
                    end

                    S_STOPB: begin
                        scl_l    <= 1'b0;
                        state    <= S_STOPC;
                        wait_cnt <= QN - 1;
                    end

                    S_STOPC: begin
                        sda_l    <= 1'b0;
                        state    <= S_IDLE;
                        wait_cnt <= QN - 1;
                    end

                    default: state <= S_IDLE;
                endcase
            end
        end
    end

endmodule
