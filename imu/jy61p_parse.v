`timescale 1ns / 1ps
//=============================================================================
// jy61p_parse.v
// JY61P（维特智能 6 轴 IMU）串口协议解析
//
// 帧格式（8N1）：共 11 字节
//   byte0  : 0x55            帧头
//   byte1  : 帧类型  0x51 加速度 / 0x52 角速度 / 0x53 角度
//   byte2~9: 4 个 int16（小端：低字节在前，高字节在后）
//   byte10 : 校验和 = byte0~byte9 之和（取低 8 位）
//
// 数据换算（供后续定点运算参考，本模块输出原始有符号数值）：
//   加速度 : ±16g      -> 数值 / 32768 * 16   (g)
//   角速度 : ±2000°/s  -> 数值 / 32768 * 2000 (°/s)
//   角度   : ±180°     -> 数值 / 32768 * 180  (°)
//   温度   : 数值 / 100 (°C)
//
// 各 *valid 为单时钟周期脉冲，仅在对应类型帧校验通过时拉高。
//=============================================================================
module jy61p_parse (
    input  wire               clk,
    input  wire               rst_n,
    input  wire        [7:0]  byte_in,
    input  wire               byte_valid,

    output reg  signed [15:0] acc_x,
    output reg  signed [15:0] acc_y,
    output reg  signed [15:0] acc_z,
    output reg  signed [15:0] gyro_x,
    output reg  signed [15:0] gyro_y,
    output reg  signed [15:0] gyro_z,
    output reg  signed [15:0] roll,
    output reg  signed [15:0] pitch,
    output reg  signed [15:0] yaw,
    output reg         [15:0] temperature,

    output reg                acc_valid,
    output reg                gyro_valid,
    output reg                angle_valid,
    output reg                frame_valid,     // 任意类型校验通过
    output reg                checksum_err     // 校验失败（调试用）
);

    localparam [7:0] FRAME_HEAD = 8'h55;
    localparam [7:0] TYPE_ACC   = 8'h51;
    localparam [7:0] TYPE_GYRO  = 8'h52;
    localparam [7:0] TYPE_ANGLE = 8'h53;

    reg [3:0] cnt;                 // 0:等帧头 1~9:数据 10:校验
    reg [7:0] sum;                 // 累加校验
    reg [7:0] frm_buf [0:9];       // 帧缓存（frm_buf[1]=类型，frm_buf[2..9]=数据）

    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt          <= 4'd0;
            sum          <= 8'd0;
            acc_x        <= 16'sd0;
            acc_y        <= 16'sd0;
            acc_z        <= 16'sd0;
            gyro_x       <= 16'sd0;
            gyro_y       <= 16'sd0;
            gyro_z       <= 16'sd0;
            roll         <= 16'sd0;
            pitch        <= 16'sd0;
            yaw          <= 16'sd0;
            temperature  <= 16'd0;
            acc_valid    <= 1'b0;
            gyro_valid   <= 1'b0;
            angle_valid  <= 1'b0;
            frame_valid  <= 1'b0;
            checksum_err <= 1'b0;
            for (i = 0; i < 10; i = i + 1)
                frm_buf[i] <= 8'd0;
        end else begin
            acc_valid    <= 1'b0;
            gyro_valid   <= 1'b0;
            angle_valid  <= 1'b0;
            frame_valid  <= 1'b0;
            checksum_err <= 1'b0;

            if (byte_valid) begin
                case (cnt)
                    4'd0: begin
                        if (byte_in == FRAME_HEAD) begin
                            cnt <= 4'd1;
                            sum <= FRAME_HEAD;
                        end
                    end

                    // byte1~byte9：类型 + 8 字节数据
                    4'd1, 4'd2, 4'd3, 4'd4, 4'd5,
                    4'd6, 4'd7, 4'd8, 4'd9: begin
                        if (byte_in == FRAME_HEAD) begin
                            // 中途又出现帧头，说明前面的帧断了，重新同步
                            cnt <= 4'd1;
                            sum <= FRAME_HEAD;
                        end else begin
                            frm_buf[cnt] <= byte_in;
                            sum      <= sum + byte_in;
                            cnt      <= cnt + 1'b1;
                        end
                    end

                    // byte10：校验和
                    4'd10: begin
                        cnt <= 4'd0;
                        if (sum == byte_in) begin
                            frame_valid <= 1'b1;
                            case (frm_buf[1])
                                TYPE_ACC: begin
                                    acc_x       <= {frm_buf[3], frm_buf[2]};
                                    acc_y       <= {frm_buf[5], frm_buf[4]};
                                    acc_z       <= {frm_buf[7], frm_buf[6]};
                                    temperature <= {frm_buf[9], frm_buf[8]};
                                    acc_valid   <= 1'b1;
                                end
                                TYPE_GYRO: begin
                                    gyro_x    <= {frm_buf[3], frm_buf[2]};
                                    gyro_y    <= {frm_buf[5], frm_buf[4]};
                                    gyro_z    <= {frm_buf[7], frm_buf[6]};
                                    gyro_valid <= 1'b1;
                                end
                                TYPE_ANGLE: begin
                                    roll       <= {frm_buf[3], frm_buf[2]};
                                    pitch      <= {frm_buf[5], frm_buf[4]};
                                    yaw        <= {frm_buf[7], frm_buf[6]};
                                    angle_valid <= 1'b1;
                                end
                                default: ;   // 0x54 四元数等其他帧暂不处理
                            endcase
                        end else begin
                            checksum_err <= 1'b1;
                        end
                    end

                    default: cnt <= 4'd0;
                endcase
            end
        end
    end

endmodule

