module i2c_mpu6050_whoami_fix4 #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer I2C_HZ = 10_000
) (
    input            clk,
    input            reset_n,
    input            start,
    input            sda_in,
    output reg       busy,
    output reg       done,
    output reg       ok,
    output reg [7:0] data_out,
    output reg [3:0] dbg_ack,
    output reg [7:0] dbg_state,
    output           scl_drive_low,
    output           sda_drive_low
);
    localparam integer TICK_DIV = CLK_HZ / (I2C_HZ * 4);

    localparam [7:0]
        S_IDLE        = 8'd0,
        S_START_0     = 8'd1,
        S_START_1     = 8'd2,
        S_START_2     = 8'd3,
        S_WBIT_0      = 8'd4,
        S_WBIT_1      = 8'd5,
        S_WBIT_2      = 8'd6,
        S_ACK_0       = 8'd7,
        S_ACK_1       = 8'd8,
        S_ACK_2       = 8'd9,
        S_RS_0        = 8'd10,
        S_RS_1        = 8'd11,
        S_RS_2        = 8'd12,
        S_RS_3        = 8'd13,
        S_RBIT_0      = 8'd14,
        S_RBIT_1      = 8'd15,
        S_RBIT_2      = 8'd16,
        S_MNAK_0      = 8'd17,
        S_MNAK_1      = 8'd18,
        S_MNAK_2      = 8'd19,
        S_STOP_0      = 8'd20,
        S_STOP_1      = 8'd21,
        S_STOP_2      = 8'd22,
        S_DONE        = 8'd23;

    reg [7:0] state;
    reg [15:0] div_cnt;
    reg [7:0] tx_byte;
    reg [7:0] rx_byte;
    reg [2:0] bit_idx;
    reg [1:0] phase;
    reg scl_low_r, sda_low_r;

    wire tick = (div_cnt == TICK_DIV-1);

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= S_IDLE;
            div_cnt <= 0;
            tx_byte <= 0;
            rx_byte <= 0;
            bit_idx <= 0;
            phase <= 0;
            busy <= 0;
            done <= 0;
            ok <= 0;
            data_out <= 0;
            dbg_ack <= 0;
            dbg_state <= 0;
            scl_low_r <= 0;
            sda_low_r <= 0;
        end else begin
            done <= 0;
            dbg_state <= state;
            if (tick) div_cnt <= 0; else div_cnt <= div_cnt + 1'b1;
            if (tick) begin
                case (state)
                    S_IDLE: begin
                        busy <= 0;
                        scl_low_r <= 0;
                        sda_low_r <= 0;
                        if (start) begin
                            busy <= 1;
                            ok <= 1;
                            dbg_ack <= 0;
                            phase <= 0;
                            tx_byte <= 8'hD0;
                            bit_idx <= 3'd7;
                            state <= S_START_0;
                        end
                    end
                    // START
                    S_START_0: begin scl_low_r <= 0; sda_low_r <= 0; state <= S_START_1; end
                    S_START_1: begin scl_low_r <= 0; sda_low_r <= 1; state <= S_START_2; end
                    S_START_2: begin scl_low_r <= 1; sda_low_r <= 1; state <= S_WBIT_0;  end

                    // WRITE BIT
                    S_WBIT_0: begin scl_low_r <= 1; sda_low_r <= ~tx_byte[bit_idx]; state <= S_WBIT_1; end
                    S_WBIT_1: begin scl_low_r <= 0; sda_low_r <= ~tx_byte[bit_idx]; state <= S_WBIT_2; end
                    S_WBIT_2: begin
                        scl_low_r <= 1; sda_low_r <= ~tx_byte[bit_idx];
                        if (bit_idx == 0) state <= S_ACK_0;
                        else begin bit_idx <= bit_idx - 1'b1; state <= S_WBIT_0; end
                    end

                    // SLAVE ACK SAMPLE
                    S_ACK_0: begin scl_low_r <= 1; sda_low_r <= 0; state <= S_ACK_1; end
                    S_ACK_1: begin
                        scl_low_r <= 0; sda_low_r <= 0;
                        if (sda_in) ok <= 0;
                        else begin
                            case (phase)
                                2'd0: dbg_ack[0] <= 1'b1;
                                2'd1: dbg_ack[1] <= 1'b1;
                                2'd2: dbg_ack[2] <= 1'b1;
                            endcase
                        end
                        state <= S_ACK_2;
                    end
                    S_ACK_2: begin
                        scl_low_r <= 1; sda_low_r <= 0;
                        case (phase)
                            2'd0: begin phase <= 2'd1; tx_byte <= 8'h75; bit_idx <= 3'd7; state <= S_WBIT_0; end
                            2'd1: begin phase <= 2'd2; tx_byte <= 8'hD1; bit_idx <= 3'd7; state <= S_RS_0;   end
                            2'd2: begin rx_byte <= 8'h00; bit_idx <= 3'd7; state <= S_RBIT_0; end
                            default: state <= S_STOP_0;
                        endcase
                    end

                    // REPEATED START
                    S_RS_0: begin scl_low_r <= 1; sda_low_r <= 0; state <= S_RS_1; end
                    S_RS_1: begin scl_low_r <= 0; sda_low_r <= 0; state <= S_RS_2; end
                    S_RS_2: begin scl_low_r <= 0; sda_low_r <= 1; state <= S_RS_3; end
                    S_RS_3: begin scl_low_r <= 1; sda_low_r <= 1; state <= S_WBIT_0; end

                    // READ BIT: sample on the FIRST high state, not on a delayed second-high state
                    S_RBIT_0: begin scl_low_r <= 1; sda_low_r <= 0; state <= S_RBIT_1; end
                    S_RBIT_1: begin
                        scl_low_r <= 0; sda_low_r <= 0;
                        rx_byte[bit_idx] <= sda_in;
                        state <= S_RBIT_2;
                    end
                    S_RBIT_2: begin
                        scl_low_r <= 1; sda_low_r <= 0;
                        if (bit_idx == 0) begin
                            data_out <= rx_byte;
                            data_out[0] <= sda_in;
                            dbg_ack[3] <= 1'b1;
                            state <= S_MNAK_0;
                        end else begin
                            bit_idx <= bit_idx - 1'b1;
                            state <= S_RBIT_0;
                        end
                    end

                    // MASTER NACK
                    S_MNAK_0: begin scl_low_r <= 1; sda_low_r <= 0; state <= S_MNAK_1; end
                    S_MNAK_1: begin scl_low_r <= 0; sda_low_r <= 0; state <= S_MNAK_2; end
                    S_MNAK_2: begin scl_low_r <= 1; sda_low_r <= 0; state <= S_STOP_0; end

                    // STOP
                    S_STOP_0: begin scl_low_r <= 1; sda_low_r <= 1; state <= S_STOP_1; end
                    S_STOP_1: begin scl_low_r <= 0; sda_low_r <= 1; state <= S_STOP_2; end
                    S_STOP_2: begin scl_low_r <= 0; sda_low_r <= 0; state <= S_DONE; end

                    S_DONE: begin busy <= 0; done <= 1; scl_low_r <= 0; sda_low_r <= 0; state <= S_IDLE; end
                    default: begin busy <= 0; scl_low_r <= 0; sda_low_r <= 0; state <= S_IDLE; end
                endcase
            end
        end
    end

    assign scl_drive_low = scl_low_r;
    assign sda_drive_low = sda_low_r;
endmodule

module I2C_PLZ(
    input           CLOCK_50,
    input   [3:0]   KEY,
    input   [9:0]   SW,
    output  [9:0]   LEDR,
    output  [6:0]   HEX0,
    output  [6:0]   HEX1,
    output  [6:0]   HEX2,
    output  [6:0]   HEX3,
    output  [6:0]   HEX4,
    output  [6:0]   HEX5,
    inout   [35:0]  GPIO
);
    wire reset_n = KEY[0];
    wire sda_in = GPIO[0];
    wire scl_in = GPIO[1];
    wire scl_drive_low, sda_drive_low;
    wire busy_tx, done_tx, ok_tx;
    wire [7:0] whoami;
    wire [3:0] dbg_ack;
    wire [7:0] dbg_state;

    assign GPIO[0] = sda_drive_low ? 1'b0 : 1'bz;
    assign GPIO[1] = scl_drive_low ? 1'b0 : 1'bz;

    // diagnostics for analyzer
    assign GPIO[2] = sda_in;          // actual SDA level seen by FPGA
    assign GPIO[3] = scl_in;          // actual SCL level seen by FPGA
    assign GPIO[4] = sda_drive_low;   // FPGA intent: drive SDA low
    assign GPIO[5] = scl_drive_low;   // FPGA intent: drive SCL low
    assign GPIO[6] = busy_tx;
    assign GPIO[7] = ok_tx;
    assign GPIO[15:8] = dbg_state;
    genvar i;
    generate for (i=16;i<36;i=i+1) begin:Z assign GPIO[i]=1'bz; end endgenerate

    reg [22:0] pwr_cnt;
    wire pwr_done = (pwr_cnt == 23'd5_000_000);
    always @(posedge CLOCK_50 or negedge reset_n) begin
        if (!reset_n) pwr_cnt <= 0;
        else if (!pwr_done) pwr_cnt <= pwr_cnt + 1'b1;
    end

    reg [24:0] gap_cnt;
    wire gap_done = (gap_cnt == 25'd20_000_000);
    reg start_tx;
    reg seen, good;
    reg [7:0] whoami_latched;

    i2c_mpu6050_whoami_fix4 u0(
        .clk(CLOCK_50), .reset_n(reset_n), .start(start_tx), .sda_in(sda_in),
        .busy(busy_tx), .done(done_tx), .ok(ok_tx), .data_out(whoami), .dbg_ack(dbg_ack), .dbg_state(dbg_state),
        .scl_drive_low(scl_drive_low), .sda_drive_low(sda_drive_low)
    );

    reg [1:0] ctrl_state;
    localparam CW=2'd0, CS=2'd1, CT=2'd2, CG=2'd3;
    always @(posedge CLOCK_50 or negedge reset_n) begin
        if (!reset_n) begin
            ctrl_state <= CW; start_tx <= 0; gap_cnt <= 0; seen <= 0; good <= 0; whoami_latched <= 0;
        end else begin
            case (ctrl_state)
                CW: begin start_tx <= 0; gap_cnt <= 0; if (pwr_done) ctrl_state <= CS; end
                CS: begin start_tx <= 1; if (busy_tx) ctrl_state <= CT; end
                CT: begin start_tx <= 0; if (done_tx) begin seen <= 1; good <= ok_tx; whoami_latched <= whoami; gap_cnt <= 0; ctrl_state <= CG; end end
                CG: begin start_tx <= 0; if (!gap_done) gap_cnt <= gap_cnt + 1'b1; else ctrl_state <= CS; end
            endcase
        end
    end

    assign LEDR[7:0] = whoami_latched;
    assign LEDR[8] = good;
    assign LEDR[9] = seen;
    assign HEX0 = 7'h7F; assign HEX1 = 7'h7F; assign HEX2 = 7'h7F;
    assign HEX3 = 7'h7F; assign HEX4 = 7'h7F; assign HEX5 = 7'h7F;
endmodule
