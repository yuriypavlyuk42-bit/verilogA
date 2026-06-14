`include "constants.h"
`include "discipline.h"

module trimm_cal (vout, bg_in);

    parameter real    temp_test      = 27;    // working/test temperature (deg C)
    parameter real    temp_cal       = 150;   // calibration temperature (deg C)
    parameter real    temp_step      = 10;    // temperature ramp step (deg C)
    parameter integer cal_en         = 1;     // 1 = full trim sequence (TRAN), 0 = bypass, hold code_init (DC)
    parameter real    vddc           = 1.2;
    parameter real    target         = 1.22;  // target bandgap value
    parameter real    Tclk           = 1m;    // settle time per step
    parameter real    start_delay    = 1m;    // initial delay before action
    parameter real    post_cal_delay = 2m;    // delay after calibration before ramping back
    parameter real    tt             = 1u;    // bit transition time
    parameter integer code_init      = 0 from [0:15];

    output [3:0] vout;
    electrical [3:0] vout;

    input bg_in;
    electrical bg_in;

    integer trim_val;     // code used in calibration (cal_en = 1) path
    integer out_code;     // code actually driven to vout
    integer step;
    integer state;
    integer post_cnt;
    real    temp_cur;

    // FSM states (cal_en = 1 path only):
    localparam INIT_HOLD  = 0;
    localparam RAMP_UP    = 1;
    localparam CALIBRATE  = 2;
    localparam POST_DELAY = 3;
    localparam RAMP_DOWN  = 4;
    localparam DONE       = 5;

    (* desc="best trim code" *)               integer best_code;
    (* desc="monotonic normalized code" *)    integer norm_code;
    (* desc="best abs error", units="V" *)    real    best_err;
    (* desc="measured bandgap", units="V" *)  real    v_meas;
    (* desc="final bandgap at temp_test", units="V" *) real v_final;

    analog begin
        @(initial_step) begin
            best_code = code_init;
            norm_code = code_init ^ 8;
            trim_val  = 0;
            step      = 0;
            post_cnt  = 0;
            best_err  = inf;
            v_meas    = 0;
            v_final   = 0;
            temp_cur  = temp_test;
            state     = INIT_HOLD;
            if (cal_en)
                $cds_set_temperature(temp_cur);   // only manage temperature in the trim sequence
        end

        // ===== cal_en = 1 : full trim sequence, runs only in transient =====
        @(timer(start_delay, Tclk)) begin
            if (cal_en) begin
                case (state)

                    INIT_HOLD: begin
                        state = RAMP_UP;
                    end

                    RAMP_UP: begin
                        if (abs(temp_cur - temp_cal) > temp_step) begin
                            if (temp_cal > temp_cur)
                                temp_cur = temp_cur + temp_step;
                            else
                                temp_cur = temp_cur - temp_step;
                            $cds_set_temperature(temp_cur);
                        end else begin
                            temp_cur = temp_cal;
                            $cds_set_temperature(temp_cur);
                            step  = 0;
                            state = CALIBRATE;
                        end
                    end

                    CALIBRATE: begin
                        if (step > 0) begin
                            v_meas = V(bg_in);
                            if (abs(v_meas - target) < best_err) begin
                                best_err  = abs(v_meas - target);
                                best_code = trim_val;
                                norm_code = trim_val ^ 8;
                            end
                        end
                        if (step < 16) begin
                            trim_val = step;
                            step     = step + 1;
                        end else begin
                            trim_val = best_code;
                            post_cnt = 0;
                            state    = POST_DELAY;
                        end
                    end

                    POST_DELAY: begin
                        post_cnt = post_cnt + 1;
                        if (post_cnt * Tclk >= post_cal_delay)
                            state = RAMP_DOWN;
                    end

                    RAMP_DOWN: begin
                        if (abs(temp_cur - temp_test) > temp_step) begin
                            if (temp_test > temp_cur)
                                temp_cur = temp_cur + temp_step;
                            else
                                temp_cur = temp_cur - temp_step;
                            $cds_set_temperature(temp_cur);
                        end else begin
                            temp_cur = temp_test;
                            $cds_set_temperature(temp_cur);
                            state = DONE;
                        end
                    end

                    DONE: begin
                        v_final = V(bg_in);
                    end

                endcase
            end
        end

        // ===== select code driven to output =====
        // cal_en = 0 : bypass everything, hold code_init from t=0 (works in DC)
        // cal_en = 1 : drive the calibration FSM's trim_val
        if (cal_en)
            out_code = trim_val;
        else
            out_code = code_init;

        V(vout[0]) <+ transition(((out_code >> 0) & 1) * vddc, 0, tt);
        V(vout[1]) <+ transition(((out_code >> 1) & 1) * vddc, 0, tt);
        V(vout[2]) <+ transition(((out_code >> 2) & 1) * vddc, 0, tt);
        V(vout[3]) <+ transition(((out_code >> 3) & 1) * vddc, 0, tt);

        @(final_step)
            $strobe("CAL_RESULT best_code=%d norm_code=%d best_err=%g v_final=%g temp_test=%g",
                    best_code, norm_code, best_err, v_final, temp_test);
    end

endmodule
