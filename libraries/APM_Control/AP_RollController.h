#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include "AP_AutoTune.h"
#include <DataFlash/DataFlash.h>
#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter2p.h>

class AP_RollController {
public:
	AP_RollController(AP_AHRS &ahrs, const AP_Vehicle::FixedWing &parms, DataFlash_Class &_dataflash) :
		aparm(parms),
        autotune(gains, AP_AutoTune::AUTOTUNE_ROLL, parms, _dataflash),
        _ahrs(ahrs)
    { 
		AP_Param::setup_object_defaults(this, var_info);
	}

	int32_t get_rate_out(float desired_rate, float scaler);
	int32_t get_servo_out(int32_t angle_err, float scaler, bool disable_integrator);

	void reset_I();

    void autotune_start(void) { autotune.start(); }
    void autotune_restore(void) { autotune.stop(); }

    const       DataFlash_Class::PID_Info& get_pid_info(void) const { return _pid_info; }

	static const struct AP_Param::GroupInfo var_info[];


    // tuning accessors
    void kP(float v) { gains.P.set(v); }
    void kI(float v) { gains.I.set(v); }
    void kD(float v) { gains.D.set(v); }
    void kFF(float v) { gains.FF.set(v); }

    AP_Float &kP(void) { return gains.P; }
    AP_Float &kI(void) { return gains.I; }
    AP_Float &kD(void) { return gains.D; }
    AP_Float &kFF(void) { return gains.FF; }

    void adaptive_tuning_send(mavlink_channel_t chan);
    
private:
	const AP_Vehicle::FixedWing &aparm;
    AP_AutoTune::ATGains gains;
    AP_AutoTune autotune;
	uint32_t _last_t;
	float _last_out;

    DataFlash_Class::PID_Info _pid_info;

	int32_t _get_rate_out(float desired_rate, float scaler, bool disable_integrator);

	AP_AHRS &_ahrs;

    /*
      adaptive control test code. Maths from Ryan Beall
    */
    struct {
        AP_Int8  enable_chan;
        AP_Float alpha;
        AP_Float gamma_theta;
        AP_Float gamma_omega;
        AP_Float gamma_sigma;
        AP_Float theta_max;
	AP_Float theta_min;
        AP_Float theta_epsilon;
        AP_Float omega_max;
	AP_Float omega_min;
        AP_Float omega_epsilon;
        AP_Float sigma_max;
	AP_Float sigma_min;
        AP_Float sigma_epsilon;
        AP_Float w0;
        AP_Float k;
	AP_Float kg;

        uint64_t last_run_us;
	float r;
        float x;
        float x_error;
        float eta;
        float theta;
        float omega;
        float sigma;
        float alpha_filt;
        float u;
        float u_lowpass;
        float x_m;
        float theta_dot;
        float omega_dot;
        float sigma_dot;
        float f;
        float f_dot;

	LowPassFilter2pFloat filter;

    } adap;

    float adaptive_control(float r);
    float projection_operator(float theta, float y, float epsilon, float theta_max, float theta_min);
};
