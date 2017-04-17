/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//	Code by Jon Challinger
//  Modified by Paul Riseborough
//  Adaptive Control by Ryan Beall

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_RollController.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_RollController::var_info[] = {
	// @Param: TCONST
	// @DisplayName: Roll Time Constant
	// @Description: This controls the time constant in seconds from demanded to achieved bank angle. A value of 0.5 is a good default and will work with nearly all models. Advanced users may want to reduce this time to obtain a faster response but there is no point setting a time less than the aircraft can achieve.
	// @Range: 0.4 1.0
	// @Units: seconds
	// @Increment: 0.1
	// @User: Advanced
	AP_GROUPINFO("TCONST",      0, AP_RollController, gains.tau,       0.5f),

	// @Param: P
	// @DisplayName: Proportional Gain
	// @Description: This is the gain from bank angle error to aileron.
	// @Range: 0.1 4.0
	// @Increment: 0.1
	// @User: User
	AP_GROUPINFO("P",        1, AP_RollController, gains.P,        0.6f),

	// @Param: D
	// @DisplayName: Damping Gain
	// @Description: This is the gain from roll rate to aileron. This adjusts the damping of the roll control loop. It has the same effect as RLL2SRV_D in the old PID controller but without the spikes in servo demands. This gain helps to reduce rolling in turbulence. It should be increased in 0.01 increments as too high a value can lead to a high frequency roll oscillation that could overstress the airframe.
	// @Range: 0 0.1
	// @Increment: 0.01
	// @User: User
	AP_GROUPINFO("D",        2, AP_RollController, gains.D,        0.02f),

	// @Param: I
	// @DisplayName: Integrator Gain
	// @Description: This is the gain from the integral of bank angle to aileron. It has the same effect as RLL2SRV_I in the old PID controller. Increasing this gain causes the controller to trim out steady offsets due to an out of trim aircraft.
	// @Range: 0 1.0
	// @Increment: 0.05
	// @User: User
	AP_GROUPINFO("I",        3, AP_RollController, gains.I,        0.1f),

	// @Param: RMAX
	// @DisplayName: Maximum Roll Rate
	// @Description: This sets the maximum roll rate that the controller will demand (degrees/sec). Setting it to zero disables the limit. If this value is set too low, then the roll can't keep up with the navigation demands and the plane will start weaving. If it is set too high (or disabled by setting to zero) then ailerons will get large inputs at the start of turns. A limit of 60 degrees/sec is a good default.
	// @Range: 0 180
	// @Units: degrees/second
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("RMAX",   4, AP_RollController, gains.rmax,       0),

	// @Param: IMAX
	// @DisplayName: Integrator limit
	// @Description: This limits the number of degrees of aileron in centi-degrees over which the integrator will operate. At the default setting of 3000 centi-degrees, the integrator will be limited to +- 30 degrees of servo travel. The maximum servo deflection is +- 45 centi-degrees, so the default value represents a 2/3rd of the total control throw which is adequate unless the aircraft is severely out of trim.
	// @Range: 0 4500
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("IMAX",      5, AP_RollController, gains.imax,        3000),

	// @Param: FF
	// @DisplayName: Feed forward Gain
	// @Description: This is the gain from demanded rate to aileron output. 
	// @Range: 0.1 4.0
	// @Increment: 0.1
	// @User: User
	AP_GROUPINFO("FF",        6, AP_RollController, gains.FF,          0.0f),

	// adaptive control parameters
	AP_GROUPINFO_FLAGS("AD_CH", 9, AP_RollController, adap.enable_chan, 0, AP_PARAM_FLAG_ENABLE),
	AP_GROUPINFO("ALPHA", 10, AP_RollController, adap.alpha, 5),
	AP_GROUPINFO("GAMMAT", 11, AP_RollController, adap.gamma_theta, 10000),
        AP_GROUPINFO("GAMMAW", 12, AP_RollController, adap.gamma_omega, 10000),
        AP_GROUPINFO("GAMMAS", 13, AP_RollController, adap.gamma_sigma, 10000),
	AP_GROUPINFO("THETAU", 14, AP_RollController, adap.theta_max, 5.0),
	AP_GROUPINFO("THETAL", 15, AP_RollController, adap.theta_min, 0.4),
	AP_GROUPINFO("THETAE", 16, AP_RollController, adap.theta_epsilon, 5925),
        AP_GROUPINFO("OMEGAU", 17, AP_RollController, adap.omega_max, 5.0),
	AP_GROUPINFO("OMEGAL", 18, AP_RollController, adap.omega_min, 0.03),
	AP_GROUPINFO("OMEGAE", 19, AP_RollController, adap.omega_epsilon, 5925),
        AP_GROUPINFO("SIGMAU", 20, AP_RollController, adap.sigma_max, 0.01),
	AP_GROUPINFO("SIGMAL", 21, AP_RollController, adap.sigma_min, -0.01),
	AP_GROUPINFO("SIGMAE", 22, AP_RollController, adap.sigma_epsilon, 203),
	AP_GROUPINFO("W0", 23, AP_RollController, adap.w0, 25),
        AP_GROUPINFO("K",24, AP_RollController, adap.k, 0.3),
    
	AP_GROUPEND
};


/*
  internal rate controller, called by attitude and rate controller
  public functions
*/
int32_t AP_RollController::_get_rate_out(float desired_rate, float scaler, bool disable_integrator)
{
	uint32_t tnow = AP_HAL::millis();
	uint32_t dt = tnow - _last_t;
	if (_last_t == 0 || dt > 1000) {
		dt = 0;
	}
	_last_t = tnow;
	
	// Calculate equivalent gains so that values for K_P and K_I can be taken across from the old PID law
    // No conversion is required for K_D
	float ki_rate = gains.I * gains.tau;
    float eas2tas = _ahrs.get_EAS2TAS();
	float kp_ff = MAX((gains.P - gains.I * gains.tau) * gains.tau  - gains.D , 0) / eas2tas;
    float k_ff = gains.FF / eas2tas;
	float delta_time    = (float)dt * 0.001f;
	
	// Limit the demanded roll rate
	if (gains.rmax && desired_rate < -gains.rmax) {
        desired_rate = - gains.rmax;
    } else if (gains.rmax && desired_rate > gains.rmax) {
        desired_rate = gains.rmax;
    }
	
	if (adap.enable_chan > 0 && hal.rcin->read(adap.enable_chan-1) >= 1700) {
	  // the user has enabled adaptive control test code
	  return adaptive_control(desired_rate);
	}

    // Get body rate vector (radians/sec)
	float omega_x = _ahrs.get_gyro().x;
	
	// Calculate the roll rate error (deg/sec) and apply gain scaler
    float achieved_rate = ToDeg(omega_x);
	float rate_error = (desired_rate - achieved_rate) * scaler;
	
	// Get an airspeed estimate - default to zero if none available
	float aspeed;
	if (!_ahrs.airspeed_estimate(&aspeed)) {
        aspeed = 0.0f;
    }

	// Multiply roll rate error by _ki_rate, apply scaler and integrate
	// Scaler is applied before integrator so that integrator state relates directly to aileron deflection
	// This means aileron trim offset doesn't change as the value of scaler changes with airspeed
	// Don't integrate if in stabilise mode as the integrator will wind up against the pilots inputs
	if (!disable_integrator && ki_rate > 0) {
		//only integrate if gain and time step are positive and airspeed above min value.
		if (dt > 0 && aspeed > float(aparm.airspeed_min)) {
		    float integrator_delta = rate_error * ki_rate * delta_time * scaler;
			// prevent the integrator from increasing if surface defln demand is above the upper limit
			if (_last_out < -45) {
                integrator_delta = MAX(integrator_delta , 0);
            } else if (_last_out > 45) {
                // prevent the integrator from decreasing if surface defln demand  is below the lower limit
                 integrator_delta = MIN(integrator_delta, 0);
            }
			_pid_info.I += integrator_delta;
		}
	} else {
		_pid_info.I = 0;
	}
	
    // Scale the integration limit
    float intLimScaled = gains.imax * 0.01f;

    // Constrain the integrator state
    _pid_info.I = constrain_float(_pid_info.I, -intLimScaled, intLimScaled);
	
	// Calculate the demanded control surface deflection
	// Note the scaler is applied again. We want a 1/speed scaler applied to the feed-forward
	// path, but want a 1/speed^2 scaler applied to the rate error path. 
	// This is because acceleration scales with speed^2, but rate scales with speed.
    _pid_info.D = rate_error * gains.D * scaler;
    _pid_info.P = desired_rate * kp_ff * scaler;
    _pid_info.FF = desired_rate * k_ff * scaler;
    _pid_info.desired = desired_rate;

	_last_out = _pid_info.FF + _pid_info.P + _pid_info.D;

    if (autotune.running && aspeed > aparm.airspeed_min) {
        // let autotune have a go at the values 
        // Note that we don't pass the integrator component so we get
        // a better idea of how much the base PD controller
        // contributed
        autotune.update(desired_rate, achieved_rate, _last_out);
    }

	_last_out += _pid_info.I;
	
	// Convert to centi-degrees and constrain
	return constrain_float(_last_out * 100, -4500, 4500);
}


/*
 Function returns an equivalent elevator deflection in centi-degrees in the range from -4500 to 4500
 A positive demand is up
 Inputs are: 
 1) desired roll rate in degrees/sec
 2) control gain scaler = scaling_speed / aspeed
*/
int32_t AP_RollController::get_rate_out(float desired_rate, float scaler)
{
    return _get_rate_out(desired_rate, scaler, false);
}

/*
 Function returns an equivalent elevator deflection in centi-degrees in the range from -4500 to 4500
 A positive demand is up
 Inputs are: 
 1) demanded bank angle in centi-degrees
 2) control gain scaler = scaling_speed / aspeed
 3) boolean which is true when stabilise mode is active
 4) minimum FBW airspeed (metres/sec)
*/
int32_t AP_RollController::get_servo_out(int32_t angle_err, float scaler, bool disable_integrator)
{
    if (gains.tau < 0.1f) {
        gains.tau.set(0.1f);
    }
	
	// Calculate the desired roll rate (deg/sec) from the angle error
	float desired_rate = angle_err * 0.01f / gains.tau;

    return _get_rate_out(desired_rate, scaler, disable_integrator);
}

void AP_RollController::reset_I()
{
	_pid_info.I = 0;
}


/*
  adaptive control test code. Maths thanks to Ryan Beall
*/
float AP_RollController::adaptive_control(float r)
{
    // r = command
    // x = actual state value from sensor (i.e. pitch rate from gyro)
    // x_m = estimated model output (i.e. estimated pitch rate given state predictor estimates)
    // u = commanded output (i.e. delta elevator) (radians)
    // u_lowpass = low passed commanded output within the bandwith of the actuator (radians)
    // theta = estimated state from state predictor (unitless)
    // omega = estimated state from state predictor (unitless)
    // sigma = estimated state from state predictor (unitless)
    // theta_upper_limit = constraint on states to ensure robustness (unitless)
    // theta_lower_limit = constraint on states to ensure robustness (unitless)
    // omega_upper_limit = constraint on states to ensure robustness (unitless)
    // omega_lower_limit = constraint on states to ensure robustness (unitless)
    // epsilon = slope of projection operator (needs to be commensurate with gain and function)

    float dt;
 
    float x =_ahrs.get_gyro().x; //radians
    adap.r = radians(r); //convert desired input into radians if required

    //reset reference model at initialization
    uint64_t now = AP_HAL::micros64();
    if (adap.last_run_us == 0 || now - adap.last_run_us > 200000UL) {
        // reset after not running for 0.2s
        adap.x_m = x;       
        adap.u = 0.0;
        adap.u_lowpass = 0.0;
        adap.theta = 1.0;
        adap.omega = 1.0;
        adap.sigma = 0.0;
        adap.last_run_us = now;
	float cutoff_hz = adap.w0/(2*M_PI); //convert cutoff freq from rad/s to hz
	adap.filter.set_cutoff_frequency(1.0/_ahrs.get_ins().get_loop_delta_t(),cutoff_hz);
	adap.filter.reset();
        return 0;
    }    

    dt = (now - adap.last_run_us) * 1.0e-6f;
    adap.last_run_us = now;

    
    // State Predictor (first order single pole recursive filter)
    float alpha_filt = exp(-adap.alpha*dt); //alpha in rad/s 
    alpha_filt = constrain_float(alpha_filt, 0.0, 1.0);
    float beta_filt = 1-alpha_filt;  

    adap.x_m = alpha_filt*adap.x_m + beta_filt*(adap.omega*adap.u_lowpass + adap.theta*x + adap.sigma);
       
    float x_error = adap.x_m-x;
    // Constrain error to +-300 deg/s
    x_error = constrain_float(x_error,-radians(300), radians(300));

    // Calculate solution to Lyapunov 1x1 matrix
    float Pb = 1/(2*adap.alpha);

    // Projection Operator
    float theta_dot = projection_operator(adap.theta,-adap.gamma_theta*x_error*Pb*x,adap.theta_epsilon,adap.theta_max,adap.theta_min);
    float omega_dot = projection_operator(adap.omega,-adap.gamma_omega*x_error*Pb*adap.u_lowpass,adap.omega_epsilon,adap.omega_max,adap.omega_min);
    float sigma_dot = projection_operator(adap.sigma,-adap.gamma_sigma*x_error*Pb,adap.sigma_epsilon,adap.sigma_max,adap.sigma_min);
			    
    // Parameter Update using Trapezoidal integration              
    adap.theta += dt*(theta_dot);
    adap.theta += (dt/2)*(theta_dot+adap.theta);
    adap.omega += dt*(omega_dot);
    adap.omega += (dt/2)*(omega_dot+adap.omega);
    adap.sigma += dt*(sigma_dot);
    adap.sigma += (dt/2)*(sigma_dot+adap.sigma);

    adap.theta = constrain_float(adap.theta, adap.theta_min, adap.theta_max);
    adap.omega = constrain_float(adap.omega, adap.omega_min, adap.omega_max);
    adap.sigma = constrain_float(adap.sigma, adap.sigma_min, adap.sigma_max);
     
    // u (controller output to plant)
    float eta = adap.theta*x + adap.omega*adap.u_lowpass + adap.sigma - adap.r;
    eta = constrain_float(eta,-radians(90)/dt, radians(90)/dt);
    adap.u -= dt*(eta)*adap.k; // C(s)= wk/(s+wk) -> k sets the first order low pass response
    adap.u -= (dt/2)*(eta*adap.k+adap.u);  //Trapezoidal Integration
    
    // Additional cascaded second order low pass filter (strictly propper
    adap.filter.set_cutoff_frequency(1.0/_ahrs.get_ins().get_loop_delta_t(),adap.w0/(2*M_PI));
    adap.u_lowpass = adap.filter.apply(adap.u);

    // for ADAP_TUNING message
    adap.theta_dot = theta_dot;
    adap.omega_dot = omega_dot;
    adap.sigma_dot = sigma_dot;
    adap.x_error = x_error;

    DataFlash_Class::instance()->Log_Write("ADAP", "TimeUS,Dt,Atheta,Aomega,Asigma,Aeta,Axm,Ax,Ar,Axerr,Au_lowpass", "Qffffffffff",
                                           now,
                                           dt,
                                           adap.theta, 
                                           adap.omega,
                                           adap.sigma,
                                           eta,
                                           degrees(adap.x_m),
                                           degrees(x),
                                           degrees(r),
                                           degrees(x_error),
                                           degrees(adap.u_lowpass));


    _pid_info.P = adap.theta;
    _pid_info.I = adap.sigma;
    _pid_info.FF = adap.x_m;
    _pid_info.D = x_error;
    _pid_info.desired = r;
    
     return degrees(constrain_float(adap.u_lowpass,radians(-45), radians(45)))*100;
}


float AP_RollController::projection_operator(float theta, float y, float epsilon, float theta_max, float theta_min)
{
	// Calculate convex function
	// Nominal un-saturated value is above zero line on a parabolic curve
	// Steepness of curve is set by epsilon
	float f_diff2 = (theta_max-theta_min)*(theta_max-theta_min);
	float f_theta = (-4*(theta_min-theta)*(theta_max-theta))/(epsilon*f_diff2);
	float f_theta_dot = (4*(theta_min+theta_max-(2*theta)))/(epsilon*f_diff2);	

	float projection_out = y;

	if (f_theta <= 0 && f_theta_dot*y < 0)
	{
		projection_out = y*(f_theta+1); //y-(y*(-1*f_theta));
	}	
 
   	return projection_out;
}

/*
  send ADAP_TUNING message
*/
void AP_RollController::adaptive_tuning_send(mavlink_channel_t chan)
{
	if (adap.enable_chan > 0 && hal.rcin->read(adap.enable_chan-1) >= 1700 &&
        HAVE_PAYLOAD_SPACE(chan, ADAP_TUNING)) {
        mavlink_msg_adap_tuning_send(chan, PID_TUNING_ROLL, 
                                     adap.r,
                                     _ahrs.get_gyro().x,
                                     adap.x_error,
                                     adap.theta,
                                     adap.omega,
                                     adap.sigma,
                                     adap.theta_dot,
                                     adap.omega_dot,
                                     adap.sigma_dot,
                                     adap.x_m,	
                                     adap.u_lowpass,
                                     adap.u);
    }
}
