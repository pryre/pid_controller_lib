#include <pid_controller_lib/pidTrackingController.h>

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <pid_controller_lib/ControlTrackingParamsConfig.h>

#include <string>
#include <math.h>

pidTrackingController::pidTrackingController(const ros::NodeHandle& nh) :
	nh_(nh),
	dyncfg_control_params_(nh),
	Kp_( 0.0 ),
	Ki_( 0.0 ),
	Kd_( 0.0 ),
	output_min_( 0.0 ),
	output_max_( 0.0 ),
	x_( 0.0 ),
	xd_( 0.0 ),
	r_( 0.0 ),
	rd_( 0.0 ),
	control_output_( 0.0 ) {

	dyncfg_control_params_.setCallback(boost::bind(&pidTrackingController::callback_cfg_params, this, _1, _2));
	reset();
}

pidTrackingController::pidTrackingController(const ros::NodeHandle& nh, double initial_x, double initial_setpoint, double initial_output ) :
	nh_(nh),
	dyncfg_control_params_(nh),
	Kp_( 0.0 ),
	Ki_( 0.0 ),
	Kd_( 0.0 ),
	output_min_( 0.0 ),
	output_max_( 0.0 ),
	x_( initial_x ),
	r_( initial_setpoint ),
	control_output_( initial_output ) {

	dyncfg_control_params_.setCallback(boost::bind(&pidTrackingController::callback_cfg_params, this, _1, _2));

	reset( x_ );
}

pidTrackingController::~pidTrackingController() {
}

void pidTrackingController::reset() {
	reset( x_ );
}

void pidTrackingController::reset( double x_prev ) {
	integrator_ = 0.0;
	x_prev_ = x_prev;
	e_prev_ = 0.0;

	control_output_ = 0.0;
	control_output_i_ = 0.0;
}

void pidTrackingController::setw0( double w0, bool update_ki ) {
	w0_ = w0;

	Kp_ = w0*w0;
	Kd_ = 2.0*w0;

	if( update_ki )
		setki(ki_);
}

void pidTrackingController::setki( double ki ) {
	//XXX: ki Should not be set more than 0.5! Will cause instabilities
	double Ki = ki*Kp_;

	if(ki == 0.0) {
		//Clear the integrator so it starts fresh
		integrator_ = 0.0;
	} else if (Ki_ != 0.0) {
		//Scale integrator so it has same weighting
		integrator_ *= (Ki_ / Ki);
	}

	ki_ = ki;
	Ki_ = Ki;
}

void pidTrackingController::setGains( double w0, double ki ) {
	setw0( w0, false );
	setki( ki );
}

bool pidTrackingController::setOutputMinMax( double min, double max ) {
	bool success = false;

	if( min < max ) {
		output_min_ = min;
		output_max_ = max;

		success = true;
	}

	return success;
}

//Calculate control step
double pidTrackingController::step( double dt, double r, double rd, double x, double xd ) {
	//Check to make sure the controller hasn't gone stale
	if( dt > 1.0 ) {
		reset( x );
		dt = 0.0;
	}

	//Calculate error
	double e = r - x;
	double ed = rd - xd;

	//Initialize Terms
	double p_term = Kp_*e;
	double i_term = 0.0;
	double d_term = Kd_*ed;

	//If it is a stale controller, just skip this section
	if( dt > 0.0 ) {
		//If the integrator is enabled
		if(Ki_ > 0.0) {
			//Integrate over dt
			integrator_ += e * dt;

			//Calculate I term
			i_term = Ki_ * integrator_;
		}
	}

	//Sum three terms: u = p_term + i_term - d_term
	double u = p_term + d_term;
	double ui = u + i_term;

	//Output Saturation
	double u_sat = ( ui > output_max_ ) ? output_max_ : ( (ui < output_min_ ) ? output_min_ : ui );

	//Integrator anti-windup
	//If the pid controller has saturated and if the integrator is the cause
	if( ( ui != u_sat ) && (Ki_ > 0.0) && ( fabs( i_term ) > fabs( u_sat - u ) ) )
			integrator_ = ( u_sat - u ) / Ki_;	//Trim the integrator to what it should currently be to only just hit the maximum

	//Set output
	control_output_ = u_sat;
	control_output_i_ = Ki_ * integrator_; //Saturated integrator term

	//Save last state
	x_prev_ = x;
	e_prev_ = e;
	r_ = r;
	rd_ = rd;
	x_ = x;
	xd_ = xd;

	return control_output_;
}

double pidTrackingController::getKp() {
	return Kp_;
}

double pidTrackingController::getKi() {
	return Ki_;
}

double pidTrackingController::getKd() {
	return Kd_;
}

double pidTrackingController::getOutputMin() {
	return output_min_;
}

double pidTrackingController::getOutputMax() {
	return output_max_;
}

double pidTrackingController::getOutput() {
	return control_output_;
}

double pidTrackingController::getOutputIterm() {
	return control_output_i_;
}

void pidTrackingController::callback_cfg_params(pid_controller_lib::ControlTrackingParamsConfig &config, uint32_t level) {
	setGains(config.w0, config.ki);

	if(!setOutputMinMax(config.min, config.max))
		ROS_ERROR("Invalid min/max config, ignoring");
}
