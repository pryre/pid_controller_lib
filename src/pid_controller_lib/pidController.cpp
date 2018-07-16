#include <pid_controller_lib/pidController.h>

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <pid_controller_lib/ControlParamsConfig.h>

#include <string>
#include <math.h>

pidController::pidController(const ros::NodeHandle& nh) :
	nh_(nh),
	dyncfg_control_params_(nh),
	kp_( 0.0 ),
	ki_( 0.0 ),
	kd_( 0.0 ),
	output_min_( 0.0 ),
	output_max_( 0.0 ),
	x_( 0.0 ),
	sp_( 0.0 ),
	control_output_( 0.0 ) {

	dyncfg_control_params_.setCallback(boost::bind(&pidController::callback_cfg_params, this, _1, _2));
	reset();
}

pidController::pidController(const ros::NodeHandle& nh, double initial_x, double initial_setpoint, double initial_output ) :
	nh_(nh),
	dyncfg_control_params_(nh),
	kp_( 0.0 ),
	ki_( 0.0 ),
	kd_( 0.0 ),
	output_min_( 0.0 ),
	output_max_( 0.0 ),
	x_( initial_x ),
	sp_( initial_setpoint ),
	control_output_( initial_output ) {

	dyncfg_control_params_.setCallback(boost::bind(&pidController::callback_cfg_params, this, _1, _2));

	reset( x_ );
}

pidController::~pidController() {

}

void pidController::reset() {
	reset( x_ );
}

void pidController::reset( double x_prev ) {
	integrator_ = 0.0;
	x_prev_ = x_prev;
	e_prev_ = 0.0;
}

void pidController::setKp( double Kp ) {
	kp_ = Kp;
}

void pidController::setKi( double Ki ) {
	if(Ki == 0.0) {
		//Clear the integrator so it starts fresh
		integrator_ = 0.0;
	} else if (ki_ != 0.0) {
		//Scale integrator so it has same weighting
		integrator_ *= (ki_ / Ki);
	}

	ki_ = Ki;
}

void pidController::setKd( double Kd ) {
	kd_ = Kd;
}

void pidController::setGains( double Kp, double Ki, double Kd ) {
	setKp( Kp );
	setKi( Ki );
	setKd( Kd );
}

bool pidController::setOutputMinMax( double min, double max ) {
	bool success = false;

	if( min < max ) {
		output_min_ = min;
		output_max_ = max;

		success = true;
	}

	return success;
}

//Calculate control step
double pidController::step( double dt, double sp, double x) {
	//Check to make sure the controller hasn't gone stale
	if( dt > 1.0 ) {
		reset( x );
		dt = 0.0;
	}

	x_ = x;
	sp_ = sp;

	//Calculate error
	double error = sp_ - x_;

	//Initialize Terms
	double p_term = error * kp_;
	double i_term = 0.0;
	double d_term = 0.0;

	//If it is a stale controller, just skip this section
	if( dt > 0.0 ) {
		//If the derivative is enabled
		if(kd_ > 0.0) {
			d_term = kd_ * ( (error - e_prev_) / dt );
		}

		//If the integrator is enabled
		if(ki_ > 0.0) {
			//Integrate over dt
			integrator_ += error * dt;

			//Calculate I term
			i_term = ki_ * integrator_;
		}
	}

	//Sum three terms: u = p_term + i_term - d_term
	double u = p_term + d_term;
	double ui = u + i_term;

	//Output Saturation
	double u_sat = ( ui > output_max_ ) ? output_max_ : ( (ui < output_min_ ) ? output_min_ : ui );

	//Integrator anti-windup
	//If the pid controller has saturated and if the integrator is the cause
	if( ( ui != u_sat ) && (ki_ > 0.0) && ( fabs( i_term ) > fabs( u_sat - u ) ) )
			integrator_ = ( u_sat - u ) / ki_;	//Trim the integrator to what it should currently be to only just hit the maximum

	//Set output
	control_output_ = u_sat;

	//Save last state
	x_prev_ = x;
	e_prev_ = error;

	return control_output_;
}

double pidController::getKp() {
	return kp_;
}

double pidController::getKi() {
	return ki_;
}

double pidController::getKd() {
	return kd_;
}

double pidController::getOutputMin() {
	return output_min_;
}

double pidController::getOutputMax() {
	return output_max_;
}

double pidController::getOutput() {
	return control_output_;
}

void pidController::callback_cfg_params(pid_controller_lib::ControlParamsConfig &config, uint32_t level) {
	setKp(config.p);
	setKi(config.i);
	setKd(config.d);

	if(!setOutputMinMax(config.min, config.max))
		ROS_ERROR("Invalid min/max config, ignoring");
}
