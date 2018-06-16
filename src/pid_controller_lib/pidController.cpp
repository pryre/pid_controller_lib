#include <pid_controller_lib/pidController.h>

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <pid_controller_lib/ControlParamsConfig.h>

#include <string>
#include <math.h>

pidController::pidController(ros::NodeHandle *nhp, std::string name) :
	nhp_(nhp),
	dyncfg_control_params_(ros::NodeHandle(*nhp, name)),
	kp_( 0.0 ),
	ki_( 0.0 ),
	kd_( 0.0 ),
	tau_( 0.0 ),
	x_( 0.0 ),
	x_dot_( 0.0 ),
	sp_( 0.0 ),
	control_output_( 0.0 ) {

	dyncfg_control_params_.setCallback(boost::bind(&pidController::callback_cfg_params, this, _1, _2));

	setOutputMinMax( -1.0, 1.0 );

	this->reset();
}

pidController::pidController(ros::NodeHandle *nhp, std::string name, double initial_x, double initial_x_dot, double initial_setpoint, double initial_output ) :
	nhp_(nhp),
	dyncfg_control_params_(ros::NodeHandle(*nhp, name + "/params")),
	kp_( 0.0 ),
	ki_( 0.0 ),
	kd_( 0.0 ),
	tau_( 0.0 ),
	x_( initial_x ),
	x_dot_( initial_x_dot ),
	sp_( initial_setpoint ),
	control_output_( initial_output ) {

	dyncfg_control_params_.setCallback(boost::bind(&pidController::callback_cfg_params, this, _1, _2));

	setOutputMinMax( -1.0, 1.0 );

	this->reset();
}

pidController::~pidController() {

}

void pidController::reset() {
	this->reset( x_ );
}

void pidController::reset( double x_prev ) {
	integrator_ = 0.0;
	x_prev_ = x_prev;
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

void pidController::setTau( double tau ) {
	tau_ = tau;
}

void pidController::setGains( double Kp, double Ki, double Kd ) {
	this->setKp( Kp );
	this->setKi( Ki );
	this->setKd( Kd );
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

//Calculate x_dot
double pidController::step( double dt, double sp, double x ) {
	//Check to make sure the controller hasn't gone stale
	if( dt > 1.0 ) {
		this->reset( x );
		dt = 0.0;
	}

	//Calculate D term (use dirty derivative if we don't have access to a measurement of the derivative)
	//The dirty derivative is a sort of low-pass filtered version of the derivative.
	double x_dot = ( ( 2.0 * tau_ - dt ) / ( 2.0 * tau_ + dt ) * x_dot_ ) + ( 2.0 / ( 2.0 * tau_ + dt ) * ( x - x_prev_ ) );

	double output = this->step( dt, sp, x, x_dot );

	//Save last state
	x_prev_ = x;

	return output;
}

//Calculate control step with known x_dot
//	time_now: Current time in seconds
double pidController::step( double dt, double sp, double x, double x_dot ) {
	x_dot_ = x_dot;

	//Check to make sure the controller hasn't gone stale
	if( dt > 1.0 ) {
		this->reset( x );
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
		d_term = kd_ * x_dot_;

		//If the integrator is enabled
		if(ki_ != 0.0) {
			//Integrate over dt
			integrator_ += error * dt;

			//Calculate I term
			i_term = ki_ * integrator_;
		}
	}

	//Sum three terms: u = p_term + i_term - d_term
	double u = p_term + i_term - d_term;

	//Output Saturation
	double u_sat = ( u > output_max_ ) ? output_max_ : ( (u < output_min_ ) ? output_min_ : u );

	//Integrator anti-windup
	//If the pid controller has saturated and if the integrator is the cause
	if( ( u != u_sat ) && ( fabs( i_term ) > fabs( u - p_term - d_term ) ) )
			integrator_ = ( u_sat - p_term - d_term ) / ki_;	//Trim the integrator to what it should currently be to only just hit the maximum

	//Set output
	control_output_ = u_sat;

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

double pidController::getTau() {
	return tau_;
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
	this->setKp(config.p);
	this->setKi(config.i);
	this->setKd(config.d);
	this->setTau(config.tau);

	if(!this->setOutputMinMax(config.min, config.max))
		ROS_ERROR("Invalid min/max config, ignoring");
}
