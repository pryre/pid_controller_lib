#pragma once

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <pid_controller_lib/ControlTrackingParamsConfig.h>

#include <string>

class pidTrackingController {
	private:
		ros::NodeHandle nh_;
		dynamic_reconfigure::Server<pid_controller_lib::ControlTrackingParamsConfig> dyncfg_control_params_;

		double Kp_;
		double Ki_;
		double Kd_;
		double w0_;
		double rki_;

		double output_max_;
		double output_min_;

		double x_;					//Current state
		double xd_;					//Current state
		double r_;					//Curret setpoint
		double rd_;					//Curret setpoint
		double control_output_;		//Latest control output
		double control_output_i_;	//Latest control output integrator component

		double integrator_;
		double x_prev_;			//Previous state
		double e_prev_;			//Previous calculated error

	public:
		pidTrackingController(const ros::NodeHandle& nh = ros::NodeHandle("~"));
		pidTrackingController(const ros::NodeHandle& nh, double initial_x, double initial_setpoint, double initial_output );
		~pidTrackingController();

		void reset();
		void reset( double x_prev );

		void setw0( double w0, bool update_Ki = true );
		void setrki( double rki );
		void setGains( double w0, double rki );
		bool setOutputMinMax( double min, double max );

		double step( double dt, double r, double rd, double x, double xd );

		double getKp();
		double getKi();
		double getKd();
		double getOutputMin();
		double getOutputMax();
		double getOutput();
		double getOutputIterm();

	private:
		void callback_cfg_params(pid_controller_lib::ControlTrackingParamsConfig &config, uint32_t level);
};
