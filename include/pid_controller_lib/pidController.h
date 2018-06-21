#pragma once

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <pid_controller_lib/ControlParamsConfig.h>

#include <string>

class pidController {
	private:
		ros::NodeHandle *nhp_;
		dynamic_reconfigure::Server<pid_controller_lib::ControlParamsConfig> dyncfg_control_params_;

		double kp_;
		double ki_;
		double kd_;

		double output_max_;
		double output_min_;

		double x_;				//Current state
		double sp_;				//Curret setpoint
		double control_output_;	//Latest control output

		double integrator_;
		double x_prev_;			//Previous state
		double e_prev_;			//Previous calculated error

	public:
		pidController(ros::NodeHandle *nhp, std::string name);
		pidController(ros::NodeHandle *nhp, std::string name, double initial_x, double initial_setpoint, double initial_output );
		~pidController();

		void reset();
		void reset( double x_prev );

		void setKp( double Kp );
		void setKi( double Ki );
		void setKd( double Kd );
		void setGains( double Kp, double Ki, double Kd );
		bool setOutputMinMax( double min, double max );

		double step( double dt, double sp, double x );

		double getKp();
		double getKi();
		double getKd();
		double getOutputMin();
		double getOutputMax();
		double getOutput();

		void callback_cfg_params(pid_controller_lib::ControlParamsConfig &config, uint32_t level);
};
