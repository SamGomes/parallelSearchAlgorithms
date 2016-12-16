#pragma once

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H


class PIDController{
private:

	double Ki;
	double Kp;
	double Kd;


	double previousError;

	double integral;
	double derivative;
	

public:
	PIDController();
	PIDController(double Ki,double Kp,double Kd);
	double getOutput(double setPoint, double mesuredValue, double deltaTime);
};

#endif
