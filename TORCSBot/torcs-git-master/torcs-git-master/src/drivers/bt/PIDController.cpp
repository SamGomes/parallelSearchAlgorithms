#include "PIDController.h"

#include <stdio.h>

PIDController::PIDController(){
	PIDController(0,0,0);
}

PIDController::PIDController(double Ki, double Kp, double Kd){
	this->Ki = Ki;
	this->Kp = Kp;
	this->Kd = Kd;

	resetController();
	this->integral = 0;
}
double PIDController::getOutput(double setPoint,double mesuredValue, double deltaTime){
	
	double error = setPoint - mesuredValue;

	double prevIntegral = integral;
	double prevDerivative = derivative;
	double prevOutput = output;

	this->integral = integral + error*deltaTime;
	this->derivative = (error - previousError) / deltaTime;
	
	output = Kp*error + Ki*integral + Kd*derivative;
	
	this->previousError = error;
	printf("error:%f\n output:%f\n", this->previousError, output);

	if (this->output >= 1.0){
		this->integral = prevIntegral;
		this->derivative = prevDerivative;
		this->output = 1.0;
		return 1.0;
	}
	return output;
}

void PIDController::resetController(){
	this->previousError = 0;
}