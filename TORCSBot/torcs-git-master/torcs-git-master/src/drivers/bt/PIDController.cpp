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

	this->integral = integral + error*deltaTime;
	this->derivative = (error - previousError) / deltaTime;
	
	output = Kp*error + Ki*integral + Kd*derivative;
	
	this->previousError = error;

	if (this->output >= 1.0){
		this->integral = prevIntegral;
		this->derivative = prevDerivative;
		this->output = 1.0;
		return 1.0;
	}

	if (this->output <= -1.0){
		this->integral = prevIntegral;
		this->derivative = prevDerivative;
		this->output = -1.0;
		return -1.0;
	}

	return output;
}

void PIDController::resetController(){
	this->previousError = 0;
}

void PIDController::equalizeOutput(double targetOutput){

	double prevIntegral = integral;
	double prevDerivative = derivative;

	double prevError = previousError;

	this->previousError = (targetOutput - Ki*prevIntegral - Kd*prevDerivative) / Kp;

	this->integral = (targetOutput - Kp*prevError - Kd*prevDerivative) / Ki;
	this->derivative = (targetOutput - Kp*prevError - Ki*prevIntegral) / Kd;
}