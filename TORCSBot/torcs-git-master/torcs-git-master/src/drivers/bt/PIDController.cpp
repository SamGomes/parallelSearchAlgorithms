#include "PIDController.h"

PIDController::PIDController(){
	PIDController(0,0,0);
}

PIDController::PIDController(double Ki, double Kp, double Kd){
	this->Ki = Ki;
	this->Kp = Kp;
	this->Kd = Kd;

	this->previousError = 0;
	this->integral = 0;

}
double PIDController::getOutput(double setPoint,double mesuredValue, double deltaTime){
	
	double error = setPoint - mesuredValue;

	this->integral = integral + error*deltaTime;

	this->derivative = (error - previousError) / deltaTime;
	
	double output = Kp*error + Ki*integral + Kd*derivative;
	
	this->previousError = error;

	return output;
}