#include "State.cuh"


CUDA_HOSTDEV
State::State(){
	this->pedalPos = 0;
	this->steerAngle = 0;
	this->parent = NULL;
	this->distance = 0;
}

CUDA_HOSTDEV
State::State(double pedalPos, double steerAngle){
	this->pedalPos = pedalPos;
	this->steerAngle = steerAngle;
	this->parent = NULL;
	this->distance = 0;
}

CUDA_HOSTDEV
State::State(double pedalPos, double steerAngle,State* parent){
	this->pedalPos = pedalPos;
	this->steerAngle = steerAngle;
	this->parent = parent;
	this->distance = 0;
}


CUDA_HOSTDEV
State* State::getParent(){
	return this->parent;
}


CUDA_HOSTDEV
void State::setParent(State* parent){
	this->parent = parent;
}




CUDA_HOSTDEV
double  State::getPedalPos(){
	return this->pedalPos;
}





CUDA_HOSTDEV
double  State::getDistance(){
	return this->distance;
}


CUDA_HOSTDEV
void  State::setDistance(double distance){
	this->distance = distance;
}




CUDA_HOSTDEV
double  State::getSteerAngle(){
	return this->steerAngle;
}



CUDA_HOST
std::string State::toString(){
	std::string res = std::string("----------StateInfo:----------- \n") + 
		std::string("address: ") + std::to_string((int)this) + std::string("\n") +
		std::string("steeringAngle: ") + std::to_string(this->steerAngle) + std::string("\n") +
		std::string("pedalPos: ") + std::to_string((float)this->pedalPos) + std::string("\n") +
		std::string("parent: ") + std::to_string((int)this->parent) + std::string("\n") +
		std::string("distance: ") + std::to_string(this->distance) + std::string("\n");

	return res;
}