#include "State.cuh"


CUDA_HOSTDEV
State::State(){
	this->speed = tPosd();
	this->pos = tPosd();
	this->acceleration = tPosd();
	this->parent = NULL;
	this->pathCost = 0;
	this->posSeg = tTrackSeg();
	//this->cost = DBL_MAX; //infinite
	this->cost = 0;
}

CUDA_HOSTDEV
State::State(tPosd pos, tPosd speed, tPosd acceleration, State* parent){
	this->speed = speed;
	this->pos = pos;
	this->acceleration = acceleration;
	this->parent = parent;
	this->pathCost = 0;
	this->posSeg = tTrackSeg();
	//this->cost = DBL_MAX; //infinite
	this->cost = 0;
}



CUDA_HOSTDEV
State::State(tPosd pos, tPosd speed, tPosd acceleration){
	this->speed = speed;
	this->pos = pos;
	this->acceleration = acceleration;
	this->parent = NULL;
	this->pathCost = 0;
	this->posSeg = tTrackSeg();
	//this->cost = DBL_MAX; //infinite
	this->cost = 0;
}


CUDA_HOSTDEV
tPosd  State::getPos(){
	return this->pos;
}

CUDA_HOSTDEV
tPosd  State::getSpeed(){
	return this->speed;
}

CUDA_HOSTDEV
tPosd State::getAcceleration(){
	return this->acceleration;
}


CUDA_HOSTDEV
State* State::getParent(){
	return this->parent;
}

CUDA_HOSTDEV
tTrackSeg State::getPosSeg(){
	return this->posSeg;
}

CUDA_HOSTDEV
void State::setParent(State* parent){
	this->parent = parent;
}

CUDA_HOSTDEV
void State::setPosSeg(tTrackSeg posSeg){
	this->posSeg = posSeg;
}


CUDA_HOSTDEV
double  State::getPathCost(){
	return this->pathCost;
}


CUDA_HOSTDEV
double  State::getCost(){
	return this->cost;
}

CUDA_HOSTDEV 
void State::setCommands(tPosd pos, tPosd speed, tPosd acceleration){
	this->pos = pos;
	this->speed = speed;
	this->acceleration = acceleration;
}


CUDA_HOSTDEV
void  State::setPathCost(double pathCost){
	this->pathCost = pathCost;
}


CUDA_HOSTDEV
void State::setCost(double cost){
	this->cost = cost;
}


CUDA_HOST
std::string State::toString(){
	std::string res = std::string();
		res =
		std::string("----------StateInfo:----------- \n") +
		std::string("address: ") + std::to_string((int)this) + std::string("\n") +
		std::string("parent: ") + std::to_string((int)this->parent) + std::string("\n") +
		std::string("- - - - - - - - - -\n") +
		std::string("cost: ") + std::to_string(this->cost) + std::string("\n") +
		std::string("pathCost: ") + std::to_string(this->pathCost) + std::string("\n") +
		std::string("- - - - - - - - - -\n") +
		std::string("pos: (") + std::to_string((double)this->pos.x) + std::string(" , ") + std::to_string((double)this->pos.y) + std::string(" ) \n") +
		std::string("speed: (") + std::to_string((double)this->speed.x) + std::string(" , ") + std::to_string((double)this->speed.y) + std::string(" ) \n") +
		std::string("acceleration: (") + std::to_string((double)this->acceleration.x) + std::string(" , ") + std::to_string((double)this->acceleration.y) + std::string(" ) \n");
		
	return res;
}