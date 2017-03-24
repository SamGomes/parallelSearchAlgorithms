#include "State.cuh"


CUDA_HOSTDEV
State::State(){

	this->initialState = false;

	this->velocity = tPosd();
	this->pos = tPosd();
	this->myGraphIndex = -1;
	this->parentGraphIndex = -1;
	this->localPos = tTrkLocPos();
	this->levelFromStart = 1;
}


CUDA_HOSTDEV
State::State(tPosd velocity){

	this->initialState = false;

	this->velocity = velocity;
	this->myGraphIndex = -1;
	this->parentGraphIndex = -1;
	this->localPos = tTrkLocPos();
	this->levelFromStart = 1;
}

CUDA_HOSTDEV
State::State(tPosd pos, tPosd velocity){

	this->initialState = false;

	this->velocity = velocity;
	this->pos = pos;
	this->myGraphIndex = -1;
	this->parentGraphIndex = -1;
	this->localPos = tTrkLocPos();
	this->levelFromStart = 1;
}



CUDA_HOSTDEV 
void State::setInitialState(bool initialState){
	this->initialState = initialState;
}

CUDA_HOSTDEV
bool State::getInitialState(){
	return initialState;
}




CUDA_HOSTDEV void State::setPos(tPosd pos){
	this->pos = pos;
}
CUDA_HOSTDEV void State::setVelocity(tPosd velocity){
	this->velocity = velocity;
}

CUDA_HOSTDEV void State::setLevelFromStart(int levelFromStart){
	this->levelFromStart = levelFromStart;
}


CUDA_HOSTDEV
void State::setCommands(tPosd pos, tPosd velocity){
	this->pos = pos;
	this->velocity = velocity;
}

CUDA_HOSTDEV
tPosd  State::getPos(){
	return this->pos;
}

CUDA_HOSTDEV
tPosd  State::getVelocity(){
	return this->velocity;
}


CUDA_HOSTDEV
int State::getLevelFromStart(){
	return this->levelFromStart;
}



CUDA_HOSTDEV
int State::getParentGraphIndex(){
	return this->parentGraphIndex;
}


CUDA_HOSTDEV
void State::setParentGraphIndex(int parentGraphIndex){
	this->parentGraphIndex = parentGraphIndex;
}


CUDA_HOSTDEV 
void State::setMyGraphIndex(int myGraphIndex){
	this->myGraphIndex = myGraphIndex;
}

CUDA_HOSTDEV 
int State::getMyGraphIndex(){
	return this->myGraphIndex;
}


CUDA_HOSTDEV
tTrkLocPos State::getLocalPos(){
	return this->localPos;
}

CUDA_HOSTDEV
void State::setLocalPos(tTrkLocPos posSeg){
	this->localPos = posSeg;
}


CUDA_HOST
std::string State::toString(){
	return
		std::string("----------StateInfo:----------- \n") +
		std::string("address: ") + std::to_string((int)this) + std::string("\n") +
		std::string("parentIndex: ") + std::to_string((int)this->parentGraphIndex) + std::string("\n") +
		std::string("- - - - - - - - - -\n") +
		std::string("pos: (") + std::to_string((double)this->pos.x) + std::string(" , ") + std::to_string((double)this->pos.y) + std::string(" ) \n") +
		std::string("velocity: (") + std::to_string((double)this->velocity.x) + std::string(" , ") + std::to_string((double)this->velocity.y) + std::string(" ) \n");
}