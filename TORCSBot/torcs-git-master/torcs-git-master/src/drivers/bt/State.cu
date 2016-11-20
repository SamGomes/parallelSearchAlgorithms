#include "State.cuh"


CUDA_HOSTDEV
State::State(){
	this->pedalPos = 0;
	this->steerAngle = 0;
	this->parent = NULL;
	this->distance = 0;
}

CUDA_HOSTDEV
State::State(tCarElt *car){
	this->car = *car;
	this->pedalPos = 0;
	this->steerAngle = 0;
	this->parent = NULL;
	this->distance = 0;

	this->initForwardModel();
}



CUDA_HOSTDEV
State::State(double pedalPos, double steerAngle){
	this->pedalPos = pedalPos;
	this->steerAngle = steerAngle;
	this->parent = NULL;
	this->distance = 0;
}

CUDA_HOSTDEV
State::State(double pedalPos, double steerAngle, State* parent){
	this->pedalPos = pedalPos;
	this->steerAngle = steerAngle;
	this->parent = parent;
	this->distance = 0;
}


//--------------------- forward model -------------------------------------

CUDA_HOSTDEV
void State::initForwardModel(){
	this->initialPos = this->car.pub.DynGCg.pos;
	//MOCKED SPEED!!!
	this->initialSpeed = this->car.pub.DynGCg.vel.ax + this->car.pub.DynGCg.vel.ay;
	
	this->predictAcceleration();

	this->computeFinalSpeed();
	this->computeFinalPos();

}


CUDA_HOSTDEV
void State::computeFinalPos(){
	tdble vx, vy;

	vx = car.pub.DynGCg.vel.x;
	vy = car.pub.DynGCg.vel.y;

	printf("pos: %f", car.pub.DynGCg.pos.x);

	this->car.pub.DynGCg.pos.x += vx * simDeltaTime;
	this->car.pub.DynGCg.pos.y += vy * simDeltaTime;
	this->car.pub.DynGCg.pos.z += this->car.pub.DynGCg.vel.z * simDeltaTime;

	this->car.pub.DynGCg.pos.ax += this->car.pub.DynGCg.vel.ax * simDeltaTime;
	this->car.pub.DynGCg.pos.ay += this->car.pub.DynGCg.vel.ay * simDeltaTime;
	this->car.pub.DynGCg.pos.az += this->car.pub.DynGCg.vel.az * simDeltaTime;

	this->normalizeAngle(this->car.pub.DynGCg.pos.az);

	if (this->car.pub.DynGCg.pos.ax > aMax) this->car.pub.DynGCg.pos.ax = aMax;
	if (this->car.pub.DynGCg.pos.ax < -aMax) this->car.pub.DynGCg.pos.ax = -aMax;
	if (this->car.pub.DynGCg.pos.ay > aMax) this->car.pub.DynGCg.pos.ay = aMax;
	if (this->car.pub.DynGCg.pos.ay < -aMax) this->car.pub.DynGCg.pos.ay = -aMax;


	printf("pos: %f", this->car.pub.DynGCg.pos.x);

	this->finalPos = this->car.pub.DynGCg.pos;

}

CUDA_HOSTDEV
void State::predictAcceleration(){
	
	//MOCKED ACCELERATION!!!
	this->acceleration = pedalPos*10;
	this->car.pub.DynGCg.acc.ax = this->acceleration;
	this->car.pub.DynGCg.acc.ay = this->acceleration;
	this->car.pub.DynGCg.acc.az = this->acceleration;
	this->car.pub.DynGCg.acc.x = this->acceleration;
	this->car.pub.DynGCg.acc.y = this->acceleration;
	this->car.pub.DynGCg.acc.z = this->acceleration;

}

CUDA_HOSTDEV
void State::computeFinalSpeed(){
	tdble	Cosz, Sinz;
	
	Cosz = cos(this->car.pub.DynGCg.pos.az);
	Sinz = sin(this->car.pub.DynGCg.pos.az);
	
	this->car.pub.DynGCg.vel.x += this->car.pub.DynGCg.acc.x * simDeltaTime;
	this->car.pub.DynGCg.vel.y += this->car.pub.DynGCg.acc.y * simDeltaTime;
	this->car.pub.DynGCg.vel.z += this->car.pub.DynGCg.acc.z * simDeltaTime;
	
	this->car.pub.DynGCg.vel.ax += this->car.pub.DynGCg.acc.ax * simDeltaTime;
	this->car.pub.DynGCg.vel.ay += this->car.pub.DynGCg.acc.ay * simDeltaTime;
	this->car.pub.DynGCg.vel.az += this->car.pub.DynGCg.acc.az * simDeltaTime;
	
	/* spin limitation */
	if (fabs(this->car.pub.DynGCg.vel.az) > 9.0) {
		this->car.pub.DynGCg.vel.az = this->signOf(this->car.pub.DynGCg.vel.az) * 9.0;
	}
	
	
	
	this->car.pub.DynGC.vel.x = this->car.pub.DynGCg.vel.x * Cosz + this->car.pub.DynGCg.vel.y * Sinz;
	this->car.pub.DynGC.vel.y = -this->car.pub.DynGCg.vel.x * Sinz + this->car.pub.DynGCg.vel.y * Cosz;
	this->car.pub.DynGC.vel.z = this->car.pub.DynGCg.vel.z;

	//MOCKED SPEED!!!
	this->finalSpeed = this->car.pub.DynGCg.vel.ax + this->car.pub.DynGCg.vel.ay;

}

CUDA_HOSTDEV
void State::normalizeAngle(double angle){
	double PI = 3.1415;
	while (angle > PI) {
		angle -= 2 * PI; 
	}
	while (angle < -PI) {
		angle += 2 * PI; 
	}

}

CUDA_HOSTDEV
double State::signOf(double number){
	return (number > 0) ? 1.0 : -1.0;
}


//-------------------------------------------------------------------

CUDA_HOSTDEV
double State::getAcceleration(){
	return this->acceleration;
}


CUDA_HOSTDEV
State* State::getParent(){
	return this->parent;
}


CUDA_HOSTDEV
void State::setParent(State* parent){
	this->parent = parent;
	this->car = parent->car;
	this->initForwardModel();
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
		std::string("pedalPos: ") + std::to_string((double)this->pedalPos) + std::string("\n") +
		std::string("parent: ") + std::to_string((int)this->parent) + std::string("\n") +
		std::string("distance: ") + std::to_string(this->distance) + std::string("\n") +

		std::string("acceleration: ") + std::to_string((double)this->acceleration) + std::string("\n") +
		std::string("init.pos: (") + std::to_string((double)this->initialPos.x) + std::string(" , ") + std::to_string((int)this->initialPos.y) + std::string(" ) \n") +
		std::string("fin. pos: (") + std::to_string((double)this->finalPos.x) + std::string(" , ") + std::to_string((int)this->finalPos.y) + std::string(" ) \n") +
		std::string("init.speed: ") + std::to_string((double)this->initialSpeed) + std::string("\n") +
		std::string("fin. speed: ") + std::to_string((double)this->finalSpeed) + std::string("\n");


	return res;
}