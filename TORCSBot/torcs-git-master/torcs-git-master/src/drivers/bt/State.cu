#include "State.cuh"


CUDA_HOSTDEV
State::State(){
	this->pedalPos = 0;
	this->steerAngle = 0;
	this->parent = NULL;
	this->pathCost = 0;
	this->cost = 0;
}

CUDA_HOSTDEV
State::State(tCarElt *car){
	this->car = *car;
	this->pedalPos = 0;
	this->steerAngle = 0;
	this->parent = NULL;
	this->pathCost = 0;
	this->cost = 0;

	this->initForwardModel();
}



CUDA_HOSTDEV
State::State(double pedalPos, double steerAngle){
	this->pedalPos = pedalPos;
	this->steerAngle = steerAngle;
	this->parent = NULL;
	this->pathCost = 0;
	this->cost = 0;
}

CUDA_HOSTDEV
State::State(double pedalPos, double steerAngle, State* parent){
	this->pedalPos = pedalPos;
	this->steerAngle = steerAngle;
	this->parent = parent;
	this->pathCost = 0;
	this->cost = 0;
}


//--------------------- forward model -------------------------------------

CUDA_HOSTDEV
void State::initForwardModel(){
	this->initialPos = this->car.pub.DynGCg.pos;
	//MOCKED SPEED!!!
	this->initialSpeed = this->car.pub.DynGC.vel.ax + this->car.pub.DynGCg.vel.ay;
	
	this->predictAcceleration();

	this->computeFinalSpeed();
	this->computeFinalPos();

	this->computeCost();
	

}


CUDA_HOSTDEV
void State::computeCost(){
	double d  = this->finalPos.x*this->finalPos.x + this->finalPos.y*this->finalPos.y;
	double d0 = this->initialPos.x*this->initialPos.x + this->initialPos.y*this->initialPos.y;
	this->cost = abs((d-d0)/((0.5*this->acceleration*this->acceleration*this->simDeltaTime)+this->initialSpeed*this->simDeltaTime)); 
}

CUDA_HOSTDEV
void State::computeFinalPos(){
	tdble vx, vy;

	vx = car.pub.DynGCg.vel.x;
	vy = car.pub.DynGCg.vel.y;


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


	this->finalPos = this->car.pub.DynGCg.pos;

}

CUDA_HOSTDEV
void State::predictAcceleration(){
	

	//tForces	F;
	//int		i;
	//tdble	m, w, minv;
	//tdble	SinTheta;
	//tdble	Cosz, Sinz;
	//tdble	v, R, Rv, Rm, Rx, Ry;

	//Cosz = car.Cosz = cos(car.pub.DynGCg.pos.az);
	//Sinz = car.Sinz = sin(car.pub.DynGCg.pos.az);

	//car.preDynGC = car.pub.DynGCg;

	///* total mass */
	//m = car.mass + car.fuel;
	//minv = 1.0 / m;
	//w = -m * 9.80665;

	///* Weight */
	//SinTheta = (-car.wheel[0].zRoad - car.wheel[1].zRoad
	//	+ car.wheel[2].zRoad + car.wheel[3].zRoad) / (2.0 * car.wheelbase);
	//F.F.x = -w * SinTheta;
	//SinTheta = (-car.wheel[0].zRoad - car.wheel[1].zRoad
	//	+ car.wheel[2].zRoad + car.wheel[3].zRoad) / (2.0 * car.wheeltrack);
	//F.F.y = -w * SinTheta;
	//F.F.z = w; /* not 3D */
	//F.M.x = F.M.y = F.M.z = 0;

	///* Wheels */
	//for (i = 0; i < 4; i++) {
	//	/* forces */
	//	F.F.x += car.wheel[i].forces.x;
	//	F.F.y += car.wheel[i].forces.y;
	//	F.F.z += car.wheel[i].forces.z;

	//	/* moments */
	//	F.M.x += car.wheel[i].forces.z * car.wheel[i].staticPos.y +
	//		car.wheel[i].forces.y * car.wheel[i].rollCenter;
	//	// Eventually TODO: activate fix below and make all cars/robots fit.
	//	//car.wheel[i].forces.y * (car.statGC.z + car.wheel[i].rideHeight);
	//	F.M.y -= car.wheel[i].forces.z * car.wheel[i].staticPos.x +
	//		car.wheel[i].forces.x * (car.statGC.z + car.wheel[i].rideHeight);
	//	F.M.z += -car.wheel[i].forces.x * car.wheel[i].staticPos.y +
	//		car.wheel[i].forces.y * car.wheel[i].staticPos.x;
	//}

	///* Aero Drag */
	//F.F.x += car.aero.drag;

	///* Wings & Aero Downforce */
	//for (i = 0; i < 2; i++) {
	//	/* forces */
	//	F.F.z += car.wing[i].forces.z + car.aero.lift[i];
	//	F.F.x += car.wing[i].forces.x;
	//	/* moments */
	//	F.M.y -= car.wing[i].forces.z * car.wing[i].staticPos.x + car.wing[i].forces.x * car.wing[i].staticPos.z;
	//	F.M.y -= car.aero.lift[i] * (car.axle[i].xpos - car.statGC.x);
	//}

	///* Rolling Resistance */
	//v = sqrt(car.pub.DynGCg.vel.x * car.pub.DynGCg.vel.x + car.pub.DynGCg.vel.y * car.pub.DynGCg.vel.y);
	//R = 0;
	//for (i = 0; i < 4; i++) {
	//	R += car.wheel[i].rollRes;
	//}
	//if (v > 0.00001) {
	//	Rv = R / v;
	//	if ((Rv * minv * simDeltaTime) > v) {
	//		Rv = v * m / simDeltaTime;
	//	}
	//}
	//else {
	//	Rv = 0;
	//}
	//Rx = Rv * car.pub.DynGCg.vel.x;
	//Ry = Rv * car.pub.DynGCg.vel.y;

	//if ((R * car.wheelbase / 2.0 * car.Iinv.z) > fabs(car.pub.DynGCg.vel.az)) {
	//	Rm = car.pub.DynGCg.vel.az / car.Iinv.z;
	//}
	//else {
	//	Rm = this->signOf(car.pub.DynGCg.vel.az) * R * car.wheelbase / 2.0;
	//}

	///* compute accelerations */
	//car.pub.DynGC.acc.x = F.F.x * minv;
	//car.pub.DynGC.acc.y = F.F.y * minv;
	//car.pub.DynGC.acc.z = F.F.z * minv;

	//car.pub.DynGCg.acc.x = (F.F.x * Cosz - F.F.y * Sinz - Rx) * minv;
	//car.pub.DynGCg.acc.y = (F.F.x * Sinz + F.F.y * Cosz - Ry) * minv;
	//car.pub.DynGCg.acc.z = car.pub.DynGC.acc.z;

	//car.pub.DynGCg.acc.ax = car.pub.DynGC.acc.ax = F.M.x * car.Iinv.x;
	//car.pub.DynGCg.acc.ay = car.pub.DynGC.acc.ay = F.M.y * car.Iinv.y;
	//car.pub.DynGCg.acc.az = car.pub.DynGC.acc.az = (F.M.z - Rm) * car.Iinv.z;





	////MOCKED ACCELERATION!!!
	this->acceleration =  1 - (1 / (pedalPos + 1)); //low pass filter



	this->car.pub.DynGC.acc.x = 0;
	this->car.pub.DynGC.acc.y = 0;
	this->car.pub.DynGC.acc.z = 0;
	this->car.pub.DynGC.acc.ax = (this->acceleration * sin(this->steerAngle));
	this->car.pub.DynGC.acc.ay = (this->acceleration * cos(this->steerAngle));
	this->car.pub.DynGC.acc.az = 0;

	/*this->car.pub.DynGC.acc.ax = this->car.pub.DynGC.acc.ax;
	this->car.pub.DynGC.acc.ay = this->car.pub.DynGC.acc.ay;
	this->car.pub.DynGC.acc.az = this->car.pub.DynGC.acc.az;
	this->car.pub.DynGC.acc.x = this->car.pub.DynGC.acc.x;
	this->car.pub.DynGC.acc.y = this->car.pub.DynGC.acc.y;
	this->car.pub.DynGC.acc.z = this->car.pub.DynGC.acc.z;*/

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










//void
//SimCarConfig(tCar *car)
//{
//	void	*hdle = car->params;
//	tdble	k;
//	tdble	w;
//	tdble	gcfrl, gcrrl, gcfr;
//	tdble	wf0, wr0;
//	tdble	overallwidth;
//	int		i;
//	tCarElt	*carElt = car->carElt;
//
//	car->dimension.x = GfParmGetNum(hdle, SECT_CAR, PRM_LEN, (char*)NULL, 4.7f);
//	car->dimension.y = GfParmGetNum(hdle, SECT_CAR, PRM_WIDTH, (char*)NULL, 1.9f);
//	overallwidth = GfParmGetNum(hdle, SECT_CAR, PRM_OVERALLWIDTH, (char*)NULL, car->dimension.y);
//	car->dimension.z = GfParmGetNum(hdle, SECT_CAR, PRM_HEIGHT, (char*)NULL, 1.2f);
//	car->mass = GfParmGetNum(hdle, SECT_CAR, PRM_MASS, (char*)NULL, 1500);
//	car->Minv = 1.0 / car->mass;
//	gcfr = GfParmGetNum(hdle, SECT_CAR, PRM_FRWEIGHTREP, (char*)NULL, .5);
//	gcfrl = GfParmGetNum(hdle, SECT_CAR, PRM_FRLWEIGHTREP, (char*)NULL, .5);
//	gcrrl = GfParmGetNum(hdle, SECT_CAR, PRM_RRLWEIGHTREP, (char*)NULL, .5);
//	car->statGC.y = -(gcfr * gcfrl + (1 - gcfr) * gcrrl) * car->dimension.y + car->dimension.y / 2.0;
//	car->statGC.z = GfParmGetNum(hdle, SECT_CAR, PRM_GCHEIGHT, (char*)NULL, .5);
//
//	car->tank = GfParmGetNum(hdle, SECT_CAR, PRM_TANK, (char*)NULL, 80);
//	car->fuel = GfParmGetNum(hdle, SECT_CAR, PRM_FUEL, (char*)NULL, 80);
//	k = GfParmGetNum(hdle, SECT_CAR, PRM_CENTR, (char*)NULL, 1.0);
//	carElt->_drvPos_x = GfParmGetNum(hdle, SECT_DRIVER, PRM_XPOS, (char*)NULL, 0.0);
//	carElt->_drvPos_y = GfParmGetNum(hdle, SECT_DRIVER, PRM_YPOS, (char*)NULL, 0.0);
//	carElt->_drvPos_z = GfParmGetNum(hdle, SECT_DRIVER, PRM_ZPOS, (char*)NULL, 0.0);
//	carElt->_bonnetPos_x = GfParmGetNum(hdle, SECT_BONNET, PRM_XPOS, (char*)NULL, carElt->_drvPos_x);
//	carElt->_bonnetPos_y = GfParmGetNum(hdle, SECT_BONNET, PRM_YPOS, (char*)NULL, carElt->_drvPos_y);
//	carElt->_bonnetPos_z = GfParmGetNum(hdle, SECT_BONNET, PRM_ZPOS, (char*)NULL, carElt->_drvPos_z);
//
//	if (car->fuel > car->tank) {
//		car->fuel = car->tank;
//	}
//	k = k * k;
//	car->Iinv.x = 12.0 / (car->mass * (car->dimension.y * car->dimension.y + car->dimension.z * car->dimension.z));
//	car->Iinv.y = 12.0 / (car->mass * (car->dimension.x * car->dimension.x + car->dimension.z * car->dimension.z));
//	car->Iinv.z = 12.0 / (car->mass * (car->dimension.y * car->dimension.y + k * car->dimension.x * car->dimension.x));
//
//	/* configure components */
//	w = car->mass * G;
//
//	wf0 = w * gcfr;
//	wr0 = w * (1 - gcfr);
//
//	car->wheel[FRNT_RGT].weight0 = wf0 * gcfrl;
//	car->wheel[FRNT_LFT].weight0 = wf0 * (1 - gcfrl);
//	car->wheel[REAR_RGT].weight0 = wr0 * gcrrl;
//	car->wheel[REAR_LFT].weight0 = wr0 * (1 - gcrrl);
//
//	for (i = 0; i < 2; i++) {
//		SimAxleConfig(car, i);
//	}
//
//	for (i = 0; i < 4; i++) {
//		SimWheelConfig(car, i);
//	}
//
//	/* Set the origin to GC */
//	car->wheelbase = car->wheeltrack = 0;
//	car->statGC.x = car->wheel[FRNT_RGT].staticPos.x * gcfr + car->wheel[REAR_RGT].staticPos.x * (1 - gcfr);
//
//	SimEngineConfig(car);
//	SimTransmissionConfig(car);
//	SimSteerConfig(car);
//	SimBrakeSystemConfig(car);
//	SimAeroConfig(car);
//	for (i = 0; i < 2; i++) {
//		SimWingConfig(car, i);
//	}
//
//	carElt->_dimension = car->dimension;
//	carElt->_statGC = car->statGC;
//	carElt->_tank = car->tank;
//	for (i = 0; i < 4; i++) {
//		carElt->priv.wheel[i].relPos = car->wheel[i].relPos;
//	}
//
//	for (i = 0; i < 4; i++) {
//		car->wheel[i].staticPos.x -= car->statGC.x;
//		car->wheel[i].staticPos.y -= car->statGC.y;
//	}
//	car->wheelbase = (car->wheel[FRNT_RGT].staticPos.x
//		+ car->wheel[FRNT_LFT].staticPos.x
//		- car->wheel[REAR_RGT].staticPos.x
//		- car->wheel[REAR_LFT].staticPos.x) / 2.0;
//	car->wheeltrack = (-car->wheel[REAR_LFT].staticPos.y
//		- car->wheel[FRNT_LFT].staticPos.y
//		+ car->wheel[FRNT_RGT].staticPos.y
//		+ car->wheel[REAR_RGT].staticPos.y) / 2.0;
//
//	/* set corners pos */
//	car->corner[FRNT_RGT].pos.x = car->dimension.x * .5 - car->statGC.x;
//	car->corner[FRNT_RGT].pos.y = -overallwidth * .5 - car->statGC.y;
//	car->corner[FRNT_RGT].pos.z = 0;
//
//	car->corner[FRNT_LFT].pos.x = car->dimension.x * .5 - car->statGC.x;
//	car->corner[FRNT_LFT].pos.y = overallwidth * .5 - car->statGC.y;
//	car->corner[FRNT_LFT].pos.z = 0;
//
//	car->corner[REAR_RGT].pos.x = -car->dimension.x * .5 - car->statGC.x;
//	car->corner[REAR_RGT].pos.y = -overallwidth * .5 - car->statGC.y;
//	car->corner[REAR_RGT].pos.z = 0;
//
//	car->corner[REAR_LFT].pos.x = -car->dimension.x * .5 - car->statGC.x;
//	car->corner[REAR_LFT].pos.y = overallwidth * .5 - car->statGC.y;
//	car->corner[REAR_LFT].pos.z = 0;
//}

















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
double State::getInitialSpeed(){
	return this->initialSpeed;
}
CUDA_HOSTDEV 
double State::getFinalSpeed(){
	return this->finalSpeed;
}
CUDA_HOSTDEV 
tPosd State::getInitialPos(){
	return this->initialPos;
}
CUDA_HOSTDEV 
tPosd State::getFinalPos(){
	return this->finalPos;
}

CUDA_HOSTDEV
double  State::getPedalPos(){
	return this->pedalPos;
}
CUDA_HOSTDEV
void  State::setPedalPos(double pedalPos){
	
	if (this->pedalPos == pedalPos){
		return;
	}

	this->pedalPos = pedalPos;
	this->initForwardModel(); //parameters have changed...
}


CUDA_HOSTDEV
double  State::getCost(){
	return this->cost;
}


CUDA_HOSTDEV
double  State::getPathCost(){
	return this->pathCost;
}


CUDA_HOSTDEV
void  State::setPathCost(double pathCost){
	this->pathCost = pathCost;
}




CUDA_HOSTDEV
double  State::getSteerAngle(){
	return this->steerAngle;
}

CUDA_HOSTDEV
void  State::setSteerAngle(double steerAngle){

	if (this->steerAngle == steerAngle){
		return;
	}

	this->steerAngle = steerAngle;
	this->initForwardModel(); //parameters have changed...
}




CUDA_HOSTDEV
void  State::setCommands(double pedalPos,double steerAngle){

	if (this->pedalPos == pedalPos &&this->steerAngle == steerAngle){
		return;
	}

	this->pedalPos = pedalPos;
	this->steerAngle = steerAngle;

	this->initForwardModel(); //parameters have changed...
}

CUDA_HOST
std::string State::toString(){
	std::string res = std::string("----------StateInfo:----------- \n") +
		std::string("address: ") + std::to_string((int)this) + std::string("\n") +
		std::string("steeringAngle: ") + std::to_string(this->steerAngle) + std::string("\n") +
		std::string("pedalPos: ") + std::to_string((double)this->pedalPos) + std::string("\n") +
		std::string("parent: ") + std::to_string((int)this->parent) + std::string("\n") +
		std::string("- - - - - - - - - -\n") +
		std::string("cost: ") + std::to_string(this->cost) + std::string("\n") +
		std::string("pathCost: ") + std::to_string(this->pathCost) + std::string("\n") +
		std::string("- - - - - - - - - -\n") +
		std::string("acceleration: ") + std::to_string((double)this->acceleration) + std::string("\n") +
		std::string("init.pos: (") + std::to_string((double)this->initialPos.x) + std::string(" , ") + std::to_string((double)this->initialPos.y) + std::string(" ) \n") +
		std::string("fin. pos: (") + std::to_string((double)this->finalPos.x) + std::string(" , ") + std::to_string((double)this->finalPos.y) + std::string(" ) \n") +
		std::string("init.speed: ") + std::to_string((double)this->initialSpeed) + std::string("\n") +
		std::string("fin. speed: ") + std::to_string((double)this->finalSpeed) + std::string("\n");


	return res;
}