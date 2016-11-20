#pragma once

#ifndef STATE_H
#define STATE_H

#include "Macros.h"

#include <tgf.h>
#include <string>
#include <iostream>
#include "car.h"
//This class is an general implementation of the State data model


class State
{
private:

	tCarElt car; //copy of car state for which the state refers to

	double pedalPos; //negative means breaking
	double steerAngle;

	tPosd initialPos;
	tPosd finalPos;

	double initialSpeed;
	double finalSpeed;

	double acceleration;

	State* parent;
	double distance; //cost (the bigger the better in this case)

	double simDeltaTime = 0.02; //assigned for debug purposes

	tdble aMax = 0.35f; //as defined in simuv2/car.cpp

public:

	CUDA_HOSTDEV State(); //for method initialization purposes only! IT LACKS THE FORWARD MODEL! DO NOT USE IT OTHERWISE!

	CUDA_HOSTDEV State(tCarElt *car);
	CUDA_HOSTDEV State(double pedalPos, double steerAngle, State* parent);
	CUDA_HOSTDEV State(double pedalPos, double steerAngle);


	//---------------- forward model ---------------------------
	CUDA_HOSTDEV void initForwardModel();
	CUDA_HOSTDEV void predictAcceleration();
	CUDA_HOSTDEV void computeFinalSpeed();
	CUDA_HOSTDEV void computeFinalPos();
	CUDA_HOSTDEV void normalizeAngle(double angle);
	CUDA_HOSTDEV double signOf(double number);
	//----------------------------------------------------------

	CUDA_HOSTDEV double getAcceleration();
	CUDA_HOSTDEV double getPedalPos();
	CUDA_HOSTDEV double getSteerAngle();
	CUDA_HOSTDEV double getDistance();
	CUDA_HOSTDEV double getInitialSpeed();
	CUDA_HOSTDEV double getFinalSpeed();
	CUDA_HOSTDEV tPosd getInitialPos();
	CUDA_HOSTDEV tPosd getFinalPos();
	CUDA_HOSTDEV State* getParent();

	CUDA_HOSTDEV void setParent(State* parent);
	CUDA_HOSTDEV void setDistance(double distance);

	CUDA_HOST std::string toString();

};

#endif;

//Functions that can be used on forward model: (on car.cpp)

//static void
//SimCarUpdateSpeed(tCar *car)
//{
//	tdble	Cosz, Sinz;
//	//tdble	mass;
//
//	//mass = car->mass + car->fuel;
//
//	Cosz = car->Cosz;
//	Sinz = car->Sinz;
//
//	car->DynGCg.vel.x += car->DynGCg.acc.x * SimDeltaTime;
//	car->DynGCg.vel.y += car->DynGCg.acc.y * SimDeltaTime;
//	car->DynGCg.vel.z += car->DynGCg.acc.z * SimDeltaTime;
//
//	car->DynGCg.vel.ax += car->DynGCg.acc.ax * SimDeltaTime;
//	car->DynGCg.vel.ay += car->DynGCg.acc.ay * SimDeltaTime;
//	car->DynGCg.vel.az += car->DynGCg.acc.az * SimDeltaTime;
//
//	/* spin limitation */
//	if (fabs(car->DynGCg.vel.az) > 9.0) {
//		car->DynGCg.vel.az = SIGN(car->DynGCg.vel.az) * 9.0;
//	}
//
//	car->DynGC.vel.ax = car->DynGCg.vel.ax;
//	car->DynGC.vel.ay = car->DynGCg.vel.ay;
//	car->DynGC.vel.az = car->DynGCg.vel.az;
//
//	car->DynGC.vel.x = car->DynGCg.vel.x * Cosz + car->DynGCg.vel.y * Sinz;
//	car->DynGC.vel.y = -car->DynGCg.vel.x * Sinz + car->DynGCg.vel.y * Cosz;
//	car->DynGC.vel.z = car->DynGCg.vel.z;
//}


//SimCarUpdatePos(tCar *car)
//{
//	tdble vx, vy;
//	//tdble accx, accy;
//
//	vx = car->DynGCg.vel.x;
//	vy = car->DynGCg.vel.y;
//
//	//accx = car->DynGCg.acc.x;
//	//accy = car->DynGCg.acc.y;
//
//	car->DynGCg.pos.x += vx * SimDeltaTime;
//	car->DynGCg.pos.y += vy * SimDeltaTime;
//	car->DynGCg.pos.z += car->DynGCg.vel.z * SimDeltaTime;
//
//	car->DynGCg.pos.ax += car->DynGCg.vel.ax * SimDeltaTime;
//	car->DynGCg.pos.ay += car->DynGCg.vel.ay * SimDeltaTime;
//	car->DynGCg.pos.az += car->DynGCg.vel.az * SimDeltaTime;
//
//	NORM_PI_PI(car->DynGCg.pos.az);
//
//	if (car->DynGCg.pos.ax > aMax) car->DynGCg.pos.ax = aMax;
//	if (car->DynGCg.pos.ax < -aMax) car->DynGCg.pos.ax = -aMax;
//	if (car->DynGCg.pos.ay > aMax) car->DynGCg.pos.ay = aMax;
//	if (car->DynGCg.pos.ay < -aMax) car->DynGCg.pos.ay = -aMax;
//
//	car->DynGC.pos.x = car->DynGCg.pos.x;
//	car->DynGC.pos.y = car->DynGCg.pos.y;
//	car->DynGC.pos.z = car->DynGCg.pos.z;
//
//	car->DynGC.pos.ax = car->DynGCg.pos.ax;
//	car->DynGC.pos.ay = car->DynGCg.pos.ay;
//	car->DynGC.pos.az = car->DynGCg.pos.az;
//
//	RtTrackGlobal2Local(car->trkPos.seg, car->DynGCg.pos.x, car->DynGCg.pos.y, &(car->trkPos), TR_LPOS_MAIN);
//}

