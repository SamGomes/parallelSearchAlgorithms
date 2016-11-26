#pragma once

#ifndef STATE_H
#define STATE_H

#include "Macros.h"

#include <tgf.h>
#include <string>
#include <iostream>
#include "car.h"
//This class is an general implementation of the State data model

#include <carstruct.h>
#include <robottools.h>


typedef struct
{
	/* driver's interface */
	tCarCtrl	*ctrl;
	void	*params;
	tCarElt	*carElt;

	tCarCtrl	preCtrl;

	/* components */
	tAxle		axle[2];
	tWheel		wheel[4];
	tSteer		steer;
	tBrakeSyst		brkSyst;
	tAero		aero;
	tWing		wing[2];
	tTransmission	transmission;	/* includes clutch, gearbox and driveshaft */
	tEngine		engine;

	/* static */
	t3Dd	dimension;	/* car's mesures */
	tdble	mass;		/* mass with pilot (without fuel) */
	tdble	Minv;		/* 1 / mass with pilot (without fuel) */
	tdble	tank;		/* fuel tank capa */
	t3Dd	statGC;		/* static pos of GC */
	t3Dd	Iinv;		/* inverse of inertial moment along the car's 3 axis */

	/* dynamic */
	tdble	fuel;		/* current fuel load */
	tDynPt	DynGC;		/* GC local data except position */
	tDynPt	DynGCg;		/* GC global data */
	tPosd	VelColl;	/* resulting velocity after collision */
	tDynPt	preDynGC;	/* previous one */
	tTrkLocPos	trkPos;		/* current track position */
	tdble	airSpeed2;	/* current air speed (squared) for aerodynamic forces */

	/* internals */
	tdble	Cosz;
	tdble	Sinz;
	tDynPt	corner[4];	/* x,y,z for static relative pos, ax,ay,az for dyn. world coord */
	int		collision;
	t3Dd	normal;
	t3Dd	collpos;
	tdble	wheelbase;
	tdble	wheeltrack;
	sgMat4	posMat;
	DtShapeRef	shape;		/* for collision */
	int		blocked;		// Flag to show if the car has had already a collision in the same timestep.
	int		dammage;

	tDynPt	restPos;	/* target rest position after the car is broken */

	int		collisionAware;
} tMockedCar;







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
		double cost; //forward model cost (the bigger the better in this case)
		double pathCost; //path cost (comulative)


		double simDeltaTime = 0.02*100; //assigned for debug purposes (100 game ticks action simulation)

		tdble aMax = 0.35f; //as defined in simuv2/car.cpp


		//---------------- forward model ---------------------------
		CUDA_HOSTDEV void initForwardModel();
		CUDA_HOSTDEV void predictAcceleration();
		CUDA_HOSTDEV void computeCost();
		CUDA_HOSTDEV void computeFinalSpeed();
		CUDA_HOSTDEV void computeFinalPos();
		CUDA_HOSTDEV void normalizeAngle(double angle);
		CUDA_HOSTDEV double signOf(double number);
		//----------------------------------------------------------


	public:

		CUDA_HOSTDEV State(); //for method initialization purposes only! IT LACKS THE FORWARD MODEL! DO NOT USE IT OTHERWISE!

		CUDA_HOSTDEV State(tCarElt *car);
		CUDA_HOSTDEV State(double pedalPos, double steerAngle, State* parent);
		CUDA_HOSTDEV State(double pedalPos, double steerAngle);



		CUDA_HOSTDEV double getAcceleration();
		CUDA_HOSTDEV double getPedalPos();
		CUDA_HOSTDEV double getSteerAngle();
		CUDA_HOSTDEV double getCost();
		CUDA_HOSTDEV double getPathCost();
		CUDA_HOSTDEV double getInitialSpeed();
		CUDA_HOSTDEV double getFinalSpeed();
		CUDA_HOSTDEV tPosd getInitialPos();
		CUDA_HOSTDEV tPosd getFinalPos();
		CUDA_HOSTDEV State* getParent();

		CUDA_HOSTDEV void setParent(State* parent);
		CUDA_HOSTDEV void setPathCost(double pathCost);

		CUDA_HOSTDEV void setPedalPos(double pedalPos);
		CUDA_HOSTDEV void setSteerAngle(double steerAngle);

		CUDA_HOSTDEV void setCommands(double pedalPos,double steerAngle);

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

