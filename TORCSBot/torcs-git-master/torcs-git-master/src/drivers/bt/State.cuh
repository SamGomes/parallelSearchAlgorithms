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

		bool initialState; //to inform the backtrack;

		tTrackSeg posSeg;
	

		tPosd pos;
		tPosd speed;
		tPosd acceleration;

		int myGraphIndex; //index of this state
		int parentGraphIndex; //index of the parent
		

		double pathCost; //path cost (comulative distance)



	public:

		tPosd posRand;
		tPosd speedRand;


		CUDA_HOSTDEV State(); //for method initialization purposes only!

		CUDA_HOSTDEV State(tPosd finalPos, tPosd finalSpeed, tPosd acceleration);


		
		CUDA_HOSTDEV void setInitialState(bool initialState);

		CUDA_HOSTDEV bool getInitialState();



		CUDA_HOSTDEV void setCommands(tPosd finalPos, tPosd finalSpeed, tPosd acceleration);
		CUDA_HOSTDEV tPosd getSpeed();
		CUDA_HOSTDEV tPosd getPos();
		CUDA_HOSTDEV tPosd getAcceleration();


		CUDA_HOSTDEV void setPosSeg(tTrackSeg posSeg);
		CUDA_HOSTDEV tTrackSeg getPosSeg();


		CUDA_HOSTDEV void setMyGraphIndex(int myGraphIndex);
		CUDA_HOSTDEV int getMyGraphIndex();

		CUDA_HOSTDEV void setParentGraphIndex(int parentGraphIndex);
		CUDA_HOSTDEV int getParentGraphIndex();


		CUDA_HOSTDEV void setPathCost(double pathCost);
		CUDA_HOSTDEV double getPathCost();

		CUDA_HOST std::string toString();

};

#endif;

