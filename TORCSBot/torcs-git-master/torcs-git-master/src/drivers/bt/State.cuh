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

		State* parent;
		

		double pathCost; //path cost (comulative distance)



	public:

		CUDA_HOSTDEV State(); //for method initialization purposes only!

		CUDA_HOSTDEV State(tPosd finalPos, tPosd finalSpeed, tPosd acceleration, State* parent);
		CUDA_HOSTDEV State(tPosd finalPos, tPosd finalSpeed, tPosd acceleration);


		
		CUDA_HOSTDEV void setInitialState(bool initialState);

		CUDA_HOSTDEV bool getInitialState();



		CUDA_HOSTDEV void setCommands(tPosd finalPos, tPosd finalSpeed, tPosd acceleration);
		CUDA_HOSTDEV tPosd getSpeed();
		CUDA_HOSTDEV tPosd getPos();
		CUDA_HOSTDEV tPosd getAcceleration();


		CUDA_HOSTDEV void setPosSeg(tTrackSeg posSeg);
		CUDA_HOSTDEV tTrackSeg getPosSeg();


		CUDA_HOSTDEV void setParent(State* parent);
		CUDA_HOSTDEV State* getParent();	


		CUDA_HOSTDEV void setPathCost(double pathCost);
		CUDA_HOSTDEV double getPathCost();

		CUDA_HOST std::string toString();

};

#endif;

