#pragma once

#ifndef STATE_H
#define STATE_H


#include "Macros.h"
#include <string>
#include <iostream>
#include "car.h"
//This class is an general implementation of the State data model

class State
{

	private:

		bool initialState; //to inform the backtrack;

		tStateRelPos localPos;
	
		tPosd pos;
		tPolarVel velocity;

		int myGraphIndex; //index of this state
		int parentGraphIndex; //index of the parent
		
		int levelFromStart; //depth of the state (number of states to initial state)

	public:
		float distFromStart = 0;

		CUDA_HOSTDEV State(); //for method initialization purposes only!

		CUDA_HOSTDEV State(tPolarVel velocity);
		CUDA_HOSTDEV State(tPosd pos,tPolarVel velocity);

		CUDA_HOSTDEV void setInitialState(bool initialState);

		CUDA_HOSTDEV bool getInitialState();

		CUDA_HOSTDEV void setPos(tPosd pos);
		CUDA_HOSTDEV void setVelocity(tPolarVel velocity);
		
		CUDA_HOSTDEV void setLevelFromStart(int levelFromStart);

		CUDA_HOSTDEV tPolarVel getVelocity();
		CUDA_HOSTDEV tPosd getPos();

		CUDA_HOSTDEV int getLevelFromStart();

		CUDA_HOSTDEV void setLocalPos(tStateRelPos localPos);
		CUDA_HOSTDEV tStateRelPos getLocalPos();

		CUDA_HOSTDEV void setMyGraphIndex(int myGraphIndex);
		CUDA_HOSTDEV int getMyGraphIndex();

		CUDA_HOSTDEV void setParentGraphIndex(int parentGraphIndex);
		CUDA_HOSTDEV int getParentGraphIndex();

		CUDA_HOST std::string toString();

};

#endif;

