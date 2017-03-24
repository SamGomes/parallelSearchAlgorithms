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

		tTrkLocPos localPos;
	
		tPosd pos;
		tPosd velocity;

		int myGraphIndex; //index of this state
		int parentGraphIndex; //index of the parent
		
		int levelFromStart; //number of nodes to initial node


	public:

		double distFromStart=0;
		tPosd posRand; //for debug purposes
		tPosd speedRand; //for debug purposes


		CUDA_HOSTDEV State(); //for method initialization purposes only!

		CUDA_HOSTDEV State(tPosd velocity);
		CUDA_HOSTDEV State(tPosd pos, tPosd velocity);


		
		CUDA_HOSTDEV void setInitialState(bool initialState);

		CUDA_HOSTDEV bool getInitialState();



		CUDA_HOSTDEV void setCommands(tPosd pos, tPosd velocity);
		CUDA_HOSTDEV void setPos(tPosd pos);
		CUDA_HOSTDEV void setVelocity(tPosd velocity);
		
		CUDA_HOSTDEV void setLevelFromStart(int levelFromStart);


		CUDA_HOSTDEV tPosd getVelocity();
		CUDA_HOSTDEV tPosd getPos();

		CUDA_HOSTDEV int getLevelFromStart();


		CUDA_HOSTDEV void setLocalPos(tTrkLocPos localPos);
		CUDA_HOSTDEV tTrkLocPos getLocalPos();


		CUDA_HOSTDEV void setMyGraphIndex(int myGraphIndex);
		CUDA_HOSTDEV int getMyGraphIndex();

		CUDA_HOSTDEV void setParentGraphIndex(int parentGraphIndex);
		CUDA_HOSTDEV int getParentGraphIndex();


		CUDA_HOST std::string toString();

};

#endif;

