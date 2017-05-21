#pragma once

#ifndef RRTSTAR_H
#define RRTSTAR_H

#include "State.cuh"
#include <vector>

//----------------- RRT Base class---------------------------
// This class is an general implementation of the planning 
// module. It is the base class for SeqRRT and ParRRT.
//-----------------------------------------------------------

class RRT
{
public:
	RRT(){}
	virtual ~RRT(){}
	virtual std::vector<State*> search() = 0;
	virtual std::vector<State> getGraph() = 0; //for debug purposes

	virtual char* getSearchName() = 0; //for debug purposes
};

#endif;

