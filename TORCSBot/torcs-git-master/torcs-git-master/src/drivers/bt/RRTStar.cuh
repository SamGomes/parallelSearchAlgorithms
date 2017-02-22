#pragma once

#ifndef RRTSTAR_H
#define RRTSTAR_H

#include "State.cuh"
#include <vector>

//This class is an general implementation of the Search sub-module within the planning module
class RRTStar
{

public:
	RRTStar(){}
	virtual ~RRTStar(){}
	virtual void updateCar(tCarElt car) = 0; //for multiple search calls
	virtual std::vector<State*> search() = 0;
	virtual std::vector<State*> getGraph() = 0; //for debug purposes

};

#endif;
