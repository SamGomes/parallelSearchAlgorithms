#pragma once

#ifndef STATE_H
#define STATE_H

#include "Macros.h"

#include <tgf.h>
#include <string>
#include <iostream>
//This class is an general implementation of the State data model


class State
{
private:
	double pedalPos; //negative means breaking
	double steerAngle;
	State* parent;
	double distance;

public:
	CUDA_HOSTDEV State();
	CUDA_HOSTDEV State(double pedalPos, double steerAngle, State* parent);
	CUDA_HOSTDEV State(double pedalPos, double steerAngle);

	CUDA_HOSTDEV double getPedalPos();
	CUDA_HOSTDEV double getSteerAngle();
	CUDA_HOSTDEV double getDistance();
	CUDA_HOSTDEV State getParent();

	CUDA_HOSTDEV void setParent(State parent);
	CUDA_HOSTDEV void setDistance(double distance);

	CUDA_HOST std::string toString();

};



#endif;