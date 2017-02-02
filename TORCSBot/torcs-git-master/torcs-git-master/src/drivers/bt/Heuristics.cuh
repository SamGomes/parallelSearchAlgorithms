#pragma once

#ifndef HEURISTICS_H
#define HEURISTICS_H

#include "State.cuh"
#include <robottools.h>

class DeltaHeuristics{
public:
	CUDA_HOSTDEV
	static void lineHeuristic(int neighboorDeltaPos, State* state, State* parent, double diffPathCost){ //only accounts the position

		double diffX = fabs(state->getPos().x - parent->getPos().x);
		double diffY = fabs(state->getPos().y - parent->getPos().y);


		double angle = (atan2((state->getPos().y - parent->getPos().y), (state->getPos().x - parent->getPos().x)));


		//try the opposite point
		if (diffPathCost < 0){
			angle = (atan2((parent->getPos().y - state->getPos().y), (parent->getPos().x - state->getPos().x)));
		}

		NORM0_2PI(angle);

		double r = neighboorDeltaPos;

		tPosd newPos = tPosd();

		double auxCalc = (sqrt(diffX*diffX + diffY*diffY));



		if (angle >= 0 && angle <PI / 2){
			newPos = { parent->getPos().x + (r*diffX) / auxCalc, parent->getPos().y + (r*diffY) / auxCalc, state->getPos().z };
		}
		else
		if (angle >= PI / 2 && angle <PI){
			newPos = { parent->getPos().x - (r*diffX) / auxCalc, parent->getPos().y + (r*diffY) / auxCalc, state->getPos().z };
		}
		else
		if (angle >= PI && angle < 3 * PI / 2){
			newPos = { parent->getPos().x - (r*diffX) / auxCalc, parent->getPos().y - (r*diffY) / auxCalc, state->getPos().z };
		}
		else
		if (angle >= 3 * PI / 2 && angle <2 * PI){
			newPos = { parent->getPos().x + (r*diffX) / auxCalc, parent->getPos().y - (r*diffY) / auxCalc, state->getPos().z };
		}

		tPosd newSpeed = state->getSpeed();

		/*double steeredAngle = state->getPosSeg().next->next->next->arc;
		NORM_PI_PI(steeredAngle);

		printf("angle:%f\n\n", steeredAngle);
		printf("currstateAntes(%f,%f)\n", newSpeed.x, newSpeed.y);

		newSpeed.x -=  newSpeed.x * abs(40 *steeredAngle) / PI;
		newSpeed.y -=  newSpeed.y * abs(40 *steeredAngle) / PI;

		printf("currstateDepois(%f,%f)\n", newSpeed.x, newSpeed.y);*/

		state->setCommands(newPos, newSpeed, state->getAcceleration());

	}
	
	CUDA_HOSTDEV
	static void bezierHeuristic(int neighboorDeltaPos, int neighboorDeltaSpeed, State* state, State* parent, double diffPathCost){ //accounts for the position and speed

		//lineHeuristic(neighboorDeltaPos, state, parent, diffPathCost);

		double curvePercent = 0.5;

		tPosd newPos = tPosd();

		double angle = (atan2((state->getPos().y - parent->getPos().y), (state->getPos().x - parent->getPos().x)));


		//try the opposite point
		if (diffPathCost < 0){
			angle = (atan2((parent->getPos().y - state->getPos().y), (parent->getPos().x - state->getPos().x)));
		}

		NORM0_2PI(angle);

		double signedSpeedX = parent->getPos().x < 0 ? state->getSpeed().x : -1 * state->getSpeed().x;

		double signedSpeedY = parent->getPos().y < 0 ? state->getSpeed().y : -1 * state->getSpeed().y;

		if (angle >= 0 && angle <PI / 2){
			signedSpeedX = parent->getPos().x > 0 ? state->getSpeed().x : -1 * state->getSpeed().x;
			signedSpeedY = parent->getPos().y > 0 ? state->getSpeed().y : -1 * state->getSpeed().y;
		}else
		if (angle >= PI / 2 && angle <PI){
			signedSpeedX = parent->getPos().x < 0 ? state->getSpeed().x : -1 * state->getSpeed().x;
			signedSpeedY = parent->getPos().y > 0 ? state->getSpeed().y : -1 * state->getSpeed().y;
		}
		else
		if (angle >= PI && angle < 3 * PI / 2){
			signedSpeedX = parent->getPos().x < 0 ? state->getSpeed().x : -1 * state->getSpeed().x;
			signedSpeedY = parent->getPos().y < 0 ? state->getSpeed().y : -1 * state->getSpeed().y;
		}
		else
		if (angle >= 3 * PI / 2 && angle <2 * PI){
			signedSpeedX = parent->getPos().x > 0 ? state->getSpeed().x : -1 * state->getSpeed().x;
			signedSpeedY = parent->getPos().y < 0 ? state->getSpeed().y : -1 * state->getSpeed().y;
		}


		newPos.x = ((1 - curvePercent)*(1 - curvePercent)) *parent->getPos().x +
			2 * curvePercent*(1 - curvePercent)* (parent->getPos().x + 2*signedSpeedX) +
			curvePercent*curvePercent* state->getPos().x;
		newPos.y = ((1 - curvePercent)*(1 - curvePercent)) *parent->getPos().y +
			2 * curvePercent*(1 - curvePercent)* (parent->getPos().y + 2*signedSpeedY) +
			curvePercent*curvePercent* state->getPos().y;
		newPos.z = state->getPos().z;


		state->setCommands(newPos, state->getSpeed(), state->getAcceleration());

	}
};

class EvalFunctions{
public:
	CUDA_HOSTDEV
	static double evaluatePathCost(State* s1, State* s2, int fwdLimit){

		tTrkLocPos l1, l2;

		RtTrackGlobal2Local(&(s1->getPosSeg()), s1->getPos().x, s1->getPos().y, &l1, TR_LPOS_MAIN);

		RtTrackGlobal2Local(&(s2->getPosSeg()), s2->getPos().x, s2->getPos().y, &l2, TR_LPOS_MAIN);



		tTrackSeg	*l1Seg = l1.seg;
		tTrackSeg	*l2Seg = l2.seg;


		if (l1Seg->id == l2Seg->id){
			double distance = 0;

			switch (l2Seg->type) {
			case TR_STR:
				distance = l2.toStart - l1.toStart;
				break;
			default:
				distance = l2.toStart*l2Seg->radius - l1.toStart*l2Seg->radius;
				break;
			}

			return distance;
		}
		else{
			tTrackSeg* currSegFwd = l1Seg->next;
			tTrackSeg* currSegBwd = l1Seg->prev;
			double bwdDist = 0;
			double fwdDist = 0;
			while (currSegFwd != currSegBwd && fwdLimit>0){

				if (currSegFwd->id == l2Seg->id){


					switch (currSegBwd->type) {
					case TR_STR:
						return fwdDist + (l1Seg->length - l1.toStart);
					default:
						return fwdDist + (l1Seg->length*currSegBwd->radius - l1.toStart*currSegBwd->radius);
					}

				}

				if (currSegBwd->id == l2Seg->id){

					switch (currSegBwd->type) {
					case TR_STR:
						return -1 * (bwdDist + l1.toStart);
					default:
						return -1 * (bwdDist + l1.toStart*currSegBwd->radius);
					}


				}

				switch (currSegBwd->type) {
				case TR_STR:
					bwdDist += currSegBwd->length;
					break;
				default:
					bwdDist += currSegBwd->length * currSegBwd->radius;
					break;
				}

				switch (currSegFwd->type) {
				case TR_STR:
					fwdDist += currSegFwd->length;
					break;
				default:
					fwdDist += currSegFwd->length *currSegFwd->radius;
					break;
				}

				currSegFwd = currSegFwd->next;
				currSegBwd = currSegBwd->prev;
				fwdLimit--;
			}
			return -1 * (bwdDist); //when they exceed forward segs limit (or equidistant if limit exceeds half the segments)
		}
	}
	
	CUDA_HOSTDEV
	static double evaluateStateCost(State* s1, State* s2, double actionSimDeltaTime){
		double a = s2->getAcceleration().x*s2->getAcceleration().x + s2->getAcceleration().y*s2->getAcceleration().y;
		double v0 = s2->getSpeed().x*s2->getSpeed().x + s2->getSpeed().y*s2->getSpeed().y;
		double dx = ((s1->getPos().x) - (s2->getPos().x))*((s1->getPos().x) - (s2->getPos().x));
		double dy = ((s1->getPos().y) - (s2->getPos().y))*((s1->getPos().y) - (s2->getPos().y));
		double d = std::sqrtf(dx*dx + dy*dy);
		double currCost = d + a*actionSimDeltaTime*actionSimDeltaTime + v0*actionSimDeltaTime;
		return currCost;
	}
};

#endif