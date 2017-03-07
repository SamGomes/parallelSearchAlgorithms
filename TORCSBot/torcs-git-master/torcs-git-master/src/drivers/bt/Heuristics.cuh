#pragma once

#ifndef HEURISTICS_H
#define HEURISTICS_H

#include "State.cuh"
#include <robottools.h>

class UtilityMethods{
public:

	const double PI = 3.14159265358979323846;

	CUDA_HOSTDEV
	static tTrkLocPos SimpleRtTrackGlobal2Local(tTrackSeg* segmentArray, int trackSegIterator, int nTrackSegs, tdble X, tdble Y, int type)
	{

		tTrkLocPos p;
		int 	segnotfound = 1;
		tdble 	x, y;

		int segArrayIterator = trackSegIterator;

		tTrackSeg 	seg = segmentArray[segArrayIterator];
		tdble 	theta, a2;
		int 	depl = 0;
		tdble	curWidth;
		p.type = type;


		while (segnotfound) {

			switch (seg.type) {
				case 3:

					//printf("aqui1");

					/* rotation */
					tdble sine, cosine;
					tdble ts;
					sine = sin(seg.angle[0]);
					cosine = cos(seg.angle[0]);
					x = X - seg.vertex[1].x;
					y = Y - seg.vertex[1].y;
					ts = x * cosine + y * sine;
					p.seg = &seg;
					p.toStart = ts;
					p.toRight = y * cosine - x * sine;
					if ((ts < 0) && (depl < 1)) {
						/* get back */
						segArrayIterator--;
						segArrayIterator = (segArrayIterator <0) ? nTrackSegs - 1 : segArrayIterator;
						seg = segmentArray[segArrayIterator];
						depl = -1;
					}
					else if ((ts > seg.length) && (depl > -1)) {

						segArrayIterator++;
						segArrayIterator = (segArrayIterator >(nTrackSegs - 1)) ? 0 : segArrayIterator;
						seg = segmentArray[segArrayIterator];
						depl = 1;
					}
					else {
						segnotfound = 0;
					}

					//printf("aqui1fim");

					break;

				case 2:

					//printf("aqui2");


					/* rectangular to polar */
					x = X - seg.center.x;
					y = Y - seg.center.y;
					a2 = seg.arc / 2.0;
					theta = atan2(y, x) - (seg.angle[6] + a2);
					theta = normPI_PI(theta);
					p.seg = &seg;
					p.toStart = theta + a2;
					p.toRight = seg.radiusr - sqrt(x*x + y*y);
					if ((theta < -a2) && (depl < 1)) {
						segArrayIterator--;
						segArrayIterator = (segArrayIterator <0) ? nTrackSegs - 1 : segArrayIterator;
						seg = segmentArray[segArrayIterator];
						depl = -1;
					}
					else if ((theta > a2) && (depl > -1)) {
						segArrayIterator++;
						segArrayIterator = (segArrayIterator >(nTrackSegs - 1)) ? 0 : segArrayIterator;
						seg = segmentArray[segArrayIterator];
						depl = 1;
					}
					else {
						segnotfound = 0;
					}

					//printf("aqui2fim");

					break;

				case 1:

					//printf("aqui3");

					/* rectangular to polar */
					x = X - seg.center.x;
					y = Y - seg.center.y;
					a2 = seg.arc / 2.0;
					theta = seg.angle[6] - a2 - atan2(y, x);
					theta = normPI_PI(theta);
					p.seg = &seg;
					p.toStart = theta + a2;
					p.toRight = sqrt(x*x + y*y) - seg.radiusr;
					if ((theta < -a2) && (depl < 1)) {
						segArrayIterator--;
						segArrayIterator = (segArrayIterator <0) ? nTrackSegs - 1 : segArrayIterator;
						seg = segmentArray[segArrayIterator];
						depl = -1;
					}
					else if ((theta > a2) && (depl > -1)) {
						segArrayIterator++;
						segArrayIterator = (segArrayIterator >(nTrackSegs - 1)) ? 0 : segArrayIterator;
						seg = segmentArray[segArrayIterator];
						depl = 1;
					}
					else {
						segnotfound = 0;
					}
					//printf("aqui3fim");
					break;
			}
			
		}

		/* The track is of constant width */
		/* This is subject to change */
		p.toMiddle = p.toRight - seg.width / 2.0;
		p.toLeft = seg.width - p.toRight;

		//printf("aqui4");

		return p;

	}

	CUDA_HOSTDEV
	static double normPI_PI(double angle){

		double PI = 3.14159265358979323846;

		while ((angle) > PI) { (angle) -= 2 * PI; }
		while ((angle) < -PI) { (angle) += 2 * PI; }

		return angle;
	}


	CUDA_HOSTDEV
	static double mod(double num){
		return num > 0 ? num : -1 * num;
	}

	CUDA_HOSTDEV
	static double norm0_2PI(double num) {				
					
		double PI = 3.14159265358979323846;

		while ((num) > 2 * PI) {
			(num) -= 2 * PI; 
		}
		while ((num) < 0) {
			(num) += 2 * PI; 
		}
		return num;

	}

};


class ConstraintChecking{
public:
	CUDA_HOSTDEV
	static	bool validPoint(tTrackSeg* segArray, int nTrackSegs, State* targetState, double distFromSides){
		tPosd target = targetState->getPos();

		//point outside track?
		
		int trackSegIterator = 0;
		tTrkLocPos targetLocalPos;

		while(trackSegIterator < nTrackSegs){
			targetLocalPos = UtilityMethods::SimpleRtTrackGlobal2Local(segArray, trackSegIterator, nTrackSegs, target.x, target.y, 0);
			if (targetLocalPos.toRight >  distFromSides && targetLocalPos.toLeft >  distFromSides){
				targetState->setPosSeg(*targetLocalPos.seg);
				return true;
			}
			trackSegIterator+=(nTrackSegs/2-1);
		}
		return false;
	}
	
};

class EvalFunctions{
public:
	CUDA_HOSTDEV
		static double evaluatePathCost(tTrackSeg* segmentArray, int nTrackSegs, State* s1, State* s2, int fwdLimit){

		tTrkLocPos l1, l2;

		l1 = UtilityMethods::SimpleRtTrackGlobal2Local(segmentArray, (s1->getPosSeg()).id, nTrackSegs, s1->getPos().x, s1->getPos().y, 0);	
		l2 = UtilityMethods::SimpleRtTrackGlobal2Local(segmentArray, (s2->getPosSeg()).id, nTrackSegs, s2->getPos().x, s2->getPos().y, 0);



		tTrackSeg	*l1Seg = &(s1->getPosSeg());
		tTrackSeg	*l2Seg = &(s2->getPosSeg());


		double totalCost = 0;

		if (l1Seg->id == l2Seg->id){
			double distance = 0;

			switch (l2Seg->type) {
			case 3:
				distance = l2.toStart - l1.toStart;
				break;
			default:
				distance = (l2.toStart - l1.toStart)*l1Seg->radius;
				break;
			}

			totalCost =  distance;
		}
		else
		{
			int fwdIterator = (l1Seg->id + 1 >(nTrackSegs - 1)) ? 0 : l1Seg->id + 1;
			int bwdIterator = (l1Seg->id - 1 <0) ? nTrackSegs - 1 : l1Seg->id - 1;
			tTrackSeg* currSegFwd = &segmentArray[fwdIterator];
			tTrackSeg* currSegBwd = &segmentArray[bwdIterator];
			double bwdDist = 0;
			double fwdDist = 0;

			while (currSegFwd != currSegBwd && fwdLimit>0){

				if (currSegFwd->id == l2Seg->id){
					switch (currSegFwd->type) {
						case TR_STR:
							switch (l1Seg->type) {
								case TR_STR:
									totalCost = fwdDist + (l2Seg->length - l2.toStart) + (l1Seg->length - l1.toStart);
									break;
								default:
									totalCost = fwdDist + (l2Seg->length - l2.toStart) + (l1Seg->length - l1.toStart*l1Seg->radius);
									break;
							}
							break;
						default:
							switch (l1Seg->type) {
								case TR_STR:
									totalCost = fwdDist + (l2Seg->length - l2.toStart*currSegFwd->radius) + (l1Seg->length - l1.toStart);
									break;
								default:
									totalCost = fwdDist + (l2Seg->length - l2.toStart*currSegFwd->radius) + (l1Seg->length - l1.toStart*l1Seg->radius);
									break;
							}
							break;
					}
					break;
				}

				if (currSegBwd->id == l2Seg->id){
					switch (currSegBwd->type) {
						case TR_STR:
							switch (l1Seg->type) {
								case TR_STR:
									totalCost = - 1 * ((bwdDist + l2.toStart*currSegBwd->radius) + (l1Seg->length - l1.toStart));
									break;
								default:
									totalCost = - 1 * ((bwdDist + l2.toStart*currSegBwd->radius) + (l1Seg->length - l1.toStart*l1Seg->radius));
									break;
							}
							break;
						default:
							switch (l1Seg->type) {
								case TR_STR:
									totalCost = - 1 * ((bwdDist + l2.toStart) + (l2Seg->length - l2.toStart) + l1.toStart);
									break;
								default:
									totalCost = - 1 * ((bwdDist + l2.toStart) + (l2Seg->length - l2.toStart) + l1.toStart*l1Seg->radius);
									break;
							}
							break;
						
					}
					break;
				}

				fwdDist += currSegFwd->length;
				bwdDist += currSegBwd->length;
					

				
				fwdIterator++;
				fwdIterator = (fwdIterator >(nTrackSegs - 1)) ? 0 : fwdIterator;
				currSegFwd = &(segmentArray[fwdIterator]);

				bwdIterator--;
				bwdIterator = (bwdIterator <0) ? nTrackSegs - 1 : bwdIterator;
				currSegBwd = &(segmentArray[bwdIterator]);

				fwdLimit--;
			}
			if (fwdLimit == 0 && currSegBwd!=currSegFwd){
				totalCost = -1 * (bwdDist); //when they exceed forward segs limit (or equidistant if limit exceeds half the segments)
			}
		}

		//totalCost = 0.6*totalCost + 0.4*totalCost*(4900 / (s1->getSpeed().x*s1->getSpeed().x + s1->getSpeed().y*s1->getSpeed().y));

		return totalCost;
	}

	CUDA_HOSTDEV
		static double oldEvaluatePathCost(State* s1, State* s2, int fwdLimit){

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
		double d = dx*dx + dy*dy;
		double currCost = d / v0*actionSimDeltaTime; //+a*actionSimDeltaTime*actionSimDeltaTime;
		return currCost;
	}

};




class DeltaHeuristics{

public:

	//accounts for the position (linear)
	CUDA_HOSTDEV
		static void lineHeuristic(int neighboorDeltaPos, State* state, State* parent, double diffPathCost){ //only accounts the position

		double PI = 3.14159265358979323846;

		double diffX = fabs(state->getPos().x - parent->getPos().x);
		double diffY = fabs(state->getPos().y - parent->getPos().y);


		double angle = (atan2((state->getPos().y - parent->getPos().y), (state->getPos().x - parent->getPos().x)));


		//try the opposite point
		if (diffPathCost < 0){
			angle = (atan2((parent->getPos().y - state->getPos().y), (parent->getPos().x - state->getPos().x)));
		}

		angle = UtilityMethods::norm0_2PI(angle);

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

		state->setCommands(newPos, newSpeed, state->getAcceleration());

	}

	//accounts for the position and speed (quadratic)
	CUDA_HOSTDEV
		static void quadraticBezierHeuristic(double neighboorDeltaPos, double neighboorDeltaSpeed, State* state, State* parent, double diffPathCost){

		double PI = 3.14159265358979323846;

		double curvePercent = 0.5;

		tPosd newPos = tPosd();

		double angle = (atan2((state->getPos().y - parent->getPos().y), (state->getPos().x - parent->getPos().x)));

		//try the opposite point
		if (diffPathCost < 0){
			angle = (atan2((parent->getPos().y - state->getPos().y), (parent->getPos().x - state->getPos().x)));
		}

		angle = UtilityMethods::norm0_2PI(angle);

		double signedSpeedX = 0;
		double signedSpeedY = 0;

		if (angle >= 0 && angle <PI / 2){
			signedSpeedX = parent->getPos().x > 0 ? state->getSpeed().x : -1 * state->getSpeed().x;
			signedSpeedY = parent->getPos().y > 0 ? state->getSpeed().y : -1 * state->getSpeed().y;
		}
		else
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
			2 * curvePercent*(1 - curvePercent)* (parent->getPos().x + 2.3*signedSpeedX) +
			curvePercent*curvePercent* state->getPos().x;
		newPos.y = ((1 - curvePercent)*(1 - curvePercent)) *parent->getPos().y +
			2 * curvePercent*(1 - curvePercent)* (parent->getPos().y + 2.3*signedSpeedY) +
			curvePercent*curvePercent* state->getPos().y;
		newPos.z = state->getPos().z;

		state->setCommands(newPos, state->getSpeed(), state->getAcceleration());

	}

	//accounts for the position and speed (quadratic)
	CUDA_HOSTDEV
		static void smoothBezierHeuristic(double neighboorDeltaPos, double neighboorDeltaSpeed, State* state, State* parent, double diffPathCost){

		double PI = 3.14159265358979323846;

		double curvePercent = 0.6;
		tPosd newPos = tPosd();
		double angle = (atan2((state->getPos().y - parent->getPos().y), (state->getPos().x - parent->getPos().x)));

		//try the opposite point
		if (diffPathCost < 0){
			angle = (atan2((parent->getPos().y - state->getPos().y), (parent->getPos().x - state->getPos().x)));
		}

		angle = UtilityMethods::norm0_2PI(angle);

		double speedPorportion = 0.3;


		double diffX = UtilityMethods::mod(parent->getSpeed().x + state->getSpeed().x);
		double diffY = UtilityMethods::mod(parent->getSpeed().y + state->getSpeed().y);

		if (angle >= 0 && angle <PI / 2){
			diffX = parent->getPos().x > 0 ? diffX : -1 * diffX;
			diffY = parent->getPos().y > 0 ? diffY : -1 * diffY;
		}
		else
			if (angle >= PI / 2 && angle <PI){
				diffX = parent->getPos().x < 0 ? diffX : -1 * diffX;
				diffY = parent->getPos().y > 0 ? diffY : -1 * diffY;
			}
			else
				if (angle >= PI && angle < 3 * PI / 2){
					diffX = parent->getPos().x < 0 ? diffX : -1 * diffX;
					diffY = parent->getPos().y < 0 ? diffY : -1 * diffY;
				}
				else
					if (angle >= 3 * PI / 2 && angle <2 * PI){
						diffX = parent->getPos().x > 0 ? diffX : -1 * diffX;
						diffY = parent->getPos().y < 0 ? diffY : -1 * diffY;
					}


		newPos.x = ((1 - curvePercent)*(1 - curvePercent)) *parent->getPos().x +
			2 * curvePercent*(1 - curvePercent)* (parent->getPos().x + speedPorportion*diffX) +
			curvePercent*curvePercent* state->getPos().x;
		newPos.y = ((1 - curvePercent)*(1 - curvePercent)) *parent->getPos().y +
			2 * curvePercent*(1 - curvePercent)* (parent->getPos().y + speedPorportion*diffY) +
			curvePercent*curvePercent* state->getPos().y;
		newPos.z = state->getPos().z;

		state->setCommands(newPos, state->getSpeed(), state->getAcceleration());

	}


	//accounts for the position, speed and acceleration (cubic)
	CUDA_HOSTDEV
		static void cubicBezierHeuristic(double neighboorDeltaPos, double neighboorDeltaSpeed, State* state, State* parent, double diffPathCost){ //accounts for the position and speed

		double PI = 3.14159265358979323846;

		double curvePercent = 0.5;

		tPosd newPos = tPosd();

		double angle = (atan2((state->getPos().y - parent->getPos().y), (state->getPos().x - parent->getPos().x)));

		//try the opposite point
		if (diffPathCost < 0){
			angle = (atan2((parent->getPos().y - state->getPos().y), (parent->getPos().x - state->getPos().x)));
		}
		angle = UtilityMethods::norm0_2PI(angle);

		double signedSpeedX = 0;
		double signedSpeedY = 0;

		double signedAccelerationX = 0;
		double signedAccelerationY = 0;

		if (angle >= 0 && angle <PI / 2){
			signedSpeedX = parent->getPos().x > 0 ? state->getSpeed().x : -1 * state->getSpeed().x;
			signedSpeedY = parent->getPos().y > 0 ? state->getSpeed().y : -1 * state->getSpeed().y;

			signedAccelerationX = parent->getPos().x > 0 ? state->getAcceleration().x : -1 * state->getAcceleration().x;
			signedAccelerationY = parent->getPos().y > 0 ? state->getAcceleration().y : -1 * state->getAcceleration().y;
		}
		else
			if (angle >= PI / 2 && angle <PI){
				signedSpeedX = parent->getPos().x < 0 ? state->getSpeed().x : -1 * state->getSpeed().x;
				signedSpeedY = parent->getPos().y > 0 ? state->getSpeed().y : -1 * state->getSpeed().y;

				signedAccelerationX = parent->getPos().x < 0 ? state->getAcceleration().x : -1 * state->getAcceleration().x;
				signedAccelerationY = parent->getPos().y > 0 ? state->getAcceleration().y : -1 * state->getAcceleration().y;
			}
			else
				if (angle >= PI && angle < 3 * PI / 2){
					signedSpeedX = parent->getPos().x < 0 ? state->getSpeed().x : -1 * state->getSpeed().x;
					signedSpeedY = parent->getPos().y < 0 ? state->getSpeed().y : -1 * state->getSpeed().y;

					signedAccelerationX = parent->getPos().x < 0 ? state->getAcceleration().x : -1 * state->getAcceleration().x;
					signedAccelerationY = parent->getPos().y < 0 ? state->getAcceleration().y : -1 * state->getAcceleration().y;
				}
				else
					if (angle >= 3 * PI / 2 && angle <2 * PI){
						signedSpeedX = parent->getPos().x > 0 ? state->getSpeed().x : -1 * state->getSpeed().x;
						signedSpeedY = parent->getPos().y < 0 ? state->getSpeed().y : -1 * state->getSpeed().y;

						signedAccelerationX = parent->getPos().x > 0 ? state->getAcceleration().x : -1 * state->getAcceleration().x;
						signedAccelerationY = parent->getPos().y < 0 ? state->getAcceleration().y : -1 * state->getAcceleration().y;
					}

		newPos.x = ((1 - curvePercent)*(1 - curvePercent)*(1 - curvePercent)) *parent->getPos().x +
			3 * curvePercent* ((1 - curvePercent)*(1 - curvePercent)) * (parent->getPos().x + 2 * signedSpeedX) +
			3 * curvePercent* ((1 - curvePercent)*(1 - curvePercent)) * (parent->getPos().x + 2 * signedSpeedX + signedAccelerationX) +
			curvePercent*curvePercent*curvePercent* state->getPos().x;

		newPos.y = ((1 - curvePercent)*(1 - curvePercent)*(1 - curvePercent)) *parent->getPos().y +
			3 * curvePercent* ((1 - curvePercent)*(1 - curvePercent)) * (parent->getPos().y + 2 * signedSpeedY) +
			3 * curvePercent* ((1 - curvePercent)*(1 - curvePercent)) * (parent->getPos().y + 2 * signedSpeedY + signedAccelerationY) +
			curvePercent*curvePercent*curvePercent* state->getPos().y;

		newPos.z = state->getPos().z;

		state->setCommands(newPos, state->getSpeed(), state->getAcceleration());

	}

	//wrapper
	CUDA_HOSTDEV
		static void applyDelta(State* state, State* parent, tTrackSeg* trackSegArray, int nTrackSegs, int forwardSegments, double neighborDeltaPos, double neighborDeltaSpeed){

		double diffPathCost = EvalFunctions::evaluatePathCost(trackSegArray, nTrackSegs, parent, state, forwardSegments);
		//double diffPathCost = EvalFunctions::oldEvaluatePathCost(parent, state, this->forwardSegments);

		//DeltaHeuristics::lineHeuristic(this->NEIGHBOR_DELTA_POS,state, parent, diffPathCost);
		quadraticBezierHeuristic(neighborDeltaPos, neighborDeltaSpeed, state, parent, diffPathCost);
		//DeltaHeuristics::smoothBezierHeuristic(this->NEIGHBOR_DELTA_POS, this->NEIGHBOR_DELTA_SPEED, state, parent, diffPathCost);
		//DeltaHeuristics::cubicBezierHeuristic(this->NEIGHBOR_DELTA_POS, this->NEIGHBOR_DELTA_SPEED, state, parent, diffPathCost);

	}

};

#endif