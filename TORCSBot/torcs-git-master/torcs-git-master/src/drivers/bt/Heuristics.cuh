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
		float 	x, y;

		int segArrayIterator = trackSegIterator;

		tTrackSeg 	seg = segmentArray[segArrayIterator];
		float 	theta, a2;
		int 	depl = 0;
		p.type = type;


		while (segnotfound) {

			switch (seg.type) {
				case 3:
					/* rotation */
					float sine, cosine;
					float ts;
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
					break;

				case 2:
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
					break;

				case 1:
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
					break;
			}
			
		}

		/* The track is of constant width */
		/* This is subject to change */
		p.toMiddle = p.toRight - seg.width / 2.0;
		p.toLeft = seg.width - p.toRight;

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
			//if (trackSegIterator > 0 && strcmp(segArray[trackSegIterator].name, segArray[trackSegIterator - 1].name) != 0){
				targetLocalPos = UtilityMethods::SimpleRtTrackGlobal2Local(segArray, trackSegIterator, nTrackSegs, target.x, target.y, 0);
				if (targetLocalPos.toRight >  distFromSides && targetLocalPos.toLeft >  distFromSides){
					targetState->setPosSeg(*targetLocalPos.seg);
					return true;
				}
			//}
			trackSegIterator += nTrackSegs/2;
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



		tTrackSeg	l1Seg = (s1->getPosSeg());
		tTrackSeg	l2Seg = (s2->getPosSeg());


		double totalCost = 0;

		if (l1Seg.id == l2Seg.id){
			double distance = 0;

			switch (l2Seg.type) {
			case 3:
				distance = l2.toStart - l1.toStart;
				break;
			default:
				distance = (l2.toStart - l1.toStart)*l1Seg.radius;
				break;
			}

			totalCost =  distance;
		}
		else
		{
			int fwdIterator = (l1Seg.id + 1 >(nTrackSegs - 1)) ? 0 : l1Seg.id + 1;
			int bwdIterator = (l1Seg.id - 1 <0) ? nTrackSegs - 1 : l1Seg.id - 1;
			tTrackSeg currSegFwd = segmentArray[fwdIterator];
			tTrackSeg currSegBwd = segmentArray[bwdIterator];
			double bwdDist = 0;
			double fwdDist = 0;

			while (currSegFwd.id != currSegBwd.id && fwdLimit>0){

				if (currSegFwd.id == l2Seg.id){
					switch (currSegFwd.type) {
						case TR_STR:
							switch (l1Seg.type) {
								case TR_STR:
									totalCost = fwdDist + (l2Seg.length - l2.toStart) + (l1Seg.length - l1.toStart);
									break;
								default:
									totalCost = fwdDist + (l2Seg.length - l2.toStart) + (l1Seg.length - l1.toStart*l1Seg.radius);
									break;
							}
							break;
						default:
							switch (l1Seg.type) {
								case TR_STR:
									totalCost = fwdDist + (l2Seg.length - l2.toStart*currSegFwd.radius) + (l1Seg.length - l1.toStart);
									break;
								default:
									totalCost = fwdDist + (l2Seg.length - l2.toStart*currSegFwd.radius) + (l1Seg.length - l1.toStart*l1Seg.radius);
									break;
							}
							break;
					}
					break;
				}

				if (currSegBwd.id == l2Seg.id){
					switch (currSegBwd.type) {
						case TR_STR:
							switch (l1Seg.type) {
								case TR_STR:
									totalCost = - 1 * ((bwdDist + l2.toStart*currSegBwd.radius) + (l1Seg.length - l1.toStart));
									break;
								default:
									totalCost = - 1 * ((bwdDist + l2.toStart*currSegBwd.radius) + (l1Seg.length - l1.toStart*l1Seg.radius));
									break;
							}
							break;
						default:
							switch (l1Seg.type) {
								case TR_STR:
									totalCost = - 1 * ((bwdDist + l2.toStart) + (l2Seg.length - l2.toStart) + l1.toStart);
									break;
								default:
									totalCost = - 1 * ((bwdDist + l2.toStart) + (l2Seg.length - l2.toStart) + l1.toStart*l1Seg.radius);
									break;
							}
							break;
						
					}
					break;
				}

				fwdDist += currSegFwd.length;
				bwdDist += currSegBwd.length;
					

				
				fwdIterator++;
				fwdIterator = (fwdIterator >(nTrackSegs - 1)) ? 0 : fwdIterator;
				currSegFwd = (segmentArray[fwdIterator]);

				bwdIterator--;
				bwdIterator = (bwdIterator <0) ? nTrackSegs - 1 : bwdIterator;
				currSegBwd = (segmentArray[bwdIterator]);

				fwdLimit--;
			}
			if (fwdLimit == 0 && currSegBwd.id!=currSegFwd.id){
				totalCost = -1 * (bwdDist); //when they exceed forward segs limit (or equidistant if limit exceeds half the segments)
			}
		}

		totalCost = 0.8*totalCost + 0.2*totalCost*(s1->getSpeed().x*s1->getSpeed().x + s1->getSpeed().y*s1->getSpeed().y);

		return totalCost;
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
		
		float PI = 3.14159265358979323846f;

		float diffX = fabs(state->getPos().x - parent->getPos().x);
		float diffY = fabs(state->getPos().y - parent->getPos().y);


		float angle = (atan2((state->getPos().y - parent->getPos().y), (state->getPos().x - parent->getPos().x)));


		//try the opposite point
		if (diffPathCost < 0){
			angle = (atan2((parent->getPos().y - state->getPos().y), (parent->getPos().x - state->getPos().x)));
		}

		angle = UtilityMethods::norm0_2PI(angle);

		int r = neighboorDeltaPos;

		tPosd newPos = tPosd();

		float auxCalc = (sqrt(diffX*diffX + diffY*diffY));



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

		//float PI = 3.14159265358979323846f;

		float curvePercent = 0.5;

		tPosd newPos = tPosd();


		float posXDiff = (state->getPos().x - parent->getPos().x);
		float posYDiff = (state->getPos().y - parent->getPos().y);

		
		//try the opposite point
		if (diffPathCost < 0){
			posXDiff *= -1.0f;
			posYDiff *= -1.0f;

		}

		float signedSpeedX = posXDiff > 0 ? state->getSpeed().x : -1 * state->getSpeed().x;
		float signedSpeedY = posYDiff > 0 ? state->getSpeed().y : -1 * state->getSpeed().y;
	
		newPos.x = ((1 - curvePercent)*(1 - curvePercent)) *parent->getPos().x +
			2 * curvePercent*(1 - curvePercent)* (parent->getPos().x + 2.2*signedSpeedX) +
			curvePercent*curvePercent* (parent->getPos().x + posXDiff);
		newPos.y = ((1 - curvePercent)*(1 - curvePercent)) *parent->getPos().y +
			2 * curvePercent*(1 - curvePercent)* (parent->getPos().y + 2.2*signedSpeedY) +
			curvePercent*curvePercent* (parent->getPos().y + posYDiff);
		newPos.z = state->getPos().z;

		
		state->setCommands(newPos, state->getSpeed(), state->getAcceleration());

	}

	//accounts for the position and speed (quadratic)
	CUDA_HOSTDEV
		static void smoothBezierHeuristic(double neighboorDeltaPos, double neighboorDeltaSpeed, State* state, State* parent, double diffPathCost){

		float PI = 3.14159265358979323846f;

		float curvePercent = 0.6f;
		tPosd newPos = tPosd();
		float angle = (atan2((state->getPos().y - parent->getPos().y), (state->getPos().x - parent->getPos().x)));

		//try the opposite point
		if (diffPathCost < 0){
			angle = (atan2((parent->getPos().y - state->getPos().y), (parent->getPos().x - state->getPos().x)));
		}

		angle = UtilityMethods::norm0_2PI(angle);

		float speedPorportion = 0.3f;


		float diffX = UtilityMethods::mod(parent->getSpeed().x + state->getSpeed().x);
		float diffY = UtilityMethods::mod(parent->getSpeed().y + state->getSpeed().y);

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


	//accounts for the position and speed (cubic)
	CUDA_HOSTDEV
		static void cubicBezierHeuristic(double neighboorDeltaPos, double neighboorDeltaSpeed, State* state, State* parent){ //accounts for the position and speed

		float PI = 3.14159265358979323846f;

		float curvePercent = 0.3f;

		tPosd newPos = tPosd();
		tPosd newSpeed = tPosd();


		tPosd p0 = parent->getPos();
		
		tPosd p1;
		p1.x = (parent->getPos().x + parent->getSpeed().x);
		p1.y = (parent->getPos().y + parent->getSpeed().y);

		tPosd p2;
		p2.x = (state->getPos().x - state->getSpeed().x);
		p2.y = (state->getPos().y - state->getSpeed().y);
		
		tPosd p3 = state->getPos();


		double curvePercentInverse = 1 - curvePercent;

		//the new position is given by the cubic bezier formula

		newPos.x = (curvePercentInverse*curvePercentInverse*curvePercentInverse) * p0.x +
			3 * curvePercent* (curvePercentInverse*curvePercentInverse) * p1.x +
			3 * curvePercent*curvePercent*curvePercentInverse * p2.x +
			curvePercent*curvePercent*curvePercent* p3.x;

		newPos.y = (curvePercentInverse*curvePercentInverse*curvePercentInverse) *p0.y +
			3 * curvePercent* (curvePercentInverse*curvePercentInverse) * p1.y +
			3 * curvePercent*curvePercent*curvePercentInverse * p2.y +
			curvePercent*curvePercent*curvePercent* p3.y;

		newPos.z = state->getPos().z;

		double speedAbsX = (parent->getSpeed().x - state->getSpeed().x)*curvePercent;
		double speedAbsY = (parent->getSpeed().y - state->getSpeed().y)*curvePercent;

		tPosd bezierTangent = tPosd();

		//the new speed is given by the derivative of the cubic bezier formula

		bezierTangent.x = (3 * curvePercentInverse*curvePercentInverse) *(p1.x - p0.x) +
			(6 * curvePercent*curvePercentInverse) * (p2.x - p1.x) +
			3 * curvePercent*curvePercent* (p3.x - p2.x);
		
		bezierTangent.y = (3 * curvePercentInverse*curvePercentInverse) *(p1.y - p0.y) +
			(6 * curvePercent*curvePercentInverse) * (p2.y - p1.y) +
			3 * curvePercent*curvePercent* (p3.y - p2.y);

		double tangentAngle = atan2(bezierTangent.y, bezierTangent.x);


		//changed to the tangent normal
		tangentAngle += PI/2;

		newSpeed.x = state->getSpeed().x + cos(tangentAngle)*speedAbsX;
		newSpeed.y = state->getSpeed().y + sin(tangentAngle)*speedAbsY;
		newSpeed.z = state->getSpeed().z;


		state->setCommands(newPos, newSpeed, state->getAcceleration());

	}

	//wrapper
	CUDA_HOSTDEV
		static void applyDelta(State* state, State* parent, tTrackSeg* trackSegArray, int nTrackSegs, int forwardSegments, double neighborDeltaPos, double neighborDeltaSpeed){

		//double diffPathCost = EvalFunctions::evaluatePathCost(trackSegArray, nTrackSegs, parent, state, forwardSegments);
		//double diffPathCost = EvalFunctions::oldEvaluatePathCost(parent, state, this->forwardSegments);

		//lineHeuristic(neighborDeltaPos, state, parent, diffPathCost);
		//quadraticBezierHeuristic(neighborDeltaPos, neighborDeltaSpeed, state, parent, diffPathCost);
		//DeltaHeuristics::smoothBezierHeuristic(this->NEIGHBOR_DELTA_POS, this->NEIGHBOR_DELTA_SPEED, state, parent, diffPathCost);
		cubicBezierHeuristic(neighborDeltaPos, neighborDeltaSpeed, state, parent);

		/*diffPathCost = EvalFunctions::evaluatePathCost(trackSegArray, nTrackSegs, parent, state, forwardSegments);
		printf("diff: %f \n", diffPathCost);*/
	}

};

#endif