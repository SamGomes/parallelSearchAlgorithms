#pragma once

#ifndef HEURISTICS_H
#define HEURISTICS_H

#include "State.cuh"
#include <robottools.h>

class UtilityMethods{
public:

	const double PI = 3.14159265358979323846;

	CUDA_HOSTDEV
	static bool SimpleRtTrackGlobal2Local(tTrkLocPos &p, tTrackSeg* segmentArray, int nTrackSegs, tdble X, tdble Y, int type)
	{

		int 	segnotfound = 1;
		float 	x, y;


		tTrackSeg 	seg;
		
		if (!getSegmentOf(seg, segmentArray, nTrackSegs, X, Y)){
			return false;
		}
		int segArrayIterator = seg.id;
		float 	theta, a2;
		int 	depl = 0;
		p.type = type;
		p.seg = &segmentArray[seg.id];



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

		return true;

	}


	CUDA_HOSTDEV
	static bool getSegmentOf(tTrackSeg& seg, tTrackSeg* segmentArray, int nTrackSegs, tdble x, tdble y){
		for (int i = 0; i < nTrackSegs; i++){

			t3Dd* bounds = segmentArray[i].vertex;

			double dist03 = getEuclideanQuadranceBetween({ bounds[0].x, bounds[0].y, bounds[0].z }, { bounds[3].x, bounds[3].y, bounds[3].z });
			double dist12 = getEuclideanQuadranceBetween({ bounds[1].x, bounds[1].y, bounds[1].z }, { bounds[2].x, bounds[2].y, bounds[2].z });

			double distP0 = getEuclideanQuadranceBetween({ x, y, 0 }, { bounds[0].x, bounds[0].y, bounds[0].z });
			double distP1 = getEuclideanQuadranceBetween({ x, y, 0 }, { bounds[1].x, bounds[1].y, bounds[1].z });
			double distP2 = getEuclideanQuadranceBetween({ x, y, 0 }, { bounds[2].x, bounds[2].y, bounds[2].z });
			double distP3 = getEuclideanQuadranceBetween({ x, y, 0 }, { bounds[3].x, bounds[3].y, bounds[3].z });

			
			if ((distP0 < dist03) && (distP3 < dist03) && (distP1 < dist12) && (distP2 < dist12)){
				seg = segmentArray[i];
				//std::cout << "tou dentro!" << seg.id << std::endl;
				return true;
			}
		}
		return false;
	}

	//calculates euclidean distance between points
	CUDA_HOSTDEV
	static double getEuclideanQuadranceBetween(tPosd p1, tPosd p2){
		return (p2.x - p1.x)*(p2.x - p1.x) + (p2.y - p1.y)*(p2.y - p1.y);
	}

	//calculates polar distance between points
	CUDA_HOSTDEV
	static double getPolarQuadranceBetween(tPosd p1, tPosd p2){

		double w1 = 1;
		double w2 = 0;

		double p1Angle = atan(p1.y / p1.x);
		double p2Angle = atan(p2.y / p2.x);

		double deltaAngle = p2Angle - p1Angle;

		double p1IntQuad = p1.x*p1.x + p1.y*p1.y;
		double p2IntQuad = p2.x*p2.x + p2.y*p2.y;

		double deltaIntesityQuad = p2IntQuad - p1IntQuad;
		return w1*deltaAngle + w2*deltaIntesityQuad;
	}

	CUDA_HOSTDEV
	static double getDotBetween(tPosd p1, tPosd p2){
		return p1.x*p2.x + p1.y*p2.y;
	}

	CUDA_HOSTDEV
	static double getTrackCenterDistanceBetween(tTrackSeg* segmentArray, int nTrackSegs, State* s2, State* s1, int fwdLimit){

		tTrkLocPos l1, l2;

		l1 = s1->getLocalPos();
		l2 = s2->getLocalPos();


		tTrackSeg	l1Seg = *l1.seg;
		tTrackSeg	l2Seg = *l2.seg;


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

			totalCost = distance;
		}
		else
		{
			int fwdIterator = (l1Seg.id + 1 > (nTrackSegs - 1)) ? 0 : l1Seg.id + 1;
			int bwdIterator = (l1Seg.id - 1 <0) ? nTrackSegs - 1 : l1Seg.id - 1;
			tTrackSeg currSegFwd = segmentArray[fwdIterator];
			tTrackSeg currSegBwd = segmentArray[bwdIterator];
			double bwdDist = 0;
			double fwdDist = 0;

			double l1ToEnd = 0;
			double l2ToEnd = 0;


			switch (l1Seg.type) {
				case 3:
					l1ToEnd = (l1Seg.length - l1.toStart);
					break;
				default:
					l1ToEnd = l1Seg.length - l1.toStart*l1Seg.radius;
					break;
			}

			switch (l2Seg.type) {
				case 3:
					l2ToEnd = (l2Seg.length - l2.toStart);
					break;
				default:
					l2ToEnd = l2Seg.length - l2.toStart*l2Seg.radius;
					break;
			}


			while (currSegFwd.id != currSegBwd.id && fwdLimit>0){

				if (currSegFwd.id == l2Seg.id){
					switch (currSegFwd.type) {
						case 3:
							totalCost = fwdDist + (l2.toStart + l1ToEnd);
							break;
						default:
							totalCost = fwdDist + (l2.toStart*l2Seg.radius + l1ToEnd);
							break;
					}
					break;
				}

				if (currSegBwd.id == l2Seg.id){
					switch (currSegBwd.type) {
						case 3:
							totalCost = -1* (bwdDist + (l1.toStart + l2ToEnd));
							break;
						default:
							totalCost = -1*(bwdDist + (l1.toStart*l1Seg.radius + l2ToEnd));
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
			if (fwdLimit == 0 && currSegBwd.id != currSegFwd.id){
				totalCost =  (bwdDist); //when they exceed forward segs limit (or equidistant if limit exceeds half the segments)
			}
		}

		return totalCost;
	}


	CUDA_HOSTDEV
	static tdble SimpleGetDistanceFromStart(tTrkLocPos p)
	{
		tTrackSeg	*seg;
		tdble	lg;

		seg = p.seg;
		lg = seg->lgfromstart;

		switch (seg->type) {
			case 3:
				lg += p.toStart;
				break;
			default:
				lg += p.toStart * seg->radius;
				break;
		}
		return lg;
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

class RandomStateGenerators{

public:
	CUDA_HOST
	static double randToNormalRand(double mean, double stddev)
	{   
		//Box muller method
		static double n2 = 0.0;
		static int n2_cached = 0;
		if (!n2_cached)
		{
			double x, y, r;
			do
			{
				x = 2.0*rand() / RAND_MAX - 1;
				y = 2.0*rand() / RAND_MAX - 1;

				r = x*x + y*y;
			} while (r == 0.0 || r > 1.0);
			{
				double d = sqrt(-2.0*log(r) / r);
				double n1 = x*d;
				n2 = y*d;
				double result = n1*stddev + mean;
				n2_cached = 1;
				return result;
			}
		}
		else
		{
			n2_cached = 0;
			return n2*stddev + mean;
		}
	}

	CUDA_HOST
	static State uniformRandomState(tTrackSeg* trackSegArray, int nTrackSegs, int initialSegIndex, int finalSegIndex){

		//-------------------- random position calculation --------------------

		/**/

		double minXVertex = DBL_MAX;
		double maxXVertex = -1 * DBL_MAX;

		double minYVertex = DBL_MAX;
		double maxYVertex = -1 * DBL_MAX;


		//check if there are lower and higher bounds
		for (int i = initialSegIndex; i%nTrackSegs != finalSegIndex; i++){

			tTrackSeg currSeg = trackSegArray[i%nTrackSegs];


			if (currSeg.vertex[2].x < minXVertex){
				minXVertex = currSeg.vertex[2].x;
			}

			if (currSeg.vertex[3].x > maxXVertex){
				maxXVertex = currSeg.vertex[3].x;
			}

			if (currSeg.vertex[1].y < minYVertex){
				minYVertex = currSeg.vertex[1].y;
			}


			if (currSeg.vertex[0].y > maxYVertex){
				maxYVertex = currSeg.vertex[0].y;
			}


		}

		double trackMapXMin = minXVertex;
		double trackMapXMax = maxXVertex;

		double trackMapXDelta = trackMapXMax - trackMapXMin;

		double trackMapYMin = minYVertex;
		double trackMapYMax = maxYVertex;

		double trackMapYDelta = trackMapYMax - trackMapYMin;

		tPosd randPos;
		randPos.x = trackMapXDelta * ((double)std::rand() / (double)RAND_MAX) + trackMapXMin;
		randPos.y = trackMapYDelta * ((double)std::rand() / (double)RAND_MAX) + trackMapYMin;

		/**/

		//-------------------- random velocity calculation --------------------


		double minSpeed = 0;
		double maxSpeed = 60;

		double minAngle = 0;
		double maxAngle = 2 * PI;

		double speedDelta = maxSpeed - minSpeed;
		double angleDelta = maxAngle - minAngle;

		double randAngle = angleDelta*((double)std::rand() / (double)RAND_MAX) + minAngle;
		double randIntensity = speedDelta * ((double)std::rand() / (double)RAND_MAX) + minSpeed;

		tPosd randVelocity;
		randVelocity.x = randIntensity*cos(randAngle);
		randVelocity.y = randIntensity*sin(randAngle);

		return State(randPos, randVelocity);
	}
	
	CUDA_HOST
	static State gaussianRandomState(tTrackSeg* trackSegArray, int nTrackSegs, int initialSegIndex, int finalSegIndex, double velAngleBias, double velIntensityBias){

		//-------------------- random position calculation --------------------

		/**/

		double minXVertex = DBL_MAX;
		double maxXVertex = -1 * DBL_MAX;

		double minYVertex = DBL_MAX;
		double maxYVertex = -1 * DBL_MAX;


		//check if there are lower and higher bounds
		for (int i = initialSegIndex; i%nTrackSegs != finalSegIndex; i++){

			tTrackSeg currSeg = trackSegArray[i%nTrackSegs];

			for (int j = 0; j < 4; j++){
				if (currSeg.vertex[j].x < minXVertex){
					minXVertex = currSeg.vertex[j].x;
				}
				if (currSeg.vertex[j].x > maxXVertex){
					maxXVertex = currSeg.vertex[j].x;
				}

				if (currSeg.vertex[j].y < minYVertex){
					minYVertex = currSeg.vertex[j].y;
				}
				if (currSeg.vertex[j].y > maxYVertex){
					maxYVertex = currSeg.vertex[j].y;
				}
			}
			

			


		}

		double trackMapXMin = minXVertex;
		double trackMapXMax = maxXVertex;

		double trackMapXDelta = trackMapXMax - trackMapXMin;

		double trackMapYMin = minYVertex;
		double trackMapYMax = maxYVertex;

		double trackMapYDelta = trackMapYMax - trackMapYMin;

		tPosd randPos;
		randPos.x = trackMapXDelta * ((double)std::rand() / (double)RAND_MAX) + trackMapXMin;
		randPos.y = trackMapYDelta * ((double)std::rand() / (double)RAND_MAX) + trackMapYMin;

		/**/

		//-------------------- random velocity calculation --------------------


		double minSpeed = 0;
		double maxSpeed = 60;

		double minAngle = 0;
		double maxAngle = 2 * PI;

		double speedDelta = maxSpeed - minSpeed;
		double angleDelta = maxAngle - minAngle;

		double randAngle = randToNormalRand(velAngleBias, 1);
		double randIntensity = randToNormalRand(velIntensityBias, 1);

		tPosd randVelocity;
		randVelocity.x = randIntensity*cos(randAngle);
		randVelocity.y = randIntensity*sin(randAngle);

		return State(randPos, randVelocity);
	}

};

class ConstraintChecking{
public:
	CUDA_HOSTDEV
	static	bool validPoint(tTrackSeg* segArray, int nTrackSegs, State* target){
		return UtilityMethods::getSegmentOf(tTrackSeg(), segArray, nTrackSegs, target->getPos().x, target->getPos().y);
	}
};

class EvalFunctions{
public:
	CUDA_HOSTDEV
	static double evaluatePathCost(State* state){
		return UtilityMethods::SimpleGetDistanceFromStart(state->getLocalPos());
	}
	
};


class DeltaFunctions{

public:


	CUDA_HOSTDEV
	static double calcBezierLengthApprox(tPosd p0, tPosd p1, tPosd p2, tPosd p3, int nSamples){
		double length = 0;
		
		tPosd* samples = (tPosd*) malloc(sizeof(tPosd)*nSamples);
		
		double curvePercent = 0.0;
		for (int i = 0; i < nSamples; i++){
			double curvePercentInverse = 1 - curvePercent;
			samples[i].x = (curvePercentInverse*curvePercentInverse*curvePercentInverse) * p0.x +
				3 * curvePercent* (curvePercentInverse*curvePercentInverse) * p1.x +
				3 * curvePercent*curvePercent*curvePercentInverse * p2.x +
				curvePercent*curvePercent*curvePercent* p3.x;

			samples[i].y = (curvePercentInverse*curvePercentInverse*curvePercentInverse) *p0.y +
				3 * curvePercent* (curvePercentInverse*curvePercentInverse) * p1.y +
				3 * curvePercent*curvePercent*curvePercentInverse * p2.y +
				curvePercent*curvePercent*curvePercent* p3.y;
			if (i > 0){
				length += UtilityMethods::getEuclideanQuadranceBetween(samples[i], samples[i - 1]);
			}
			
			curvePercent += (1.0 / nSamples);
		}
		free(samples);

		return length;

	}

	//wrapper
	CUDA_HOSTDEV
	static bool applyBezierDelta(State& state, State& parent, tTrackSeg* trackSegArray, int nTrackSegs, double actionSimDeltaTime){
	
		tPosd newPos = tPosd();
		tPosd newSpeed = tPosd();


		tPosd p0 = parent.getPos();


		tPosd p1;
		p1.x = (parent.getPos().x + actionSimDeltaTime*parent.getVelocity().x);
		p1.y = (parent.getPos().y + actionSimDeltaTime*parent.getVelocity().y);
		State p1State = State(p1, parent.getVelocity());


		tPosd p2;
		p2.x = (state.getPos().x - actionSimDeltaTime*state.getVelocity().x);
		p2.y = (state.getPos().y - actionSimDeltaTime*state.getVelocity().y);
		State p2State = State(p2, state.getVelocity());


		tPosd p3 = state.getPos();

		//the bezier lies outside of the track
		if (!ConstraintChecking::validPoint(trackSegArray, nTrackSegs, &p1State) ||
			!ConstraintChecking::validPoint(trackSegArray, nTrackSegs, &p2State) //||
			//(p1.x / p1.x + p2.x / p2.x != 0 && p1.y / p1.y + p2.y / p2.y != 0)
			){
			return false;
		}

		double viLength = parent.getVelocity().x*parent.getVelocity().x + parent.getVelocity().y*parent.getVelocity().y;
		double bezierLength = calcBezierLengthApprox(p0,p1,p2,p3,10);

		double curvePercent = (actionSimDeltaTime*viLength) / (bezierLength);

		if (curvePercent > 0.4){
			curvePercent = 0.4;
		}
		//printf("curvePercent: %f\n", curvePercent);
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

		newPos.z = state.getPos().z;

		tPosd bezierTangent = tPosd();

		//the new speed is given by the derivative of the cubic bezier formula
		bezierTangent.x = (3 * curvePercentInverse*curvePercentInverse) *(p1.x - p0.x) +
			(6 * curvePercent*curvePercentInverse) * (p2.x - p1.x) +
			3 * curvePercent*curvePercent* (p3.x - p2.x);

		bezierTangent.y = (3 * curvePercentInverse*curvePercentInverse) *(p1.y - p0.y) +
			(6 * curvePercent*curvePercentInverse) * (p2.y - p1.y) +
			3 * curvePercent*curvePercent* (p3.y - p2.y);


		newSpeed.x = bezierTangent.x;
		newSpeed.y = bezierTangent.y;

		state.posRand = state.getPos();
		state.speedRand = state.getVelocity();

		state.setCommands(newPos, newSpeed);

		//if the control points are inside but the middle point is out (that can supposidely happen)
		if (!ConstraintChecking::validPoint(trackSegArray, nTrackSegs, &state)){
			return false;
		}

		return true;

	}

	CUDA_HOSTDEV
	static bool applyPhysicsDelta(State& state, State& parent, tTrackSeg* trackSegArray, int nTrackSegs, double actionSimDeltaTime){
		
		double angle0 = atan2f(parent.getVelocity().y, parent.getVelocity().x);;
		double angleF = atan2f(state.getVelocity().y , state.getVelocity().x);;

		double intensity0 = sqrt(parent.getVelocity().x*parent.getVelocity().x + parent.getVelocity().y*parent.getVelocity().y);
		double intensityF = sqrt(state.getVelocity().x*state.getVelocity().x + state.getVelocity().y*state.getVelocity().y);
		
		
		tPosd p_i_1 = parent.getPos();
		tPosd p_i;

		tPosd pDelta;
		tPosd vDelta;

		int deltaI = 2;
		double k = 10;

		double inc = 1 / k;

		for (double i = inc; i < 1.0; i += inc){
		
			double angle_i = angle0 * (1 - inc) + angleF * inc;
			double intensity_i = intensity0*(1 - inc) + intensityF*inc;

			tPosd v_i;
			v_i.x = intensity_i*cos(angle_i);
			v_i.y = intensity_i*sin(angle_i);

			p_i.x = p_i_1.x + v_i.x*inc*actionSimDeltaTime;
			p_i.y = p_i_1.y + v_i.y*inc*actionSimDeltaTime;


			p_i_1 = p_i;

			if (i*k == (double) deltaI){
				pDelta = p_i;
				vDelta = v_i;
			}
		}

		state.setPos({ pDelta.x, pDelta.y });
		state.setVelocity({ vDelta.x, vDelta.y });
		
		if (!ConstraintChecking::validPoint(trackSegArray, nTrackSegs, &state)){
			return false;
		}
		return true;
	}


	CUDA_HOSTDEV
	static bool applyDelta(State& state, State& parent, tTrackSeg* trackSegArray, int nTrackSegs, double actionSimDeltaTime){
		return applyPhysicsDelta(state, parent, trackSegArray, nTrackSegs, actionSimDeltaTime);
	}

};

#endif