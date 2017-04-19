#pragma once

#ifndef HEURISTICS_H
#define HEURISTICS_H

#include "State.cuh"
#include <robottools.h>
#include <curand.h>
#include <curand_kernel.h>
#define MAX_CAR_SPEED_CONSIDERED 80

//(for CUDA)
#define PI 3.14159265358979323846

class UtilityMethods{
public:

	CUDA_HOSTDEV //the nearest point is the one in which its finalPos prediction ajusts to the current pos
	static State nearestNeighbor(State state, State* graph, int graphSize){

		State closestState;
		double minDist = DBL_MAX;

		
		for (int i = 0; i < graphSize; i++){
			State currState = graph[i];
			if (currState.getMyGraphIndex() == -1 || state.getMyGraphIndex()==i)  //still unassigned
				continue;
			double dist = UtilityMethods::getPolarQuadranceBetween(state.getVelocity(), graph[i].getVelocity());
			if (dist <= minDist){
				minDist = dist;
				closestState = currState;
			}
		}
		return closestState;
	}


	CUDA_HOSTDEV
	static bool SimpleRtTrackGlobal2Local(tStateRelPos* p, tTrackSeg* segmentArray, int nTrackSegs, tdble X, tdble Y, int type)
	{

		int 	segnotfound = 1;
		float 	x, y;

		tTrackSeg* seg = getSegmentOf(segmentArray, nTrackSegs, X, Y);
		
		if (seg==nullptr){
			return false;
		}

		int segArrayIterator = seg->id;
		float 	theta, a2;
		int 	depl = 0;
		p->type = type;
		p->segId = segArrayIterator;



		while (segnotfound) {

			switch (seg->type) {
				case 3:
					/* rotation */
					float sine, cosine;
					float ts;
					sine = sin(seg->angle[0]);
					cosine = cos(seg->angle[0]);
					x = X - seg->vertex[1].x;
					y = Y - seg->vertex[1].y;
					ts = x * cosine + y * sine;
					p->segId = segArrayIterator;
					p->toStart = ts;
					p->toRight = y * cosine - x * sine;
					if ((ts < 0) && (depl < 1)) {
						/* get back */
						segArrayIterator--;
						segArrayIterator = (segArrayIterator <0) ? nTrackSegs - 1 : segArrayIterator;
						seg = &segmentArray[segArrayIterator];
						depl = -1;
					}
					else if ((ts > seg->length) && (depl > -1)) {
						segArrayIterator++;
						segArrayIterator = (segArrayIterator >(nTrackSegs - 1)) ? 0 : segArrayIterator;
						seg = &segmentArray[segArrayIterator];
						depl = 1;
					}
					else {
						segnotfound = 0;
					}
					break;

				case 2:
					/* rectangular to polar */
					x = X - seg->center.x;
					y = Y - seg->center.y;
					a2 = seg->arc / 2.0;
					theta = atan2(y, x) - (seg->angle[6] + a2);
					theta = normPI_PI(theta);
					p->segId = segArrayIterator;
					p->toStart = theta + a2;
					p->toRight = seg->radiusr - sqrt(x*x + y*y);
					
					if ((theta < -a2) && (depl < 1)) {
						segArrayIterator--;
						segArrayIterator = (segArrayIterator <0) ? nTrackSegs - 1 : segArrayIterator;
						seg = &segmentArray[segArrayIterator];
						depl = -1;
					}
					else if ((theta > a2) && (depl > -1)) {
						segArrayIterator++;
						segArrayIterator = (segArrayIterator >(nTrackSegs - 1)) ? 0 : segArrayIterator;
						seg = &segmentArray[segArrayIterator];
						depl = 1;
					}
					else {
						segnotfound = 0;
					}
					break;

				case 1:
					/* rectangular to polar */
					x = X - seg->center.x;
					y = Y - seg->center.y;
					a2 = seg->arc / 2.0;
					theta = seg->angle[6] - a2 - atan2(y, x);
					theta = normPI_PI(theta);
					p->segId = segArrayIterator;
					p->toStart = theta + a2;
					p->toRight = sqrt(x*x + y*y) - seg->radiusr;
					
					if ((theta < -a2) && (depl < 1)) {
						segArrayIterator--;
						segArrayIterator = (segArrayIterator <0) ? nTrackSegs - 1 : segArrayIterator;
						seg = &segmentArray[segArrayIterator];
						depl = -1;
					}
					else if ((theta > a2) && (depl > -1)) {
						segArrayIterator++;
						segArrayIterator = (segArrayIterator >(nTrackSegs - 1)) ? 0 : segArrayIterator;
						seg = &segmentArray[segArrayIterator];
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
		p->toMiddle = p->toRight - seg->width / 2.0;
		p->toLeft = seg->width - p->toRight;

		return true;

	}


	//this returns a close approximation of the current segment that can be used to find the local pos (because of the floating point errors)
	CUDA_HOSTDEV
	static tTrackSeg* getSegmentOf(tTrackSeg* segmentArray, int nTrackSegs, tdble x, tdble y){
		tTrackSeg* seg = nullptr;
		
		for (int i = 0; i <= nTrackSegs; i++){

			t3Dd* bounds = segmentArray[i].vertex;

			double dist03 = getEuclideanQuadranceBetween({ bounds[0].x, bounds[0].y, bounds[0].z }, { bounds[3].x, bounds[3].y, bounds[3].z });
			double dist12 = getEuclideanQuadranceBetween({ bounds[1].x, bounds[1].y, bounds[1].z }, { bounds[2].x, bounds[2].y, bounds[2].z });

			double distP0 = getEuclideanQuadranceBetween({ x, y, 0 }, { bounds[0].x, bounds[0].y, bounds[0].z });
			double distP1 = getEuclideanQuadranceBetween({ x, y, 0 }, { bounds[1].x, bounds[1].y, bounds[1].z });
			double distP2 = getEuclideanQuadranceBetween({ x, y, 0 }, { bounds[2].x, bounds[2].y, bounds[2].z });
			double distP3 = getEuclideanQuadranceBetween({ x, y, 0 }, { bounds[3].x, bounds[3].y, bounds[3].z });

			double a1 = distP0 / dist03;
			double a2 = distP3 / dist03;
			double a3 = distP1 / dist12;
			double a4 = distP2 / dist12;

			
			if ((a1<1) && (a2<1) && (a3<1) && (a4<1)){
				seg = &segmentArray[i];
			}
		}
		return seg;
	}

	CUDA_HOSTDEV
	static double getTrackCenterDistanceBetween(tTrackSeg* segmentArray, int nTrackSegs, State* s2, State* s1, int fwdLimit){

		tStateRelPos l1, l2;

		l1 = s1->getLocalPos();
		l2 = s2->getLocalPos();


		tTrackSeg	l1Seg = segmentArray[l1.segId];
		tTrackSeg	l2Seg = segmentArray[l2.segId];


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
		while ((angle) > PI) { (angle) -= 2 * PI; }
		while ((angle) < -PI) { (angle) += 2 * PI; }
		return angle;
	}

	CUDA_HOSTDEV
	static double mod(double num){
		return num > 0 ? num : -num;
	}

	CUDA_HOSTDEV
	static double norm0_2PI(double num) {									
		while ((num) > 2 * PI) {
			(num) -= 2 * PI; 
		}
		while ((num) < 0) {
			(num) += 2 * PI; 
		}
		return num;
	}


	//calculates the minimal signed rotation angle between arbitrary angles
	CUDA_HOSTDEV
	static double rotationBetween(double first, double second)
	{

		double diff = second - first;

		diff = norm0_2PI(diff) + 3 * PI;

		diff = norm0_2PI(diff);

		diff -= PI;

		return diff;
	}


	//calculates euclidean distance between points
	CUDA_HOSTDEV
	static double getEuclideanQuadranceBetween(tPosd p1, tPosd p2){
		return (p2.x - p1.x)*(p2.x - p1.x) + (p2.y - p1.y)*(p2.y - p1.y);
	}

	//calculates polar distance between points
	CUDA_HOSTDEV
	static double getPolarQuadranceBetween(tPolarVel p1, tPolarVel p2){

	/*	double w1 = 0.5;
		double w2 = 0.5;*/

		//get shortest angle

		//velocity: 0 .. 80 (quad: 0 ..1600)
		//angle 0 .. PI


		double deltaAngle = rotationBetween(p1.angle, p2.angle);


		//double deltaIntesity = p2.intensity - p1.intensity;
		return deltaAngle/PI;
	}

	CUDA_HOSTDEV
	static double getDotBetween(tPosd p1, tPosd p2){
		return p1.x*p2.x + p1.y*p2.y;
	}

};


class RandomStateGenerators{

public:
	CUDA_HOSTDEV
	static double generateRandomNumber0_1(void* curandState){
		#ifdef __CUDA_ARCH__
			return curand_uniform((curandState_t*) curandState);
		#else
			return (double)std::rand() / (double)RAND_MAX;
		#endif
	}


	CUDA_HOSTDEV
	static double randToNormalRand(double mean, double stddev, void* curandState)
	{   
		//Box muller method
		double n2 = 0.0;
		int n2_cached = 0;
		if (!n2_cached)
		{
			double x, y, r;
			do
			{
				x = 2.0*generateRandomNumber0_1(curandState) - 1;
				y = 2.0*generateRandomNumber0_1(curandState) - 1;

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

	CUDA_HOSTDEV
	static State uniformRandomState(tTrackSeg* trackSegArray, int nTrackSegs, void* curandState){

		//-------------------- random velocity calculation --------------------


		double minSpeed = 0;
		double maxSpeed = 80;

		double minAngle = -PI;
		double maxAngle = PI;

		double speedDelta = maxSpeed - minSpeed;
		double angleDelta = maxAngle - minAngle;

		double randAngle = angleDelta*generateRandomNumber0_1(curandState) + minAngle;
		double randIntensity = speedDelta * generateRandomNumber0_1(curandState) + minSpeed;

		tPolarVel randVelocity;
		randVelocity.angle = randAngle;
		randVelocity.intensity = randIntensity;

		return State(randVelocity);
	}
	
	CUDA_HOSTDEV
	static State gaussianRandomState(tTrackSeg* trackSegArray, int nTrackSegs, tPolarVel velAngleBias, void* curandState){


		//-------------------- random velocity calculation --------------------


		double minSpeed = 0;
		double maxSpeed = 100;

		/*double minAngle = 0;
		double maxAngle = 2*PI;*/

		double speedDelta = maxSpeed - minSpeed;

		double randAngle = randToNormalRand(velAngleBias.angle, 10, curandState);
		//double randIntensity = randToNormalRand(velIntensityBias, 1);
		double randIntensity = speedDelta * generateRandomNumber0_1(curandState) + minSpeed;


		tPolarVel randVelocity;
		randVelocity.angle = randAngle;
		randVelocity.intensity = randIntensity;

		return State(randVelocity);
	}

};

class ConstraintChecking{
public:
	CUDA_HOSTDEV
	static	bool validPoint(tTrackSeg* segArray, int nTrackSegs, State* target){
		return (UtilityMethods::getSegmentOf(segArray, nTrackSegs, target->getPos().x, target->getPos().y)!=nullptr);
	}
};

class Interpolations{

public:
	CUDA_HOSTDEV
	static double linearInterpolation(double p0, double p1, double t){
		return p0 * (1 - t) + p1 * t;
	}

	CUDA_HOSTDEV
	static double cosineInterpolation(double p0, double p1, double t){
		t = (1 - cos(t*PI)) / 2;
		return(p0*(1 - t) + p1*t);
	}

	CUDA_HOSTDEV
	static double exponentialInterpolation(double p0, double p1, double t, double a){
		t = (pow(t, a) - 1) / (a - 1);
		return p0 * (1 - t) + p1 * t;
	}

	CUDA_HOSTDEV
	static double logaritmicInterpolation(double p0, double p1, double t, double a){
		t = t / (t + a);
		return p0 * (1 - t) + p1 * t;
	}

	CUDA_HOSTDEV
	static float angleLinearInterpolation(double from, double to, double t)
	{
		double rotation = UtilityMethods::rotationBetween(from, to);

		double a = from * (1 - t) + (from + rotation) * t;

		a = UtilityMethods::normPI_PI(a);

		return a;
	}

	CUDA_HOSTDEV
	static float angleLogaritmicInterpolation(double from, double to, double t, double a)
	{
		t = t / (t + a);
		if (abs(to - from) > PI){
			return from * (1 - t) - to * t;
		}
		return from * (1 - t) + to * t;
	}

};

class DeltaFunctions{

public:

	//-------------bezier based forward model (for future work maybe)----------------------

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

	CUDA_HOSTDEV
	static bool applyBezierDelta(State* state, State* parent, tTrackSeg* trackSegArray, int nTrackSegs, double actionSimDeltaTime){
	
		tPosd newPos = tPosd();
		tPolarVel newSpeed = tPolarVel();


		tPosd cartesianStateVel;
		cartesianStateVel.x = state->getVelocity().intensity*cos(state->getVelocity().angle);
		cartesianStateVel.y = state->getVelocity().intensity*sin(state->getVelocity().angle);

		tPosd cartesianParentVel;
		cartesianParentVel.x = parent->getVelocity().intensity*cos(parent->getVelocity().angle);
		cartesianParentVel.y = parent->getVelocity().intensity*sin(parent->getVelocity().angle);


		tPosd p0 = parent->getPos();


		tPosd p1;
		p1.x = (parent->getPos().x + actionSimDeltaTime*cartesianParentVel.x);
		p1.y = (parent->getPos().y + actionSimDeltaTime*cartesianParentVel.y);
		State p1State = State();
		p1State.setPos(p1);


		tPosd p2;
		p2.x = (state->getPos().x - actionSimDeltaTime*cartesianStateVel.x);
		p2.y = (state->getPos().y - actionSimDeltaTime*cartesianStateVel.y);
		State p2State = State();
		p2State.setPos(p2);


		tPosd p3 = state->getPos();

		//the bezier lies outside of the track
		if (!ConstraintChecking::validPoint(trackSegArray, nTrackSegs, &p1State) ||
			!ConstraintChecking::validPoint(trackSegArray, nTrackSegs, &p2State) //||
			//(p1.x / p1.x + p2.x / p2.x != 0 && p1.y / p1.y + p2.y / p2.y != 0)
			){
			return false;
		}

		double viLength = cartesianParentVel.x*cartesianParentVel.x + cartesianParentVel.y*cartesianParentVel.y;
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

		newPos.z = state->getPos().z;

		tPosd bezierTangent = tPosd();

		//the new speed is given by the derivative of the cubic bezier formula
		bezierTangent.x = (3 * curvePercentInverse*curvePercentInverse) *(p1.x - p0.x) +
			(6 * curvePercent*curvePercentInverse) * (p2.x - p1.x) +
			3 * curvePercent*curvePercent* (p3.x - p2.x);

		bezierTangent.y = (3 * curvePercentInverse*curvePercentInverse) *(p1.y - p0.y) +
			(6 * curvePercent*curvePercentInverse) * (p2.y - p1.y) +
			3 * curvePercent*curvePercent* (p3.y - p2.y);


		newSpeed.angle = atan2(bezierTangent.y,bezierTangent.x);
		newSpeed.intensity = sqrt(bezierTangent.x*bezierTangent.x + bezierTangent.y*bezierTangent.y);

		state->posRand = state->getPos();
		state->speedRand = cartesianStateVel;

		state->setPos(newPos);
		state->setVelocity(newSpeed);

		//if the control points are inside but the middle point is out (that can supposidely happen)
		if (!ConstraintChecking::validPoint(trackSegArray, nTrackSegs, state)){
			return false;
		}

		return true;

	}


	//--------------------used physics based forward model----------------------------------

	CUDA_HOSTDEV
	static bool applyPhysicsDelta(State* state, State* parent, tTrackSeg* trackSegArray, int nTrackSegs, double actionSimDeltaTime){
		
		double angle0 = parent->getVelocity().angle;
		double angleF = state->getVelocity().angle;

		double intensity0 = parent->getVelocity().intensity;
		double intensityF = state->getVelocity().intensity;

		tPosd p_i_1 = parent->getPos();
		tPosd p_i;

		tPosd pDelta;
		tPolarVel vDelta;

		int deltaI = 5;
		double k = 20;

		double inc = (double) 1.0 / k;

		//divide the pathe between the two states and pick the state on the delta variation
		for (int i = 0; i < k; i ++){
				
			double angle_i = Interpolations::angleLinearInterpolation(angle0, angleF, inc);
			double intensity_i = Interpolations::logaritmicInterpolation(intensity0, intensityF, inc,0.03);

			tPolarVel v_i;
			v_i.angle = angle_i;
			v_i.intensity = intensity_i;

			tPosd cartesianV_i;
			cartesianV_i.x = v_i.intensity*cos(v_i.angle);
			cartesianV_i.y = v_i.intensity*sin(v_i.angle);

			p_i.x = p_i_1.x + cartesianV_i.x*inc*actionSimDeltaTime;
			p_i.y = p_i_1.y + cartesianV_i.y*inc*actionSimDeltaTime;

			State aux = State(p_i, v_i);

			//if one point in the path is not valid, the path is discarded
			if (!ConstraintChecking::validPoint(trackSegArray, nTrackSegs, &aux)){
				return false;
			}
			p_i_1 = p_i;
			if (i == deltaI){
				pDelta = p_i;
				vDelta = v_i;
			}
		}
		state->setPos(pDelta);
		state->setVelocity(vDelta);

		return true;
	}

	//--------------------------------wrapper--------------------------------
	CUDA_HOSTDEV
	static bool applyDelta(State* state, State* parent, tTrackSeg* trackSegArray, int nTrackSegs, double actionSimDeltaTime){
		return applyPhysicsDelta(state, parent, trackSegArray, nTrackSegs, actionSimDeltaTime);
	}

};

#endif