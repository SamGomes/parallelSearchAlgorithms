/***************************************************************************

file                 : driver.cpp
created              : Thu Dec 20 01:21:49 CET 2002
copyright            : (C) 2002-2004 Bernhard Wymann
email                : berniw@bluewin.ch
version              : $Id: driver.cpp,v 1.16.2.2 2008/12/31 03:53:53 berniw Exp $

***************************************************************************/

/***************************************************************************
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU General Public License as published by  *
*   the Free Software Foundation; either version 2 of the License, or     *
*   (at your option) any later version.                                   *
*                                                                         *
***************************************************************************/

#include "driver.h"


const float Driver::SHIFT = 0.9f;							// [-] (% of rpmredline) When do we like to shift gears.
const float Driver::SHIFT_MARGIN = 4.0f;					// [m/s] Avoid oscillating gear changes.
const float Driver::CLUTCH_SPEED = 5.0f;					// [m/s]
const float Driver::CLUTCH_FULL_MAX_TIME = 2.0f;			// [s] Time to apply full clutch.

// Static variables.
Cardata *Driver::cardata = NULL;
double Driver::currentsimtime;



Driver::Driver(int index)
{
	INDEX = index;
}


Driver::~Driver()
{
	delete opponents;
	delete[] radius;
	if (cardata != NULL) {
		delete cardata;
		cardata = NULL;
	}
	delete mycardata;
}


// Called for every track change or new race.
void Driver::initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s)
{
	track = t;

	const int BUFSIZE = 256;
	char buffer[BUFSIZE];
	


	*carParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);
	if (*carParmHandle == NULL) {
		snprintf(buffer, BUFSIZE, "drivers/bt/%d/default.xml", INDEX);
		*carParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);
	}


	// Load and set parameters.
	MU_FACTOR = GfParmGetNum(*carParmHandle, BT_SECT_PRIV, BT_ATT_MUFACTOR, (char*)NULL, 0.69f);
}


// Start a new race.
void Driver::newRace(tCarElt* car, tSituation *s)
{
	float deltaTime = (float)RCM_MAX_DT_ROBOTS;
	this->car = car;
	clutchtime = 0.0f;

	// Create just one instance of cardata shared by all drivers.
	if (cardata == NULL) {
		cardata = new Cardata(s);
	}
	mycardata = cardata->findCar(car);
	currentsimtime = s->currentTime;

	// initialize the list of opponents.
	opponents = new Opponents(s, this, cardata);
	opponent = opponents->getOpponentPtr();


	// Initialize radius of segments.
	radius = new float[track->nseg];
	computeRadius(radius);


}

bool Driver::validPoint(tPosd target){
	//point on oponent?

	if (target.x <= (opponent->getCarPtr()->_pos_X + 20) &&
		target.x >= (opponent->getCarPtr()->_pos_X - 20) &&
		target.y <= (opponent->getCarPtr()->_pos_Y + 20) &&
		target.y >= (opponent->getCarPtr()->_pos_Y - 20)){
			return false;

	}
	//point outside track?
	tTrkLocPos pos;
	tTrackSeg* seg = track->seg;
	tTrackSeg* currSeg = seg->next;
	while (currSeg != seg){
		RtTrackGlobal2Local(currSeg, target.x, target.y, &pos, TR_LPOS_MAIN);
		if (pos.toRight > 0 && pos.toLeft > 0){
			return true;
		}

		currSeg = currSeg->next;
	}
	return false;
}


void Driver::seek(tPosd target){

	tdble carX = car->_pos_X;
	tdble carY = car->_pos_Y;
	tdble carZ = car->_pos_Z;
	tdble targetX = target.x;
	tdble targetY = target.y;
	tdble targetZ = target.z;

	float targetAngle;


	targetAngle = atan2(targetY - carY, targetX - carX);
	targetAngle -= car->_yaw;
	NORM_PI_PI(targetAngle);

	if (abs(targetAngle) > car->_steerLock){
		car->_accelCmd = 0.5;
		car->_gearCmd = -1;
		targetAngle = car->_steerLock * -(targetAngle / abs(targetAngle));
	}

	car->_steerCmd = targetAngle / car->_steerLock;
	//car->_steerCmd = currState.getSteerAngle();
	car->_accelCmd = currState.getPedalPos() > 0 ? currState.getPedalPos() : 0;
	car->_brakeCmd = currState.getPedalPos() < 0 ? -1.0*currState.getPedalPos() : 0;
	car->_gearCmd = getGear();
}



// Drive during race.
void Driver::drive(tSituation *s)
{
	memset(&car->ctrl, 0, sizeof(tCarCtrl));
	update(s);
}



/***************************************************************************
*
* utility functions
*
***************************************************************************/



void Driver::computeRadius(float *radius)
{
	float lastturnarc = 0.0f;
	int lastsegtype = TR_STR;

	tTrackSeg *currentseg, *startseg = track->seg;
	currentseg = startseg;

	do {
		if (currentseg->type == TR_STR) {
			lastsegtype = TR_STR;
			radius[currentseg->id] = FLT_MAX;
		}
		else {
			if (currentseg->type != lastsegtype) {
				float arc = 0.0f;
				tTrackSeg *s = currentseg;
				lastsegtype = currentseg->type;

				while (s->type == lastsegtype && arc < PI / 2.0f) {
					arc += s->arc;
					s = s->next;
				}
				lastturnarc = arc / (PI / 2.0f);
			}
			radius[currentseg->id] = (currentseg->radius + currentseg->width / 2.0) / lastturnarc;
		}
		currentseg = currentseg->next;
	} while (currentseg != startseg);

}

// Compute gear.
int Driver::getGear()
{
	if (car->_gear <= 0) {
		return 1;
	}
	float gr_up = car->_gearRatio[car->_gear + car->_gearOffset];
	float omega = (car->_enginerpmRedLine*0.5) / gr_up;
	float wr = car->_wheelRadius(2);

	if (omega*wr*SHIFT < car->_speed_x) {
		return car->_gear + 1;
	}
	else {
		float gr_down = car->_gearRatio[car->_gear + car->_gearOffset - 1];
		omega = (car->_enginerpmRedLine*0.5) / gr_down;
		if (car->_gear > 1 && omega*wr*SHIFT > car->_speed_x + SHIFT_MARGIN) {
			return car->_gear - 1;
		}
	}
	return car->_gear;
}


// Compute steer value.
float Driver::getSteer(tPosd target)
{
	double targetAngle = atan2(target.y - car->_pos_Y, target.x - car->_pos_X);
	targetAngle -= car->_yaw;
	NORM_PI_PI(targetAngle);
	return targetAngle / car->_steerLock;
}


// Compute the clutch value.
float Driver::getClutch()
{
	if (car->_gear > 1) {
		clutchtime = 0.0f;
		return 0.0f;
	}
	else {
		float drpm = car->_enginerpm - car->_enginerpmRedLine / 2.0f;
		clutchtime = MIN(CLUTCH_FULL_MAX_TIME, clutchtime);
		float clutcht = (CLUTCH_FULL_MAX_TIME - clutchtime) / CLUTCH_FULL_MAX_TIME;
		if (car->_gear == 1 && car->_accelCmd > 0.0f) {
			clutchtime += (float)RCM_MAX_DT_ROBOTS;
		}

		if (drpm > 0) {
			float speedr;
			if (car->_gearCmd == 1) {
				// Compute corresponding speed to engine rpm.
				float omega = car->_enginerpmRedLine / car->_gearRatio[car->_gear + car->_gearOffset];
				float wr = car->_wheelRadius(2);
				speedr = (CLUTCH_SPEED + MAX(0.0f, car->_speed_x)) / fabs(wr*omega);
				float clutchr = MAX(0.0f, (1.0f - speedr*2.0f*drpm / car->_enginerpmRedLine));
				return MIN(clutcht, clutchr);
			}
			else {
				// For the reverse gear.
				clutchtime = 0.0f;
				return 0.0f;
			}
		}
		else {
			return clutcht;
		}
	}
}

// Update my private data every timestep.
void Driver::update(tSituation *s)
{
	// Update global car data (shared by all instances) just once per timestep.

	if (currentsimtime != s->currentTime) {
		currentsimtime = s->currentTime;
		cardata->update();


		if (delay == 0){
			SeqRRTStar RRTStar = SeqRRTStar(new State(), 50);
			path = RRTStar.search();


		}
		
		if (delay == 100){
			/*currState = cuda_search(State())[0];
			std::cout << "B ou Smol? " << currState.getPedalPos() << " , " << currState.getSteerAngle() << std::endl;*/


			//std::cout << "bauauauauyeye...: \n" << std::endl;


			//for (std::vector<State>::iterator i = path.begin(); i != path.end(); ++i)
			//	std::cout << i->toString() << ' ' << std::endl;




			currState = path[(currPoint++)%path.size()];

			std::cout << "curr...:" << currState.toString() << std::endl;


			delay = 1;
		}
		else{

			//tPosd otherPos;

			//otherPos.x = opponent->getCarPtr()->_pos_X;
			//otherPos.y = opponent->getCarPtr()->_pos_Y;
			//otherPos.z = opponent->getCarPtr()->_pos_Z;

			////printf("------%d------\n", delay);
			//this->seek(otherPos);

			car->_steerCmd = currState.getSteerAngle();
			car->_accelCmd = currState.getPedalPos() > 0 ? currState.getPedalPos() : 0;
			car->_brakeCmd = currState.getPedalPos() < 0 ? -1.0*currState.getPedalPos() : 0;
			car->_gearCmd = getGear();

			delay++;
		}
	}

}





