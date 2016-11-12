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

const float Driver::MAX_UNSTUCK_ANGLE = 15.0f / 180.0f*PI;	// [radians] If the angle of the car on the track is smaller, we assume we are not stuck.
const float Driver::UNSTUCK_TIME_LIMIT = 2.0f;				// [s] We try to get unstuck after this time.
const float Driver::MAX_UNSTUCK_SPEED = 5.0f;				// [m/s] Below this speed we consider being stuck.
const float Driver::MIN_UNSTUCK_DIST = 3.0f;				// [m] If we are closer to the middle we assume to be not stuck.
const float Driver::G = 9.81f;								// [m/(s*s)] Welcome on Earth.
const float Driver::FULL_ACCEL_MARGIN = 1.0f;				// [m/s] Margin reduce oscillation of brake/acceleration.
const float Driver::SHIFT = 0.9f;							// [-] (% of rpmredline) When do we like to shift gears.
const float Driver::SHIFT_MARGIN = 4.0f;					// [m/s] Avoid oscillating gear changes.
const float Driver::ABS_SLIP = 2.0f;						// [m/s] range [0..10]
const float Driver::ABS_RANGE = 5.0f;						// [m/s] range [0..10]
const float Driver::ABS_MINSPEED = 3.0f;					// [m/s] Below this speed the ABS is disabled (numeric, division by small numbers).
const float Driver::TCL_SLIP = 2.0f;						// [m/s] range [0..10]
const float Driver::TCL_RANGE = 10.0f;						// [m/s] range [0..10]
const float Driver::LOOKAHEAD_CONST = 17.0f;				// [m]
const float Driver::LOOKAHEAD_FACTOR = 0.33f;				// [-]
const float Driver::WIDTHDIV = 3.0f;						// [-] Defines the percentage of the track to use (2/WIDTHDIV).
const float Driver::SIDECOLL_MARGIN = 3.0f;					// [m] Distance between car centers to avoid side collisions.
const float Driver::BORDER_OVERTAKE_MARGIN = 0.5f;			// [m]
const float Driver::OVERTAKE_OFFSET_SPEED = 5.0f;			// [m/s] Offset change speed.
const float Driver::PIT_LOOKAHEAD = 6.0f;					// [m] Lookahead to stop in the pit.
const float Driver::PIT_BRAKE_AHEAD = 200.0f;				// [m] Workaround for "broken" pitentries.
const float Driver::PIT_MU = 0.4f;							// [-] Friction of pit concrete.
const float Driver::MAX_SPEED = 84.0f;						// [m/s] Speed to compute the percentage of brake to apply.
const float Driver::MAX_FUEL_PER_METER = 0.0008f;			// [liter/m] fuel consumtion.
const float Driver::CLUTCH_SPEED = 5.0f;					// [m/s]
const float Driver::CENTERDIV = 0.1f;						// [-] (factor) [0.01..0.6].
const float Driver::DISTCUTOFF = 200.0f;					// [m] How far to look, terminate while loops.
const float Driver::MAX_INC_FACTOR = 5.0f;					// [m] Increment faster if speed is slow [1.0..10.0].
const float Driver::CATCH_FACTOR = 10.0f;					// [-] select MIN(catchdist, dist*CATCH_FACTOR) to overtake.
const float Driver::CLUTCH_FULL_MAX_TIME = 2.0f;			// [s] Time to apply full clutch.
const float Driver::USE_LEARNED_OFFSET_RANGE = 0.2f;		// [m] if offset < this use the learned stuff

const float Driver::TEAM_REAR_DIST = 50.0f;					//
const int Driver::TEAM_DAMAGE_CHANGE_LEAD = 700;			// When to change position in the team?

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
	delete pit;
	delete[] radius;
	delete learn;
	delete strategy;
	if (cardata != NULL) {
		delete cardata;
		cardata = NULL;
	}
}


// Called for every track change or new race.
void Driver::initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s)
{
	track = t;

	const int BUFSIZE = 256;
	char buffer[BUFSIZE];
	// Load a custom setup if one is available.
	// Get a pointer to the first char of the track filename.
	char* trackname = strrchr(track->filename, '/') + 1;

	switch (s->_raceType) {
	case RM_TYPE_PRACTICE:
		snprintf(buffer, BUFSIZE, "drivers/bt/%d/practice/%s", INDEX, trackname);
		break;
	case RM_TYPE_QUALIF:
		snprintf(buffer, BUFSIZE, "drivers/bt/%d/qualifying/%s", INDEX, trackname);
		break;
	case RM_TYPE_RACE:
		snprintf(buffer, BUFSIZE, "drivers/bt/%d/race/%s", INDEX, trackname);
		break;
	default:
		break;
	}

	*carParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);
	if (*carParmHandle == NULL) {
		snprintf(buffer, BUFSIZE, "drivers/bt/%d/default.xml", INDEX);
		*carParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);
	}

	// Create a pit stop strategy object.
	strategy = new SimpleStrategy2();

	// Init fuel.
	strategy->setFuelAtRaceStart(t, carParmHandle, s, INDEX);

	// Load and set parameters.
	MU_FACTOR = GfParmGetNum(*carParmHandle, BT_SECT_PRIV, BT_ATT_MUFACTOR, (char*)NULL, 0.69f);
}


// Start a new race.
void Driver::newRace(tCarElt* car, tSituation *s)
{
	float deltaTime = (float)RCM_MAX_DT_ROBOTS;
	MAX_UNSTUCK_COUNT = int(UNSTUCK_TIME_LIMIT / deltaTime);
	OVERTAKE_OFFSET_INC = OVERTAKE_OFFSET_SPEED*deltaTime;
	stuck = 0;
	alone = 1;
	clutchtime = 0.0f;
	oldlookahead = 0.0f;
	this->car = car;
	CARMASS = GfParmGetNum(car->_carHandle, SECT_CAR, PRM_MASS, NULL, 1000.0f);
	myoffset = 0.0f;
	initCa();
	initCw();
	initTireMu();
	initTCLfilter();

	// Create just one instance of cardata shared by all drivers.
	if (cardata == NULL) {
		cardata = new Cardata(s);
	}
	mycardata = cardata->findCar(car);
	currentsimtime = s->currentTime;

	// initialize the list of opponents.
	opponents = new Opponents(s, this, cardata);
	opponent = opponents->getOpponentPtr();

	// Set team mate.
	const char *teammate = GfParmGetStr(car->_carHandle, BT_SECT_PRIV, BT_ATT_TEAMMATE, NULL);
	if (teammate != NULL) {
		opponents->setTeamMate(teammate);
	}

	// Initialize radius of segments.
	radius = new float[track->nseg];
	computeRadius(radius);

	learn = new SegLearn(track, s, INDEX);

	// create the pit object.
	pit = new Pit(s, this);


}
int currPoint = 0;

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

State currState;

void Driver::seek(tPosd target){

	tdble carX = car->_pos_X;
	tdble carY = car->_pos_Y;
	tdble carZ = car->_pos_Z;
	tdble targetX = target.x;
	tdble targetY = target.y;
	tdble targetZ = target.z;



	float targetAngle;


	/*car->_accelCmd = currState.getPedalPos() > 0 ? currState.getPedalPos() : 0;
	car->_brakeCmd = currState.getPedalPos() < 0 ? -1.0*currState.getPedalPos() : 0;
	car->_gearCmd = getGear();
	car->_steerCmd = currState.getSteerAngle();*/

	/*if ((car->_speed_X + car->_speed_Y) / 2 > 50){
	car->_accelCmd = 0.0f;
	car->_brakeCmd = 0.5f;
	return;
	}


	car->_accelCmd = 0.7f;
	car->_gearCmd = getGear();*/

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

	//pit->setPitstop(true);


}







/***************************************************************************
*
* utility functions
*
***************************************************************************/

// Set pitstop commands.
int Driver::pitCommand(tSituation *s)
{
	return 0;
}
// End of the current race.
void Driver::endRace(tSituation *s)
{
	// Nothing for now.
}


// Compute target point for steering.
vec2f Driver::getTargetPoint()
{
	return vec2f();// Nothing for now.
}

int Driver::isAlone()
{
	int i;
	for (i = 0; i < opponents->getNOpponents(); i++) {
		if (opponent[i].getState() & (OPP_COLL | OPP_LETPASS)) {
			return 0;	// Not alone.
		}
	}
	return 1;	// Alone.
}

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
float Driver::getSteer()
{
	float targetAngle;
	vec2f target = getTargetPoint();

	targetAngle = atan2(target.y - car->_pos_Y, target.x - car->_pos_X);
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

std::vector<State> path;
int delay = 0;
// Update my private data every timestep.
void Driver::update(tSituation *s)
{
	// Update global car data (shared by all instances) just once per timestep.

	if (currentsimtime != s->currentTime) {
		currentsimtime = s->currentTime;
		cardata->update();


		if (delay == 0){
			SeqRRTStar RRTStar = SeqRRTStar(new State(), 40);
			path = RRTStar.search();


		}
		
		if (delay == 300){
			/*currState = cuda_search(State())[0];
			std::cout << "B ou Smol? " << currState.getPedalPos() << " , " << currState.getSteerAngle() << std::endl;*/


			std::cout << "bauauauauyeye...: \n" << std::endl;


			for (std::vector<State>::iterator i = path.begin(); i != path.end(); ++i)
				std::cout << i->toString() << ' ' << std::endl;


			std::cout << "curr...:" << currState.toString() << std::endl;

			currState = path[(currPoint++)%path.size()];


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

	// Update the local data rest.
	speedangle = mycardata->getTrackangle() - atan2(car->_speed_Y, car->_speed_X);
	NORM_PI_PI(speedangle);
	mass = CARMASS + car->_fuel;
	currentspeedsqr = car->_speed_x*car->_speed_x;
	opponents->update(s, this);
	strategy->update(car, s);
	if (!pit->getPitstop()) {
		pit->setPitstop(strategy->needPitstop(car, s));
	}
	pit->update();
	alone = isAlone();
	learn->update(s, track, car, alone, myoffset, car->_trkPos.seg->width / WIDTHDIV - BORDER_OVERTAKE_MARGIN, radius);
}



// Compute aerodynamic downforce coefficient CA.
void Driver::initCa()
{
	const char *WheelSect[4] = { SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL, SECT_REARRGTWHEEL, SECT_REARLFTWHEEL };
	float rearwingarea = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGAREA, (char*)NULL, 0.0f);
	float rearwingangle = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGANGLE, (char*)NULL, 0.0f);
	float wingca = 1.23f*rearwingarea*sin(rearwingangle);

	float cl = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_FCL, (char*)NULL, 0.0f) +
		GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_RCL, (char*)NULL, 0.0f);
	float h = 0.0f;
	int i;
	for (i = 0; i < 4; i++)
		h += GfParmGetNum(car->_carHandle, WheelSect[i], PRM_RIDEHEIGHT, (char*)NULL, 0.20f);
	h *= 1.5f; h = h*h; h = h*h; h = 2.0f * exp(-3.0f*h);
	CA = h*cl + 4.0f*wingca;
}


// Compute aerodynamic drag coefficient CW.
void Driver::initCw()
{
	float cx = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_CX, (char*)NULL, 0.0f);
	float frontarea = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_FRNTAREA, (char*)NULL, 0.0f);
	CW = 0.645f*cx*frontarea;
}


// Init the friction coefficient of the the tires.
void Driver::initTireMu()
{
	const char *WheelSect[4] = { SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL, SECT_REARRGTWHEEL, SECT_REARLFTWHEEL };
	float tm = FLT_MAX;
	int i;

	for (i = 0; i < 4; i++) {
		tm = MIN(tm, GfParmGetNum(car->_carHandle, WheelSect[i], PRM_MU, (char*)NULL, 1.0f));
	}
	TIREMU = tm;
}


// Reduces the brake value such that it fits the speed (more downforce -> more braking).
float Driver::filterBrakeSpeed(float brake)
{
	float weight = (CARMASS + car->_fuel)*G;
	float maxForce = weight + CA*MAX_SPEED*MAX_SPEED;
	float force = weight + CA*currentspeedsqr;
	return brake*force / maxForce;
}



// Antilocking filter for brakes.
float Driver::filterABS(float brake)
{
	if (car->_speed_x < ABS_MINSPEED) return brake;
	int i;
	float slip = 0.0f;
	for (i = 0; i < 4; i++) {
		slip += car->_wheelSpinVel(i) * car->_wheelRadius(i);
	}
	slip = car->_speed_x - slip / 4.0f;
	if (slip > ABS_SLIP) {
		brake = brake - MIN(brake, (slip - ABS_SLIP) / ABS_RANGE);
	}
	return brake;
}


// TCL filter for accelerator pedal.
float Driver::filterTCL(float accel)
{
	float slip = (this->*GET_DRIVEN_WHEEL_SPEED)() - car->_speed_x;
	if (slip > TCL_SLIP) {
		accel = accel - MIN(accel, (slip - TCL_SLIP) / TCL_RANGE);
	}
	return accel;
}


// Traction Control (TCL) setup.
void Driver::initTCLfilter()
{
	const char *traintype = GfParmGetStr(car->_carHandle, SECT_DRIVETRAIN, PRM_TYPE, VAL_TRANS_RWD);
	if (strcmp(traintype, VAL_TRANS_RWD) == 0) {
		GET_DRIVEN_WHEEL_SPEED = &Driver::filterTCL_RWD;
	}
	else if (strcmp(traintype, VAL_TRANS_FWD) == 0) {
		GET_DRIVEN_WHEEL_SPEED = &Driver::filterTCL_FWD;
	}
	else if (strcmp(traintype, VAL_TRANS_4WD) == 0) {
		GET_DRIVEN_WHEEL_SPEED = &Driver::filterTCL_4WD;
	}
}


// TCL filter plugin for rear wheel driven cars.
float Driver::filterTCL_RWD()
{
	return (car->_wheelSpinVel(REAR_RGT) + car->_wheelSpinVel(REAR_LFT)) *
		car->_wheelRadius(REAR_LFT) / 2.0f;
}


// TCL filter plugin for front wheel driven cars.
float Driver::filterTCL_FWD()
{
	return (car->_wheelSpinVel(FRNT_RGT) + car->_wheelSpinVel(FRNT_LFT)) *
		car->_wheelRadius(FRNT_LFT) / 2.0f;
}


// TCL filter plugin for all wheel driven cars.
float Driver::filterTCL_4WD()
{
	return ((car->_wheelSpinVel(FRNT_RGT) + car->_wheelSpinVel(FRNT_LFT)) *
		car->_wheelRadius(FRNT_LFT) +
		(car->_wheelSpinVel(REAR_RGT) + car->_wheelSpinVel(REAR_LFT)) *
		car->_wheelRadius(REAR_LFT)) / 4.0f;
}


