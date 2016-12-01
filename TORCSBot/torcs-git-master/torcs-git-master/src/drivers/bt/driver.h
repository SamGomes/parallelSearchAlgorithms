/***************************************************************************

    file                 : driver.h
    created              : Thu Dec 20 01:20:19 CET 2002
    copyright            : (C) 2002-2004 Bernhard Wymann
    email                : berniw@bluewin.ch
    version              : $Id: driver.h,v 1.12.2.1 2008/11/09 17:50:19 berniw Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef _DRIVER_H_
#define _DRIVER_H_

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <math.h>

#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>
#include <portability.h>


#include "Kernel.cuh"
#include "SeqRRTStar.h"

//#include "carstruct.h"

#include "GL/glut.h"

#include "opponent.h"
#include "cardata.h"

#define BT_SECT_PRIV "bt private"
#define BT_ATT_MUFACTOR "mufactor"


class Opponents;
class Opponent;


class Driver {

	private:
		// Utility functions.
		void computeRadius(float *radius);
		void update(tSituation *s);

		float getAccel();
		float getBrake();
		int getGear();
		float getSteer(tPosd target);
		float getClutch();
		float getOffset();



		State currState;
		std::vector<State> path = std::vector<State>();

		bool LASTNODE = true; //flag to wait for last node to be executed before path recalc
		bool STUCKONAPOINT = false;


		// Per robot global data.
		tCarElt *car;			// Pointer to tCarElt struct.

		Opponents *opponents;	// The container for opponents.
		Opponent *opponent;		// The array of opponents.


		static Cardata *cardata;		// Data about all cars shared by all instances.
		SingleCardata *mycardata;		// Pointer to "global" data about my car.
		static double currentsimtime;	// Store time to avoid useless updates.

		float clutchtime;		// Clutch timer.

		float *radius;

		// Data that should stay constant after first initialization.
		int INDEX;
		float MU_FACTOR;				// [-]

		// Class constants.
		static const float SHIFT;
		static const float SHIFT_MARGIN;
		static const float CLUTCH_SPEED;
		static const float CLUTCH_FULL_MAX_TIME;

		// Track variable.
		tTrack* track;

		bool seek(tPosd target); //true if its in place

		void drawFilledSphere(GLfloat x, GLfloat y, GLfloat z, GLfloat radius); //for debug purposes

	public:

		Driver(int index);
		~Driver();


		// Callback functions called from TORCS.
		void initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s);
		void newRace(tCarElt* car, tSituation *s);
		bool validPoint(tPosd target);
		void drive(tSituation *s);

		tCarElt *getCarPtr() { return car; }
		tTrack *getTrackPtr() { return track; }
		float getSpeed() { return mycardata->getSpeedInTrackDirection(); /*speed;*/ }

	
};

#endif

