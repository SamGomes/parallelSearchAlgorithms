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


#include "PIDController.h"

#include "Kernel.cuh"

#include "SeqRRTStar.h"
#include "SeqRRTOpt.h"

#include "GL/glut.h"

#include "opponent.h"
#include "cardata.h"

#define BT_SECT_PRIV "bt private"
#define BT_ATT_MUFACTOR "mufactor"


class Opponents;
class Opponent;


class Driver {

	private:
		PIDController pedalsPidController;
		PIDController steerPidController;

		SeqRRTOpt RRTOpt;


		std::vector<State*> path; 
		int pathIterator = -1; //to go trough the path!
		State* currState;

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

		bool passedPoint(State* target); //true if its reached target

		bool pidControl(State* target); //true if delay passed


		// Utility functions.
		void computeRadius(float *radius);
		void update(tSituation *s);

		float getAccel();
		float getBrake();
		int getGear();
		float getSteer(tPosd target);
		float getClutch();
		float getOffset();




		//search vars
		int delay = 0;

		SeqRRTStar RRTStarAux;
		std::vector<State*> pathAux;

		//search tunning vars
		int numberOfIterations = 200;
		int numberOfRealIterations = numberOfIterations;
		int numberOfPartialIterations = numberOfIterations / 4;

	public:

		Driver(int index);
		~Driver();


		// Callback functions called from TORCS.
		void initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s);
		void newRace(tCarElt* car, tSituation *s);
		bool validPoint(tPosd target);
		void drive(tSituation *s);

		void initGLUTWindow();
		void GLUTWindowRedisplay();


		tCarElt *getCarPtr() { return car; }
		tTrack *getTrackPtr() { return track; }
		float getSpeed() { return mycardata->getSpeedInTrackDirection(); /*speed;*/ }

	
};




//display related global procedures
void drawSearchPoints();
void drawCurrStats();
void drawMap(GLfloat x, GLfloat y, int width, int height);
void drawCircle(State point, GLfloat radius);
void drawLine(double initialPointX, double initialPointY, double finalPointX, double finalPointY);
GLuint loadTexture(const char * filename);
void printTextInWindow(int x, int y, char *st);
#endif

