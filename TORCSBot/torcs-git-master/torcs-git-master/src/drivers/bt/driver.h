
/***************************************************************************
*                                                                         *
*  This is the driver class implementation that creates and links         *
*      the bots modules.                                                  *
*                                                                         *
*  It was developed under the base of the "bt" TORCS driver.              *
*                                                                         *
***************************************************************************/

#ifndef _DRIVER_H_
#define _DRIVER_H_


#include <iostream>
#include <windows.h>
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

#include "SeqRRT.h"
#include "ParRRT.h"
#include "Kernel.cuh"
#include "RRT.cuh"

#include "PIDController.h"

#include "GL/glut.h"

#include "opponent.h"
#include "cardata.h"

#define BT_SECT_PRIV "bt private"
#define BT_ATT_MUFACTOR "mufactor"


class Opponents;
class Opponent;


class Driver {
	private:

		// Class constants.
		static const float SHIFT;
		static const float SHIFT_MARGIN;
		static const float CLUTCH_SPEED;
		static const float CLUTCH_FULL_MAX_TIME;

		// Data that should stay constant after first initialization.
		int INDEX;
		float MU_FACTOR;

		tTrackSeg* trackSegArray; //needed because of CUDA memory management
		
		//Kernel vars (only used in ParRRT)
		tTrackSeg* kernelSegArray;
		State* kernelGraph;

		static double currentsimtime;	// Store time to avoid useless updates.
		float clutchtime;
		float *radius;

		tPolarVel maxCarAcceleration; //can be used in the future to limit car acceleration (not currently used)

		PIDController pedalsPidController;

		RRT* RRTAux = NULL;	
	
		std::vector<State*> searchedPaths; //save the searched paths
		std::vector<State*> pathAhead; //ahead of current path
		std::vector<State*> path;
		int pathIterator = -1; //to go trough the path!
		State* currState;
		bool LASTNODE = true; //flag to wait for last node to be executed before path recalc
		bool STUCKONAPOINT = false;

		//search tunning vars
		int delay = 0;
		int SEARCH_RECALC_DELAY = 150;
		double ACTION_SIM_DELTA_TIME = 2.5;


		// Per robot global data.
		tCarElt *car;
		static Cardata *cardata;
		SingleCardata *mycardata;

		Opponents *opponents;
		Opponent *opponent;
		
		tTrack* track;

	private:// methods

		//------------CONTROL MODULE-----------------
		void computeRadius(float *radius);
		float getPedalsPos(double targetSpeed);
		int getGear();
		float getSteer(tPosd target);
		float getClutch();
		bool control(); //true if delay passed

		//------------PLANNING MODULE-----------------
		bool passedPoint(State* target); //true if its reached target
		void recalcPath(State initialState);
		void simplePlan(); // algorithm test
		void humanControl();

		//--------------MAIN UPDATE-------------------
		void update(tSituation *s);

	public:
		Driver(int index);
		~Driver();

		// Callback functions called from TORCS.
		void initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s);
		void newRace(tCarElt* car, tSituation *s);
		void drive(tSituation *s);

		// Callback functions called from oponnent.cpp
		tCarElt *getCarPtr() { return car; }
		tTrack *getTrackPtr() { return track; }
		float getSpeed() { return mycardata->getSpeedInTrackDirection(); /*speed;*/ }

		//display related local procedures
		void initGLUTWindow();
		void GLUTWindowRedisplay();
	
};




//display related global procedures
void drawSearchPoints();
void draw2DProjectedSearchPoints(); //for printing projected state space
void drawCurrStats();
void drawMapSegments();
void drawCircle(tPosd point, GLfloat radius);
void drawLine(double initialPointX, double initialPointY, double finalPointX, double finalPointY);
void drawCubicBezier(tPosd p0, tPosd p1, tPosd p2, tPosd p3, unsigned int numPartialPoints);
GLuint loadTexture(const char * filename);
void printTextInWindow(int x, int y, char *st);

#endif
