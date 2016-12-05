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

//----------------------- CONTROL MODULE ----------------------------------


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
	float omega = (car->_enginerpmRedLine) / gr_up;
	float wr = car->_wheelRadius(2);

	if (omega*wr*SHIFT < car->_speed_x) {
		return car->_gear + 1;
	}
	else {
		float gr_down = car->_gearRatio[car->_gear + car->_gearOffset - 1];
		omega = (car->_enginerpmRedLine) / gr_down;
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



bool Driver::seek(tPosd target){

	tdble carX = car->_pos_X;
	tdble carY = car->_pos_Y;
	tdble carZ = car->_pos_Z;
	tdble targetX = target.x;
	tdble targetY = target.y;
	tdble targetZ = target.z;

	float targetAngle;

	if (abs(carX - targetX) < 15 && abs(carY - targetY) < 15){
		STUCKONAPOINT = true;
		car->_accelCmd = 0.3;
		car->_gearCmd = getGear();

		return true;
	}

	STUCKONAPOINT = false;

	targetAngle = atan2(targetY - carY, targetX - carX);
	targetAngle -= car->_yaw;
	NORM_PI_PI(targetAngle);

	double diff = (currState.getSpeed().x + currState.getSpeed().y) - (car->pub.DynGCg.vel.x + car->pub.DynGCg.vel.y);

	if (diff > 0){
		car->_accelCmd = abs(diff)/ 120;
		car->_brakeCmd = 0;
	}
	else{
		car->_accelCmd = 0;
		car->_brakeCmd = abs(diff) / 100;
	}
	car->_gearCmd = getGear();


	//car->_accelCmd = 0.4f;
	//car->_gearCmd = getGear();
	//


	car->_steerCmd = targetAngle / car->_steerLock;


	return false;
}

double xG;
double yG;
double trkMinX;
double trkMinY;
double trkMaxX;
double trkMaxY;
int auxWindow;
std::vector<State> pathG = std::vector<State>();
bool CREATEDWINDOW = false;
// Update my private data every timestep.
void Driver::update(tSituation *s)
{


	// Update global car data (shared by all instances) just once per timestep.
	if (currentsimtime != s->currentTime) {
		currentsimtime = s->currentTime;
		cardata->update();

		

		//------------ producer --------------
		if (path.size() == 0 && LASTNODE && !STUCKONAPOINT){
			LASTNODE = false;


			State* initialState = new State(car->pub.DynGCg.pos, car->pub.DynGCg.vel, car->pub.DynGCg.acc);
			initialState->setPosSeg(*(car->pub.trkPos.seg));

			int numberOfIterations = 30;

			path.reserve(numberOfIterations);

			SeqRRTStar RRTStar = SeqRRTStar(*initialState, numberOfIterations, *track, *(car->pub.trkPos.seg), 20);
			path = RRTStar.search();


		 	printf( "escrever path: \n");

			for (std::vector<State>::iterator i = path.begin(); i != path.end(); ++i)
				std::cout << (*i).toString() << std::endl;


			currState = path.back();
			path.pop_back();


		}
		//------------ consumer --------------
		

		if (this->seek(currState.getPos())){
			if (path.size() == 0)
				LASTNODE = true;
			else{
				currState = path.back();
				path.pop_back();
			}
		}
		

		
		
	 //   tTrkLocPos otherLoc;

			//tPosd otherPos;

			//otherPos.x = opponent->getCarPtr()->pub.trkPos.seg->vertex[0].x;
			//otherPos.y = opponent->getCarPtr()->pub.trkPos.seg->vertex[0].y;
			//otherPos.z = opponent->getCarPtr()->pub.DynGC.pos.z;
			//
			////RtTrackGlobal2Local(opponent->getCarPtr()->pub.trkPos.seg, otherPos.x, otherPos.y, &otherLoc, TR_LPOS_MAIN);
			////std::cout << "(" << opponent->getCarPtr()->pub.trkPos.seg->name << ")" << std::endl;
			//std::cout << opponent->getCarPtr()->pub.trkPos.seg->id<<" : " << std::endl;
			//std::cout << "0(" << opponent->getCarPtr()->pub.trkPos.seg->vertex[0].x << "," << opponent->getCarPtr()->pub.trkPos.seg->vertex[0].y << ")" << std::endl;
			//std::cout << "1(" << opponent->getCarPtr()->pub.trkPos.seg->vertex[1].x << "," << opponent->getCarPtr()->pub.trkPos.seg->vertex[1].y << ")" << std::endl;
			//std::cout << "2(" << opponent->getCarPtr()->pub.trkPos.seg->vertex[2].x << "," << opponent->getCarPtr()->pub.trkPos.seg->vertex[2].y << ")" << std::endl;
			//std::cout << "3(" << opponent->getCarPtr()->pub.trkPos.seg->vertex[3].x << "," << opponent->getCarPtr()->pub.trkPos.seg->vertex[3].y << ")" << std::endl;
	
		xG = car->pub.DynGCg.pos.x;
		yG = car->pub.DynGCg.pos.y;
		trkMinX = track->min.x;
		trkMinY = track->min.y;
		trkMaxX = track->max.x;
		trkMaxY = track->max.y;
		pathG = path;

		if (!CREATEDWINDOW){
			initGLUTWindow();
			CREATEDWINDOW = true;
		}
		GLUTWindowRedisplay();
	}

}


void Driver::initGLUTWindow(){
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(trkMaxX - trkMinX, trkMaxY - trkMinY);
	glutCreateWindow("im a potato window bananana boyyyyyy");
	auxWindow = glutGetWindow();
	glutDisplayFunc(drawSearchPoints);
}

void Driver::GLUTWindowRedisplay(){
	int gameplayWindow = glutGetWindow();
	glutSetWindow(auxWindow);
	glutPostRedisplay();
	glutSetWindow(gameplayWindow);
}

void drawSearchPoints(){

	int w = glutGet(GLUT_WINDOW_WIDTH);
	int h = glutGet(GLUT_WINDOW_HEIGHT);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0.0f, w, h, 0.0f, 0.0f, 1.0f);

	glMatrixMode(GL_MODELVIEW);	
	glLoadIdentity();
	glPushMatrix();

		drawMap(0, 0, w, h);

		glColor3f(0, 0, 1);
		for (int i = 0; i < pathG.size(); i++){
			drawCircle(pathG[i].getPos().x, pathG[i].getPos().y, 2);
		}

		glColor3f(1, 0, 0);
		drawCircle(xG,yG, 2);

	glPopMatrix();
	glutSwapBuffers();
}

void drawCircle(GLfloat x, GLfloat y, GLfloat radius)
{

	int w = glutGet(GLUT_WINDOW_WIDTH);
	int h = glutGet(GLUT_WINDOW_HEIGHT);

	int i;
	int triangleAmount = 30;
	GLfloat twicePi = 2.0f * PI;

	glEnable(GL_LINE_SMOOTH);
	glLineWidth(5.0);

	glBegin(GL_LINES);
		for (i = 0; i <= triangleAmount; i++)
		{
			glVertex2f(x, y);
			glVertex2f(x + (radius * cos(i * twicePi / triangleAmount)), y + (radius * sin(i * twicePi / triangleAmount)));
		}
	glEnd();
}

void drawMap(GLfloat x, GLfloat y, int width, int height)
{
	glEnable(GL_TEXTURE_2D);
	glPushMatrix();	
	glLoadIdentity();
	glBindTexture(GL_TEXTURE_2D, loadTexture("tracks/g-track-1.bmp"));
	glBegin(GL_QUADS);
		glTexCoord2d(0.0, 0.0); glVertex2f(x, y);
		glTexCoord2d(1.0, 0.0); glVertex2f(x + width, y);
		glTexCoord2d(1.0, 1.0); glVertex2f(x + width, y + height);
		glTexCoord2d(0.0, 1.0); glVertex2f(x, y + width);
	glEnd();
	glPopMatrix();
	glDisable(GL_TEXTURE_2D);

}

GLuint loadTexture(const char * filename)
{

	GLuint texture;

	int width, height;



	FILE * file;

	file = fopen(filename, "rb");

	if (file == NULL) { 
		std::cout << "wrong path!" << std::endl;
		return 0; 
	}
	width = 512;
	height = 512;

	unsigned char data[512*512 * 3];
	//int size = fseek(file,);
	fread(data, width * height * 3, 1, file);
	fclose(file);

	std::vector<unsigned char> dataVector = std::vector<unsigned char>(width * height * 3);

	dataVector.assign(data, data + sizeof(data)); //transform array in vector
	dataVector.erase(dataVector.begin(), dataVector.begin() + 54);//removes windows bmp header

	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);


	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	gluBuild2DMipmaps(GL_TEXTURE_2D, 3, width, height, GL_RGB, GL_UNSIGNED_BYTE, dataVector.data());

	return texture;
}




//currState = cuda_search(State())[0];
//std::cout << "currState:" << currState.getPedalPos() << " , " << currState.getSteerAngle() << std::endl;

/*std::cout << "(" << car->pub.DynGCg.pos.x << "," << car->pub.DynGCg.pos.y << ")" << std::endl;
std::cout << "(" << currState.getPos().x << "," << currState.getPos().y << ")" << std::endl;*/

//std::cout << "curr...:" << currState.toString() << std::endl;*/
