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


//-------------- display GLOBAL aux vars -----------------------
tDynPt carDynCg;
double trkMinX;
double trkMinY;
double trkMaxX;
double trkMaxY;
State currStateG;

int pathsauxWindow;
int currStatsWindow;

std::vector<State*> pathG;
std::vector<State*> graphG;
bool CREATEDWINDOW = false;

GLuint mapTextureID;

//-------------------------------------------------------------

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
	if (RRTStarAux != NULL) {
		delete RRTStarAux;
		RRTStarAux = NULL;
	}

	for (int i = 0; i < path.size(); i++){
		delete path[i];
	}
	
	for (int i = 0; i < track->nseg; i++){
		delete &trackSegArray[i];
	}
	free(trackSegArray);

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

	trackSegArray = (tTrackSeg*)malloc(sizeof(tTrackSeg)*track->nseg);

	double size = sizeof(trackSegArray);


	trackSegArray[0] = *(track->seg->next);
	tTrackSeg* currSeg = track->seg->next->next;
	int trackSegIterator=1;
	while (currSeg != track->seg->next){
		trackSegArray[trackSegIterator] = *currSeg;
		currSeg = currSeg->next;
		trackSegIterator++;
	}

	for (int i = 0; i < track->nseg; i++){
		printf("segId:%d\n", trackSegArray[i].id);
	}

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

	//------------------------------------------------------------------------------------------

	this->RRTStarAux = NULL;

	//create PID controllers 

	//split channels for the pedals
	gasPidController = PIDController(0.015, 0.005, 0.001);
	brakePidController = PIDController(0.005, 0.001, 0.001);

	//steerPidController = PIDController(2, 5, 0.0001); 


}


// Interface to be called to drive during race.
void Driver::drive(tSituation *s)
{
	memset(&car->ctrl, 0, sizeof(tCarCtrl));
	update(s);
}


//----------------------------------------------
//------------CONTROLLER MODULE-----------------
//----------------------------------------------

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
	float omega = (car->_enginerpmRedLine*0.8) / gr_up;
	float wr = car->_wheelRadius(2);

	if (omega*wr*SHIFT < car->_speed_x) {
		return car->_gear + 1;
	}
	else {
		float gr_down = car->_gearRatio[car->_gear + car->_gearOffset - 1];
		omega = (car->_enginerpmRedLine*0.7) / gr_down;
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

float Driver::getPedalsPos(tPosd targetSpeed)
{
	double targetSpeedMod = std::sqrt(targetSpeed.x*targetSpeed.x + targetSpeed.y*targetSpeed.y);
	double carSpeedMod = std::sqrt(car->pub.DynGC.vel.x*car->pub.DynGC.vel.x + car->pub.DynGC.vel.y*car->pub.DynGC.vel.y);


	double output = 0;


	output = gasPidController.getOutput(targetSpeedMod, carSpeedMod, 0.02);
	return output;

}

bool Driver::control(){

	//--------------- STEER CONTROL--------------------------

	car->_steerCmd = this->getSteer(currState->getPos());

	//--------------- SHIFT CONTROL--------------------------

	car->ctrl.gear = getGear();

	//--------------- PEDAL CONTROL--------------------------

	float pedalsOutput = getPedalsPos(currState->getSpeed());
	pedalsOutput > 0 ? car->_accelCmd = pedalsOutput : car->_brakeCmd = -1*pedalsOutput;

	return true;

}


//----------------------------------------------
//------------PLANNING MODULE-------------------
//----------------------------------------------

bool Driver::passedPoint(State* target){

	tdble carX = car->_pos_X;
	tdble carY = car->_pos_Y;
	tdble carZ = car->_pos_Z;
	tdble targetX = target->getPos().x;
	tdble targetY = target->getPos().y;
	tdble targetZ = target->getPos().z;


	if (abs(carX - targetX) < 18 && abs(carY - targetY) < 18){
		STUCKONAPOINT = true;
		return true;
	}

	STUCKONAPOINT = false;
	return false;
}

void Driver::recalcPath(State initialState){
	if (RRTStarAux != NULL) {
		delete RRTStarAux;
		RRTStarAux = NULL;
		graphG.clear(); //to avoid having deleted members
	}
	RRTStarAux = new SeqRRTStar(initialState, numberOfPartialIterations, *car, trackSegArray, track->nseg, initialState.getPosSeg(), SEARCH_SEGMENTS_AHEAD);
}

void Driver::plan()
{
	delay++;

	//------------ producer --------------

	//check if new recalc is needed
	if (pathIterator<0 && LASTNODE){
		LASTNODE = false;

		//delocate current path before path recalc
		for (int i = 0; i < path.size(); i++){
			delete path[i];
		}
		path.clear();


		if (pathAhead.size() == 0){
			State initialState = State(car->pub.DynGCg.pos, car->pub.DynGCg.vel, car->pub.DynGCg.acc);
			initialState.setPosSeg(*(car->pub.trkPos.seg));
			initialState.setInitialState(true); //it is indeed the initial state!
			recalcPath(initialState);
			path = RRTStarAux->search();
		}else{
			path = pathAhead;
		}


		pathIterator = path.size() - 1;
		pathG = path;
		currState = path[pathIterator--];
		
		searchedPaths.clear(); //clear the paths deleted later

		State* initialStateAux = path[0];
		initialStateAux->setInitialState(true);
		recalcPath(*initialStateAux);
		pathAhead = RRTStarAux->search();

	}else{
		if (numberOfRealIterations == 0){
			State initialStateAux = *pathAhead[0];
			initialStateAux.setInitialState(true);
			recalcPath(initialStateAux);

			searchedPaths.clear();
			searchedPaths = pathAhead;

			numberOfRealIterations = numberOfIterations;
		}
	}

	//------------ consumer --------------
	if (this->passedPoint(currState)){
		gasPidController.resetController();
		brakePidController.resetController();
		if (pathIterator<0){
			LASTNODE = true;
		}else{
			currState = path[pathIterator--];
		}
	}

	//avoid more than 1 lap in search still not implemented!
	if (delay == SEARCH_RECALC_DELAY){
		RRTStarAux->updateCar(*car);
		pathAhead = RRTStarAux->search();
		pathAhead.insert(pathAhead.end(), searchedPaths.begin(), searchedPaths.end());

		graphG = RRTStarAux->getGraph(); //update aux window var

		delay = 0;
		numberOfRealIterations -= numberOfPartialIterations;
	}
	
	//-------------------AUX WINDOW VARS-------------------------------------

	carDynCg = car->pub.DynGCg;
	trkMinX = track->min.x;
	trkMinY = track->min.y; 
	trkMaxX = track->max.x; 
	trkMaxY = track->max.y; 
	currStateG = *this->currState;

	//init aux windows
	if (!CREATEDWINDOW){
		initGLUTWindow();
		CREATEDWINDOW = true;
	}
	GLUTWindowRedisplay(); //update aux windows

}


void Driver::simplePlan() // algorithm test 
{
	

	//-------------------AUX WINDOW VARS-------------------------------------

	carDynCg = car->pub.DynGCg;
	trkMinX = track->min.x;
	trkMinY = track->min.y;
	trkMaxX = track->max.x;
	trkMaxY = track->max.y;
	currStateG = State();

	//init aux windows
	if (!CREATEDWINDOW){
		initGLUTWindow();
		CREATEDWINDOW = true;
	}
	GLUTWindowRedisplay(); //update aux windows
	
	if (path.size() == 0)
	{
		/*if (GetAsyncKeyState(VK_TAB) & 0x8000)
		{*/
			State initialState = State(carDynCg.pos, carDynCg.vel, carDynCg.acc);
			initialState.setPosSeg(*(car->pub.trkPos.seg));
			initialState.setInitialState(true); //it is indeed the initial state!
			RRTStarAux = new ParRRTStar(initialState, 200, *car, trackSegArray, track->nseg, initialState.getPosSeg(), SEARCH_SEGMENTS_AHEAD);
			path = RRTStarAux->search();
			pathG = path;
			graphG = RRTStarAux->getGraph();
		//}
		currState = path[path.size() - 1];
	}else{
		if (passedPoint(currState)){
			path.pop_back();
			if (path.size()!=0)
				currState = path[path.size()-1];
			
		}
	}
}

State* firstState;
void Driver::humanControl(){
	if (GetAsyncKeyState(VK_UP) & 0x8000)
	{
		car->_accelCmd = 0.5;
	}

	if (GetAsyncKeyState(VK_DOWN) & 0x8000)
	{
		car->_brakeCmd = 0.5;
	}
	if (GetAsyncKeyState(VK_LEFT) & 0x8000)
	{
		car->_steerCmd = car->_steerLock*2;
	}
	if (GetAsyncKeyState(VK_RIGHT) & 0x8000)
	{
		car->_steerCmd = - car->_steerLock*2;
	}
	if (GetAsyncKeyState(VK_SPACE) & 0x8000)
	{
		car->_gearCmd = -1;
	}
	else{
		car->_gearCmd = getGear();
	}


	
	/*if (!CREATEDWINDOW){
		firstState = new State(car->pub.DynGC.pos, car->pub.DynGC.vel, car->pub.DynGC.acc);
		firstState->setPosSeg(*car->pub.trkPos.seg);
		CREATEDWINDOW = true;
	}*/
	
	/*currState = &State(car->pub.DynGC.pos, car->pub.DynGC.vel, car->pub.DynGC.acc);
	std::cout << "currSegment: " << car->pub.trkPos.seg->name << "::" << car->pub.trkPos.seg->id << std::endl;
	currState->setPosSeg(*car->pub.trkPos.seg);
	printf("fritar a pipoca: %f\n", EvalFunctions::evaluatePathCost(trackSegArray, track->nseg, currState, firstState, 150));
	printf("fritar a pipoca2: %f\n", UtilityMethods::SimpleRtTrackGlobal2Local(trackSegArray, car->pub.trkPos.seg->id, track->nseg, car->pub.DynGC.pos.x, car->pub.DynGC.pos.y, 0).toStart);
	tTrkLocPos p;
	RtTrackGlobal2Local(car->pub.trkPos.seg, car->pub.DynGC.pos.x, car->pub.DynGC.pos.y,&p, 0);
	printf("fritar a pipoca3: %f\n", p.toStart*car->pub.trkPos.seg->radius);
*/
	/*if (!ConstraintChecking::validPoint(trackSegArray, track->nseg, currState, 0))
		printf("asian driver!\n");
	else
		printf("normal!\n");*/

}
//--------------------------------------------------------------------
// - Main update (calls the planning module and the control module). -
//--------------------------------------------------------------------
void Driver::update(tSituation *s)
{

	//if (delay == 10000){
	//	getchar();
	//}

	//printf("delay: %d\n", delay);
	
	// Update global car data (shared by all instances) just once per timestep.
	if (currentsimtime != s->currentTime) {
		currentsimtime = s->currentTime;
		cardata->update();

		//this->plan();
		this->simplePlan();
		this->control();
		//this->humanControl();
	}
}


//------------------------------------------------
//------------DEBUG WINDOW CODE-------------------
//------------------------------------------------

void Driver::initGLUTWindow(){

	glutInitWindowSize(600, 300);
	currStatsWindow = glutCreateWindow("current stats...");
	glutPositionWindow(200,700);
	glutDisplayFunc(drawCurrStats);

	glutInitWindowSize((int)(trkMaxX - trkMinX), (int)(trkMaxY - trkMinY));
	
	pathsauxWindow = glutCreateWindow("drawing paths...");
	glutPositionWindow(800, 100);
	glutDisplayFunc(drawSearchPoints);

	glutSetWindow(pathsauxWindow);
	mapTextureID = loadTexture("auxWindow/g-track-1.bmp");
	
}

void Driver::GLUTWindowRedisplay(){
	int gameplayWindow = glutGetWindow();
	
	glutSetWindow(currStatsWindow);
	glutPostRedisplay();
	glutSetWindow(pathsauxWindow);
	glutPostRedisplay();	
	glutSetWindow(gameplayWindow);

}


void drawCurrStats(){
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(1, 1, 1, 1);

	int w = glutGet(GLUT_WINDOW_WIDTH);
	int h = glutGet(GLUT_WINDOW_HEIGHT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glOrtho(0.0f, w, h, 0.0f, 0.0f, 1.0f);
	glPushMatrix();
		glColor3f(0, 0, 0);
		std::string currStateGVelInfo = std::string("speed: (") + std::to_string((double)currStateG.getSpeed().x) + std::string(" , ") + std::to_string((double)currStateG.getSpeed().y) + std::string(" ) \n");
		std::string currStateGAccelInfo = std::string("acceleration: (") + std::to_string((double)currStateG.getAcceleration().x) + std::string(" , ") + std::to_string((double)currStateG.getAcceleration().y) + std::string(" ) \n");
		printTextInWindow(24, 24, (char*)currStateGVelInfo.c_str());
		printTextInWindow(24, 40, (char*)currStateGAccelInfo.c_str());
	glPopMatrix();
	glutSwapBuffers();
}

void drawSearchPoints(){

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(1, 1, 1, 1);

	int w = glutGet(GLUT_WINDOW_WIDTH);
	int h = glutGet(GLUT_WINDOW_HEIGHT);


	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0.0f, w, h, 0.0f, 0.0f, 1.0f);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glPushMatrix();



	drawMap(0, 0, w, h);

	for (int i = 0; i < (graphG).size(); i++){
		glColor3f(0, 0, 0);
		drawCircle(*(graphG[i]), 0.5);
		if (!graphG[i]->getInitialState()){
			drawLine(graphG[i]->getPos().x, h - graphG[i]->getPos().y, graphG[i]->getParent()->getPos().x, h - graphG[i]->getParent()->getPos().y);
		}

	}

	glColor3f(0, 0, 1);
	for (int i = 1; i < pathG.size(); i++){
		drawLine(pathG[i]->getPos().x, h -pathG[i]->getPos().y, pathG[i - 1]->getPos().x, h - pathG[i - 1]->getPos().y);
	}

	for (int i = 0; i < pathG.size(); i++){
		if (pathG[i]->getPathCost() < currStateG.getPathCost()){
			glColor3f(0, 1, 1);
			drawCircle(*pathG[i], 2);
		}
		else{

			glColor3f(0, 0, 1);
			drawCircle(*pathG[i], 2);
		}
		std::string statePosSeg = std::to_string((double)pathG[i]->getPathCost());
		printTextInWindow(pathG[i]->getPos().x + 10, (h -pathG[i]->getPos().y) + 10, (char*)statePosSeg.c_str());

	}

	glColor3f(1, 0, 1);
	drawCircle(currStateG, 3);

	glColor3f(1, 0, 0);

	drawCircle(State(carDynCg.pos, carDynCg.vel, carDynCg.acc), 2);

	glPopMatrix();
	glutSwapBuffers();
}

void drawCircle(State point, GLfloat radius)
{


	int w = glutGet(GLUT_WINDOW_WIDTH);
	int h = glutGet(GLUT_WINDOW_HEIGHT);

	int x = point.getPos().x;
	int y = h - point.getPos().y;

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

void drawLine(double initialPointX, double initialPointY, double finalPointX, double finalPointY)
{
	glLineWidth(0.5);
	glBegin(GL_LINES);
		glVertex2f(initialPointX, initialPointY);
		glVertex2f(finalPointX, finalPointY);
	glEnd();

}

void drawMap(GLfloat x, GLfloat y, int width, int height)
{
	glEnable(GL_TEXTURE_2D);
	glPushMatrix();
	glLoadIdentity();
	glBindTexture(GL_TEXTURE_2D, mapTextureID);
	glBegin(GL_QUADS);
	glTexCoord2d(0.0, 1.0); glVertex2f(x, y);
	glTexCoord2d(1.0, 1.0); glVertex2f(x + width, y);
	glTexCoord2d(1.0, 0.0); glVertex2f(x + width, y + height);
	glTexCoord2d(0.0, 0.0); glVertex2f(x, y + height);
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
		//std::cout << "wrong path!" << std::endl;
		return 0;
	}
	width = 512;
	height = 512;

	unsigned char data[512 * 512 * 3];
	//int size = fseek(file,);
	fread(data, width * height * 3, 1, file);
	fclose(file);

	std::vector<unsigned char> dataVector = std::vector<unsigned char>(width * height * 3);

	dataVector.assign(data, data + sizeof(data)); //transform array in vector
	dataVector.erase(dataVector.begin(), dataVector.begin() + 54);//removes windows bmp header

	//std::reverse(dataVector.begin(), dataVector.end());

	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);


	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	gluBuild2DMipmaps(GL_TEXTURE_2D, 3, width, height, GL_RGB, GL_UNSIGNED_BYTE, dataVector.data());

	return texture;
}

void printTextInWindow(int x, int y, char *st)
{
	int l, i;

	l = strlen(st); // see how many characters are in text string.
	glRasterPos2i(x, y); // location to start printing text
	for (i = 0; i < l; i++) // loop until i is greater then l
	{
		glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, st[i]); // Print a character on the screen
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

