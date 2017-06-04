#include "driver.h"

//-------------- display GLOBAL aux vars -----------------------
tDynPt carDynCg;
double trkMinX;
double trkMinY;
double trkMaxX;
double trkMaxY;
State currStateG;

trackSeg* segmentArrayG;
int nsegsG;

int pathsauxWindow;
int currStatsWindow;

std::vector<State*> pathG;
std::vector<State> graphG;
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


// For test purposes
int run;
int numStates = 6400;

int numKernelThreads = 400;
int numKernelBlocks = 4;



Driver::Driver(int index)
{
	INDEX = index;
}

Driver::~Driver()
{
	StatsLogWriter::closeLog("ParallelRRTSearchTimes");
	StatsLogWriter::closeLog("SequentialRRTSearchTimes");
	delete opponents;
	delete[] radius;
	if (cardata != NULL) {
		delete cardata;
		cardata = NULL;
	}
	if (RRTAux != NULL) {
		delete RRTAux;
		RRTAux = NULL;
	}
	for (int i = 0; i < path.size(); i++){
		delete path[i];
	}
	for (int i = 0; i < track->nseg; i++){
		delete &trackSegArray[i];
	}
	free(trackSegArray);
	Kernel::gpuFree(kernelGraph,kernelSegArray);
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

	// pass segments to array in order to pass them to the kernel when needed later on
	trackSegArray[0] = *(track->seg->next);
	tTrackSeg* currSeg = track->seg->next->next;
	int trackSegIterator=1;
	while (currSeg != track->seg->next){
		trackSegArray[trackSegIterator] = *currSeg;
		currSeg = currSeg->next;
		trackSegIterator++;
	}
	
	segmentArrayG = trackSegArray;
	nsegsG = track->nseg;
	for (int i = 0; i < track->nseg; i++){
		printf("segId:%d\n", trackSegArray[i].id);
	}
	
	Kernel::gpuInit(&kernelGraph, &kernelSegArray, numStates, trackSegArray, track->nseg);
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
	this->RRTAux = NULL;

	//initialize pedals PID controller
	pedalsPidController = PIDController(0.005, 0.008, 0.001);

	maxCarAcceleration.angle = car->_steerLock;
	maxCarAcceleration.intensity = 5;

	//to get a good start
	car->_accelCmd = 1.0;

}


// Interface to be called to drive during race.
void Driver::drive(tSituation *s)
{
	memset(&car->ctrl, 0, sizeof(tCarCtrl));
	update(s);
}


//----------------------------------------------
//---------------CONTROL MODULE-----------------
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
	float omega = (car->_enginerpmRedLine*1.0) / gr_up;
	float wr = car->_wheelRadius(2);

	if (omega*wr*SHIFT < car->_speed_x) {
		return car->_gear + 1;
	}
	else {
		float gr_down = car->_gearRatio[car->_gear + car->_gearOffset - 1];
		omega = (car->_enginerpmRedLine*1.0) / gr_down;
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

float Driver::getPedalsPos(double targetSpeed)
{
	double carSpeedMod = std::sqrt(car->pub.DynGC.vel.x*car->pub.DynGC.vel.x + car->pub.DynGC.vel.y*car->pub.DynGC.vel.y);

	double output = 0;
	output = pedalsPidController.getOutput(targetSpeed, carSpeedMod, 0.02);
	
	return output;
}

bool Driver::control(){

	//--------------- STEER CONTROL--------------------------
	car->_steerCmd = this->getSteer(currState->getPos());

	//--------------- SHIFT CONTROL--------------------------
	car->ctrl.gear = getGear();

	//--------------- PEDAL CONTROL--------------------------
	float pedalsOutput = getPedalsPos(currState->getVelocity().intensity);
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

	if (abs(carX - targetX) < 8 && abs(carY - targetY) < 8){
		STUCKONAPOINT = true;
		return true;
	}
	STUCKONAPOINT = false;
	return false;
}


int oldLaps;
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
	
	//------------------ CALL CYCLE -----------------------------------------
	if (path.size() == 0 || delay == SEARCH_RECALC_DELAY)
	{
		delay = 0;
		if (RRTAux != NULL) {
			delete RRTAux;
			RRTAux = NULL;
			graphG.clear(); //to avoid having deleted members
		}

		double initialStateVelAngle = atan2(carDynCg.vel.y, carDynCg.vel.x);
		double initialStateVelIntensity = sqrt(carDynCg.vel.x*carDynCg.vel.x + carDynCg.vel.y*carDynCg.vel.y);

		tPolarVel carVel;
		carVel.angle = initialStateVelAngle;
		carVel.intensity = initialStateVelIntensity;

		tStateRelPos carLocPos;
		carLocPos.segId = car->pub.trkPos.seg->id;
		carLocPos.toLeft = car->pub.trkPos.toLeft;
		carLocPos.toMiddle = car->pub.trkPos.toMiddle;
		carLocPos.toRight = car->pub.trkPos.toRight;
		carLocPos.toStart = car->pub.trkPos.toStart;


		State initialState = State(carDynCg.pos, carVel);
		initialState.setLocalPos(carLocPos);
		initialState.setInitialState(true); //it is indeed the initial state!
		//RRTAux = new SeqRRT(initialState, numStates, trackSegArray, track->nseg, ACTION_SIM_DELTA_TIME, maxCarAcceleration);
		RRTAux = new ParRRT(initialState, numStates, kernelGraph, kernelSegArray, track->nseg, ACTION_SIM_DELTA_TIME, maxCarAcceleration, numKernelBlocks, numKernelThreads);
			
		clock_t searchTimer = clock();
			
		path = RRTAux->search();
			
		searchTimer = clock() - searchTimer;
		std::string stats = std::string(" ") + std::to_string(double(searchTimer) / (double)CLOCKS_PER_SEC);
			
		if (car->race.remainingLaps != oldLaps){
			stats += std::string("; lap time: ") + std::to_string(car->race.lastLapTime);
			oldLaps = car->race.remainingLaps;
		}

		if (strcmp(RRTAux->getSearchName(), "SequentialRRT") == 0){
		
			StatsLogWriter::writeToLog((char*)std::string("SequentialRRT states_" + std::to_string(numStates) + " run_" + std::to_string(run)).c_str(), stats);
		}
		
		if (strcmp(RRTAux->getSearchName(), "ParallelRRT") == 0){
			StatsLogWriter::writeToLog((char*)std::string("ParallelRRT(" + std::to_string(numKernelBlocks) + "," + std::to_string(numKernelThreads) + ") states_" + std::to_string(numStates) + " run_" + std::to_string(run)).c_str(), stats);
		}

		pathG = path;
		graphG = RRTAux->getGraph();
		std::reverse(pathG.begin(), pathG.end());

		currState = path[path.size() - 1];
	}else{
		if (passedPoint(currState)){
			path.pop_back();
			if (path.size() != 0){
				currState = path[path.size()-1];
				
			}
		}
	}
	currStateG = *currState;
	delay++;
}

State initialState; 
void Driver::humanControl(){ //allows for manual car control and search call
	
	if (GetAsyncKeyState(VK_UP) & 0x8000)
	{
		car->_accelCmd = 1.0;
	}
	if (GetAsyncKeyState(VK_TAB) & 0x8000)
	{
		if (RRTAux != NULL) {
			delete RRTAux;
			RRTAux = NULL;
			graphG.clear(); //to avoid having deleted members
		}

		double initialStateVelAngle = atan2(carDynCg.vel.y, carDynCg.vel.x);
		double initialStateVelIntensity = sqrt(carDynCg.vel.x*carDynCg.vel.x + carDynCg.vel.y*carDynCg.vel.y);

		tPolarVel carVel;
		carVel.angle = initialStateVelAngle;
		carVel.intensity = initialStateVelIntensity;

		tStateRelPos carLocPos;
		carLocPos.segId = car->pub.trkPos.seg->id;
		carLocPos.toLeft = car->pub.trkPos.toLeft;
		carLocPos.toMiddle = car->pub.trkPos.toMiddle;
		carLocPos.toRight = car->pub.trkPos.toRight;
		carLocPos.toStart= car->pub.trkPos.toStart;

		initialState = State(carDynCg.pos,carVel);
		initialState.setLocalPos(carLocPos);
		initialState.setInitialState(true); //it is indeed the initial state!

		//RRTAux = new SeqRRT(initialState, numStates, trackSegArray, track->nseg, ACTION_SIM_DELTA_TIME, maxCarAcceleration);
		RRTAux = new ParRRT(initialState, numStates, kernelGraph, kernelSegArray, track->nseg, ACTION_SIM_DELTA_TIME, maxCarAcceleration, numKernelBlocks, numKernelThreads);
		path = RRTAux->search();
		pathG = path;
		std::reverse(pathG.begin(), pathG.end());
		graphG = RRTAux->getGraph();

		currState = &initialState;
	}

	if (GetAsyncKeyState(VK_DOWN) & 0x8000)
	{
		car->_brakeCmd = 0.8;
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
	
	//-------------------AUX WINDOW VARS-------------------------------------
	carDynCg = car->pub.DynGCg;
	trkMinX = track->min.x;
	trkMinY = track->min.y;
	trkMaxX = track->max.x;
	trkMaxY = track->max.y;
	//currStateG = State();

	//init aux windows
	if (!CREATEDWINDOW){
		initGLUTWindow();
		CREATEDWINDOW = true;
	}
	GLUTWindowRedisplay(); //update aux windows

	State carState;
	carState.setPos(car->pub.DynGC.pos);
	tStateRelPos carLocPos;
	carLocPos.segId = car->pub.trkPos.seg->id;
	carLocPos.toLeft = car->pub.trkPos.toLeft;
	carLocPos.toMiddle = car->pub.trkPos.toMiddle;
	carLocPos.toRight = car->pub.trkPos.toRight;
	carLocPos.toStart = car->pub.trkPos.toStart;

	carState.setLocalPos(carLocPos);
}

//--------------------------------------------------------------------
// - Main update (calls the planning module and the control module). -
//--------------------------------------------------------------------
void Driver::update(tSituation *s)
{	
	// Update global car data (shared by all instances) just once per timestep.
	if (currentsimtime != s->currentTime) {
		currentsimtime = s->currentTime;
		cardata->update();

		//this->simplePlan();
		//this->control();
		this->humanControl(); //allows to control the car if the search makes unrecoverable error
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
		std::string currStateGPosInfo = std::string("pos: (") + std::to_string((double)currStateG.getPos().x) + std::string(" , ") + std::to_string((double)currStateG.getPos().y) + std::string(" ) \n");
		printTextInWindow(24, 40, (char*)currStateGPosInfo.c_str());
	glPopMatrix();
	glutSwapBuffers();
}

//projects states into map
void drawSearchPoints(){

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.7, 0.7, 0.7, 0.7);

	int w = glutGet(GLUT_WINDOW_WIDTH);
	int h = glutGet(GLUT_WINDOW_HEIGHT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0.0f, w, h, 0.0f, 0.0f, 1.0f);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glPushMatrix();

	for (int i = 1; i < (graphG).size(); i++){
		
		glColor3f(0, 0, 0);
		if (graphG[i].getMyGraphIndex() == -1)  //still unassigned
			continue;

		tPosd pointCartesianVelocity;
		pointCartesianVelocity.x = graphG[i].getPos().x;
		pointCartesianVelocity.y = graphG[i].getPos().y;

		tPosd parentCartesianVelocity;
		parentCartesianVelocity.x = graphG[graphG[i].getParentGraphIndex()].getPos().x;
		parentCartesianVelocity.y = graphG[graphG[i].getParentGraphIndex()].getPos().y;

		drawCircle(pointCartesianVelocity, 0.5);
		if (!graphG[i].getInitialState()){
			drawLine(pointCartesianVelocity.x, pointCartesianVelocity.y, parentCartesianVelocity.x, parentCartesianVelocity.y);
		}
	}

	for (int i = 1; i < pathG.size(); i++){
		drawLine(pathG[i]->getPos().x, pathG[i]->getPos().y, pathG[i - 1]->getPos().x, pathG[i - 1]->getPos().y);
	}
	for (int i = 0; i < pathG.size(); i++){
		glColor3f(0, 0 + 0.2*i, 1);
		drawCircle(pathG[i]->getPos(), 2);

		std::string statePosSeg = std::to_string((double)pathG[i]->distFromStart);
		printTextInWindow(pathG[i]->getPos().x + 10, (h - pathG[i]->getPos().y) + 10, (char*)statePosSeg.c_str());
	}

	glColor3f(1, 0, 1);
	drawCircle(currStateG.getPos(), 3);
	glColor3f(1, 0, 0);
	drawCircle(carDynCg.pos, 2);

	drawMapSegments();

	glPopMatrix();
	glutSwapBuffers();
}

//projects abstract states into 2D plane
void draw2DProjectedSearchPoints(){

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.7, 0.7, 0.7, 0.7);

	int w = glutGet(GLUT_WINDOW_WIDTH);
	int h = glutGet(GLUT_WINDOW_HEIGHT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0.0f, w, h, 0.0f, 0.0f, 1.0f);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glPushMatrix();
	
	glColor3f(0, 0, 0);
	for (int i = 1; i < (graphG).size(); i++){
		if (graphG[i].getMyGraphIndex() == -1)  //still unassigned
			continue;

		tPosd pointCartesianVelocity;
		pointCartesianVelocity.x = 300 + graphG[i].getVelocity().intensity *(500 / 80);
		pointCartesianVelocity.y = 300 + graphG[i].getVelocity().angle *(500 / 80);


		tPosd parentCartesianVelocity;
		parentCartesianVelocity.x = 300 + graphG[graphG[i].getParentGraphIndex()].getVelocity().intensity *(500 / 80);
		parentCartesianVelocity.y = 300 + graphG[graphG[i].getParentGraphIndex()].getVelocity().angle *(500 / 80);
		
		drawCircle(pointCartesianVelocity, 0.05);
		if (!graphG[i].getInitialState()){
			drawLine(pointCartesianVelocity.x, pointCartesianVelocity.y, parentCartesianVelocity.x, parentCartesianVelocity.y);
		}
	}

	glColor3f(1, 0, 0);
	if (graphG.size() > 0){
		tPosd initialState = { 300 + graphG[0].getVelocity().intensity *(500 / 80), 300 + graphG[0].getVelocity().angle *(500 / 80) };
		drawCircle(initialState, 0.05);
	}
	glPopMatrix();
	glutSwapBuffers();
}

void drawCircle(tPosd point, GLfloat radius)
{
	int w = glutGet(GLUT_WINDOW_WIDTH);
	int h = glutGet(GLUT_WINDOW_HEIGHT);

	int x = point.x;
	int y = h - point.y;

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

//used in old forward model
void drawCubicBezier(tPosd p0, tPosd p1, tPosd p2, tPosd p3, unsigned int numPartialPoints)
{
	int h = glutGet(GLUT_WINDOW_HEIGHT);

	tPosd* partialBezierPoints = (tPosd*) malloc(sizeof(tPosd)*numPartialPoints+1);
	double curvePercent=0.0;
	int pointsIt=0;
	while (curvePercent < 1.0){
		partialBezierPoints[pointsIt].x = ((1 - curvePercent)*(1 - curvePercent)*(1 - curvePercent)) * p0.x +
			3 * curvePercent* ((1 - curvePercent)*(1 - curvePercent)) * p1.x +
			3 * curvePercent*curvePercent* (1 - curvePercent) * p2.x +
			curvePercent*curvePercent*curvePercent* p3.x;

		partialBezierPoints[pointsIt].y = ((1 - curvePercent)*(1 - curvePercent)*(1 - curvePercent)) *p0.y +
			3 * curvePercent* ((1 - curvePercent)*(1 - curvePercent)) * p1.y +
			3 * curvePercent*curvePercent* (1 - curvePercent) * p2.y +
			curvePercent*curvePercent*curvePercent* p3.y;

		curvePercent += (1.0 / numPartialPoints); 
		pointsIt++;
	}

	glLineWidth(0.5);
	for (int i = 1; i < numPartialPoints+1; i++){
		
		glBegin(GL_LINES);
			glVertex2f(partialBezierPoints[i].x, h - partialBezierPoints[i].y);
			glVertex2f(partialBezierPoints[i - 1].x, h - partialBezierPoints[i - 1].y);
		glEnd();
	}
}

void drawLine(double initialPointX, double initialPointY, double finalPointX, double finalPointY)
{
	int h = glutGet(GLUT_WINDOW_HEIGHT);

	glLineWidth(0.5);
	glBegin(GL_LINES);
		glVertex2f(initialPointX,h - initialPointY);
		glVertex2f(finalPointX,h - finalPointY);
	glEnd();

}


void drawMapSegments()
{
	glColor3f(0.8, 0.2, 0);
	t3Dd* vertexes = segmentArrayG[0].vertex;
	int j = 0;
	while (j < nsegsG){
		vertexes = segmentArrayG[j].vertex;
		drawCircle({ vertexes[0].x, vertexes[0].y, vertexes[0].z }, 1);
		drawCircle({ vertexes[1].x, vertexes[1].y, vertexes[1].z }, 1);
		drawCircle({ vertexes[2].x, vertexes[2].y, vertexes[2].z }, 1);
		drawCircle({ vertexes[3].x, vertexes[3].y, vertexes[3].z }, 1);
		j++;
	}

}

GLuint loadTexture(const char * filename)
{

	GLuint texture;
	int width, height;
	FILE * file;
	file = fopen(filename, "rb");
	if (file == NULL) {
		return 0;
	}
	width = 512;
	height = 512;

	unsigned char data[512 * 512 * 3];
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

void printTextInWindow(int x, int y, char *string)
{
	int length, i;
	length = strlen(string);
	glRasterPos2i(x, y); //find where to start printing (pixel information)
	for (i = 0; i < length; i++)
	{
		glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, string[i]); // Print a character of the text on the window
	}

}



