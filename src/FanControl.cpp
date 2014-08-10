//
//  FanControl.cpp
//  FanControl
//
//  Created by Matthew Bailey, Yosub Lee, Peter Quan, Marc Murphy,   
//	Mohammed Rezwanul Islam, Arun Rai & Xin Gan on 7/23/14.
//
//  Copyright (c) 2014 Matthew Bailey, Yosub Lee, Peter Quan, Marc Murphy, 
//	Mohammed Rezwanul Islam, Arun Rai & Xin Gan. All rights reserved.
//

#include "FanControl.h"

/**
Constructor
*/
FanControl::FanControl(int BAUD_RATE) {
	logging = false;
	running = false;
	
	this->BAUD_RATE = BAUD_RATE;
	sampleRate = 30;	//in milliseconds
	baseSpeed = 1200;
	pidOutLimitMax = 100;
	pidOutLimitMin = -100;
	
	speedY_1 = baseSpeed;
	speedY_2 = baseSpeed;
	speedX_3 = baseSpeed;
	speedX_4 = baseSpeed;
	
	// Initialize
	initVN100();
	initFanController();
	pid_X = new PID();
	pid_Y = new PID();
	pid_Z = new PID();

	pGain = 0.1;
	iGain = 0;
	dGain = 0;	
	// default setting for PID
	pid_X->setPoint(0);
	pid_Y->setPoint(0);
	pid_Z->setPoint(0);
	pid_X->setTunings(pGain, iGain, dGain);
	pid_Y->setTunings(pGain, iGain, dGain);
	pid_Z->setTunings(pGain, iGain, dGain);
	pid_X->setOutputLimits(pidOutLimitMin, pidOutLimitMax);
	pid_Y->setOutputLimits(pidOutLimitMin, pidOutLimitMax);
	pid_Z->setOutputLimits(pidOutLimitMax, pidOutLimitMax);
	setSampleRate(sampleRate);	
	
	loggingOn();
	
}

/**
destructor
*/
FanControl::~FanControl() {
	finishLogging();
	running = false;
	pidThread.join();
	cout << pidThread.joinable() << endl;
	delete Log;
	delete pid_X;
	delete pid_Y;
	delete pid_Z;
	delete fanController;
	delete motorController;
	cout << "Fan Controller Finished" << endl;
}

/*
Initialize VN100 (AHRS)
*/
void FanControl::initVN100() {
	const char *COM_PORT = "/dev/ttyUSB0";
	if ( vn100_connect(&vn100, COM_PORT, BAUD_RATE) == VNERR_PERMISSION_DENIED) {
		printf("Current user does not have permission to open the COM port.\n");
		printf("Try running again using 'sudo'.\n");
	}		
	else {
		cout << "[VN100] Initialized." << endl;
	}
}

/*
Initialize Motor Controller
*/
void FanControl::initFanController() {
	string pololuPath = "/dev/ttyACM";
	
	int postFix = 0;
	
	while(postFix < 10){
		string newPath = pololuPath + to_string(postFix);
		if (open(newPath.c_str(), O_RDWR | O_NONBLOCK) > 0) {
			pololuPath = newPath;
			break;
		}
		postFix++;
	}
	
	motorController = new PololuMaestroMotorController(12, pololuPath.c_str());
	fanController = new FanController(motorController);
}

/*
@param sampleRate
*/
void FanControl::setSampleRate(double sampleRate) {
	pid_X->setSampleTime(sampleRate);
	pid_Y->setSampleTime(sampleRate);
	pid_Z->setSampleTime(sampleRate);
}

void FanControl::setBaseSpeed(int speed) {
	baseSpeed = speed;
}

/*
Log file will be saved in 'Log' folder.
*/
void FanControl::loggingOn() {
	time_t timer;
	char buffer[25];
	struct tm* tm_info;
	
	time(&timer);
	tm_info = localtime(&timer);
	
	strftime(buffer, 25, "./Log/%F-%H-%M", tm_info);
	Log = new ofstream(buffer, ios::out);
	*Log << "PID Gains," << pGain << "," << iGain << "," << dGain << "\n";
	*Log << "BaseSpeed," << baseSpeed << "\n";
	*Log << "Pitch,Roll,Yaw,Pid_output_X,pid_output_Y,pid_output_Z\n";
	logging = true;	
}


/**
Save new PID setting to a txt file
*/
void FanControl::saveConfig() {
	ofstream config("./config/config.txt");
	
	// Format of Config file
	// basespeed,Limit_max,Limit_min,
	// PID_X	=p,i,d,
	// PID_Y	=p,i,d,
	// PID_Z	=p,i,d,

	config << baseSpeed <<"," << pidOutLimitMax << "," << pidOutLimitMin << 
	"," << pid_X->getP() << "," << pid_X->getI() << "," << pid_X->getD() << 
	"," << pid_Y->getP() << "," << pid_Y->getI() << "," << pid_Y->getD() << 
	"," << pid_Z->getP() << "," << pid_Z->getI() << "," << pid_Z->getD(); 

	cout << "[New setting SAVED]\n";	
	config.close();
}

void FanControl::restoreConfig() {
	ifstream config("./config/config.txt");

	char buf[10];
	int count = 0;
	double prev[12];
	if ( config.is_open() ){
		while( config.good() ) {
			config.getline(buf, 256, ',');
			prev[count] = atof(buf);
			count++;
		}
		
		if ( count != 12) throw count;
		
		baseSpeed = prev[0];
		pidOutLimitMax = prev[1];
		pidOutLimitMin = prev[2];
		setPidGainsX(prev[3], prev[4], prev[5]);
		setPidGainsY(prev[6], prev[7], prev[8]);
	}
	else {
		cout << "Warning: Can't find Config.txt\n";
		throw 1;
	}
}

void FanControl::finishLogging() {
	if (logging == true) 
		Log->close();	
}

/*
Writes log of AHRS and PID output
*/
void FanControl::writeLog() {
	if(logging){
		*Log << ypr.pitch << "," << ypr.roll << "," << ypr.yaw << "," <<
		m_outputX << "," << m_outputY << "," << m_outputZ << "\n";
	}
	else {
		cerr <<  "[Log is not initialized]\n";
	}

}

void FanControl::writeLog(string message) {
	if (logging) {
		*Log << message << "\n" ;
	}
	else {
		cerr << "[Log is not initialized]\n";
	}
}

/*
Send speed command to pololu channels 0 through 5
@param speed
*/
void FanControl::setMotorSpeedAll(int speed) {
	fanController->setSpeed(speed, 0);
	usleep(5000);
	fanController->setSpeed(speed, 1);
	usleep(5000);
	fanController->setSpeed(speed, 2);
	usleep(5000);
	fanController->setSpeed(speed, 3);
	usleep(5000);
	fanController->setSpeed(speed, 4);
	usleep(5000);
	fanController->setSpeed(speed, 5);
	usleep(5000);
}

/**
-Set the X and Y axis' equal to the same PID values
-X is Pitch & Y is Roll
@param p, i, d
*/
void FanControl::setPidAll(double p, double i, double d) {
	setPidGainsX(p,i,d);
	setPidGainsY(p,i,d);
	string newMsg = "\nNew PID Setting: " + to_string(p) + "," + to_string(i) + "," + to_string(d) + "\n";
	writeLog(newMsg);
	saveConfig();
}

void FanControl::computePID() {
	m_outputX = pid_X->compute(ypr.pitch);
	m_outputY = pid_Y->compute(ypr.roll);
	m_outputZ = pid_Z->compute(ypr.yaw);	

	// Test
	// cout << "PID: " << m_outputX << ", " << m_outputY << ", " << m_outputZ << " AHRS: " << ypr.pitch << ", " <<ypr.roll << ", " <<ypr.yaw << endl;;
}

/*
Apply new pid values and adjust the speed of the motors
*/
void FanControl::displaySetting() {
	cout << "/////////////////////////\n";
	cout << "// BaseSpeed   = " << baseSpeed << endl;
	cout << "// Output High = " << pidOutLimitMax << endl;
	cout << "// Output Low  = " << pidOutLimitMin << endl;
	cout << "//=======================\n";
	cout << "// X - Pitch" << endl;
	cout << "//-----------------------\n";
	cout << "// P = " << pid_X->getP() << endl;
	cout << "// I = " << pid_X->getI() << endl;
	cout << "// D = " << pid_X->getD() << endl;
	cout << "//=======================\n";
	cout << "// Y - Roll" << endl;
	cout << "//-----------------------\n";
	cout << "// P = " << pid_Y->getP() << endl;
	cout << "// I = " << pid_Y->getI() << endl;
	cout << "// D = " << pid_Y->getD() << endl;
	cout << "/////////////////////////\n";

}

/*
Having baseSpeed in the equation enables quicker reaction because it 
manipulating velocity rather than acceleration
*/
void FanControl::setMotorSpeed() {
	speedX_3 = baseSpeed - m_outputX;
	speedX_4 = baseSpeed + m_outputX;
	
	speedY_1 = baseSpeed + m_outputY;
	speedY_2 = baseSpeed - m_outputY;

	/*
	speedX_3 -= m_outputX;
	speedX_4 += m_outputX;
	speedY_1 += m_outputY;
	speedY_2 -= m_outputY;
	*/

	fanController->setSpeed(speedY_1, 1);
	fanController->setSpeed(speedY_2, 2);
	
	fanController->setSpeed(speedX_3, 0);
	fanController->setSpeed(speedX_4, 3);	
	
}

void FanControl::controlMotors() {
	while(1)
	{
		m_now = chrono::high_resolution_clock::now();

		// Time difference in Sec
		float timeDiff = (chrono::duration_cast<chrono::milliseconds>(m_now-m_lastTime).count())/1000.0f;
			

		if (vn100_getYawPitchRoll(&vn100, &ypr) != 0) {
			writeLog("VN100 Error occured");
			break;
		}
		
		//setSampleRate(timeDiff);
		computePID();
		setMotorSpeed();
		writeLog();

		m_lastTime = m_now;
		
		if ( running == false ) break;
		
	}
	
}

void FanControl::start() {
	// Setting Menu
	int input = 1;
    
    // Motor Test ///////////////////////
	cout << "[Test Motors - Speed at 1200]\n";
	while(1) {
		setMotorSpeedAll(1200);
		sleep(1);
		setMotorSpeedAll(1);
		cout << "\n(1).Again?\n(ANY).Next Step \n";
		cin >> input;
		cout << endl;
		if ( input != 1 ) break;
	}

	input = 0;
		
    // Pid Setting ///////////////////////
	cout << "\n\t[Do you want to set PID values?]\n(1).Yes\n(ANY). use previous values\n";
	cin >> input;
	if ( input == 1)
		setPID();
	else {
		cout << "Restoring Setting ...\n";
		
		try {
			restoreConfig();
		} catch (int e) {
			if ( e == 1) {
				cout << "\nPrevious Configuration Does not exist!\n";
			} else{
				cout << "\nWarning: /config/config.txt is Broken\n";	
			}
		}
	}
	system("clear");
	cout << "\n\t Current Setting is ... \n";
	displaySetting();
	cout << "\n\t[Do you want to set PID output Limit?]\n";
	cout << "(1). Yes\n(ANY). Nah\n";
	cin >> input;
	if ( input == 1) {
		char buffer[10];
		cout << "PID output Max = ";
		cin >> buffer;
		pidOutLimitMax = atof(buffer);
		cout << "\nPid output Min = ";
		cin >> buffer;
		pidOutLimitMin = atof(buffer);
		setOutputLimitsX(pidOutLimitMin, pidOutLimitMax);
		setOutputLimitsY(pidOutLimitMin, pidOutLimitMax);
		cout<<"\n[New setting applied]\n";
		saveConfig();
	}

	running = true;
	displaySetting();
	
	// Run Thread
	cout << "\nStart in .. 3" << endl;
	sleep(1);
	cout << "\nstart in .. 2" << endl;
	sleep(1);
	cout << "\nstart in .. 1" << endl;
	sleep(1);	

	system("clear");
	pidThread = std::thread(&FanControl::controlMotors, this);
}

void FanControl::setPidGainsX(double p, double i, double d) {
	pid_X->setTunings(p,i,d);
	cout << "[PID_X, Pitch] PID gains = " << p << " " << i << " " << d << endl;
}

void FanControl::setPidGainsY(double p, double i, double d) {
	pid_Y->setTunings(p,i,d);
	cout << "[PID_Y, Roll] PID gains = " << p << " " << i << " " << d << endl;
}

/*
Specify the desired set point for the PID algorithm about the X axis
@param point
*/
void FanControl::setPointPidX(int point) {
	pid_X->setPoint(point);
	cout << "[PID_X, Pitch] the Point is set to - "<< point << endl;
}

/*
Specify the desired set point for the PID algorithm about the Y axis
@param point
*/
void FanControl::setPointPidY(int point) {
	pid_Y->setPoint(point);
	cout << "[PID_Y, Roll] the Point is set to - "<< point << endl;
}

/*
Set PID output limits to mitigate windup
	-Prevents Lag and ensures a quick PID response time
@param min, max
*/
void FanControl::setOutputLimitsX(int min, int max) {
	pid_X->setOutputLimits(min, max);
	cout << "[PID_X, Pitch] Output Low/High = "<< min << " " << max << endl;
}

/*
Set PID output limits to mitigate windup
	-Prevents Lag and ensures a quick PID response time
@param min, max
*/
void FanControl::setOutputLimitsY(int min, int max) {
	pid_X->setOutputLimits(min, max);
	cout << "[PID_Y, Roll] Output Low/High = "<< min << " " << max << endl;
}

/*
-The user will specify the desired PID gains about the X and Y axis
-Will add code to enable the user to specify different PID gains about
 each axis
 @param *pid
*/
void FanControl::setPID(PID *pid){ 
	double point;

	cout << "\n\t[Set Point]\n";
	cout << "SetPoint = ";
	cin >> point;

	cout << "\n\t[Set P, I, D coefficient]\n";
	cout << "p = ";
	cin >> pGain;
	cout << "\ni = ";
	cin >> iGain;
	cout << "\nd = ";
	cin >> dGain;
	cout << endl;

	//pid->setOutputLimits(min, max);
	pid->setTunings(pGain, iGain, dGain);
	pid->setPoint(point);
}
	
void FanControl::setPID() {
	cout << "\n\t[PID X-axis Pitch setting]\n";
	setPID(pid_X);
	
    cout << "\nDo you want to set the same PID for PID_Y (Roll) ?\n 1.(Yes)\n(any number).No\n";
	int option;
	cin >> option;
	
	if (option != 1){
		setPID(pid_Y);
	}
	else {
		pid_Y->setTunings(pid_X->getP(), pid_X->getI(), pid_X->getD());
		pid_Y->setPoint(pid_X->getPoint());
	}

	saveConfig();
	
}

void FanControl::stop() {
	running = false;
	setMotorSpeedAll(1);
	usleep(50000);
	setMotorSpeedAll(1);
}
