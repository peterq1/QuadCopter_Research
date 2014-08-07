//	
//  mainFlight.cpp
//  mainFlight
//
//  Created by Matthew Bailey, Yosub Lee, Peter Quan, Marc Murphy, 
//	Mohammed Rezwanul Islam, Arun Rai & Xin Gan
//  Copyright (c) 2014 Yosub Lee. All rights reserved.
//

#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <thread>
#include <string>
#include <sstream>

#include <stdio.h>
#include <unistd.h>
#include <thread>
#include "vectornav.h"
#include "serialDevice.h"
#include "pololuMaestroMotorController.h"
#include "fan.h"
#include "pinRead.h"
#include "FanControl.h"

/* Change the connection settings to your configuration. */

using namespace std;

int main()
{
//	const int BAUD_RATE = 115200;
	
	FanControl fanController;

	int keyboardInput = 0;
	fanController.start();
	while(1) {
		cout << "\n\t(1).setBaseSpeed\n\t(2).set PID\n\t(Any otherNumbers). STOP!!\n\n\t ";
		cin >> keyboardInput;

		if ( keyboardInput == 1 ) {
			int newSpeed = 0;
			cout << "new Speed: ";
			cin >> newSpeed;
			fanController.setBaseSpeed(newSpeed);
			system("clear");
			cout << "Current BaseSpeed = " << newSpeed << endl;
		}
		else if ( keyboardInput == 2) {
			double p,i,d;
			cout << "\nP = ";
			cin >> p;
			cout << "\nI = ";
			cin >> i;
			cout << "\nD = ";
			cin >> d;
			system("clear");
			fanController.setPidAll(p, i, d); 

			fanController.displaySetting();
		}
		else {
			fanController.stop();
			break;
		}
	}


	cout << "Stopped.\n";

	return 0;
}

