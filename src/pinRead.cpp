//
//  pinRead.cpp
//  pinRead
//
//  Created by Matthew Bailey, Peter Quan and Yosub Lee on 06/15/14.
//  Copyright (c) 2014 Matthew Bailey, Peter Quan and Yosub Lee. All rights reserved.
//

/**
  Our team is using this class to open ADC pin 6 on the Gumstix to communicate
  with an Ultrasonic Range Finder. However, we designed this class to enable 
  the user to specify which pin they want to open.
*/

#include "pinRead.h"

pinRead::pinRead(){
	adc_out = 0;
	initialized = false;

	for(int i = 0; i < 7; i++){
		adcVal2[i] = 0;
	}
}

/*
	Specify a new path to access a different pin
*/
pinRead::pinRead(string newPath){
  initialized = true;
	pinPath = newPath;
}

pinRead::~pinRead(){
	delete fp;
}

int pinRead::connectRead(){
  if (!initialized){
    if (( fp = fopen("/sys/class/hwmon/hwmon0/device/in6_input", "r")) == NULL){
         printf("Can not open hwmon0 \n");
         exit(1);
     }
  }

  if(initialized){
    if ((fp = fopen(pinPath.c_str(), "r")) == NULL){
      printf("Can not open hwmon0 \n");
      exit(1);
    }
  }

  rewind(fp);
  fread(&read_value, sizeof(char), 3 , fp);
  strcpy(adcVal2 , read_value);
  fclose(fp);

  adc_out = atoi(adcVal2);
  cout << "adc = " << adc_out << endl;
  return adc_out;
}