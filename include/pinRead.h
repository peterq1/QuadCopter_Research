//
//  pinRead.h
//  pinRead
//
//  Created by Matthew Bailey, Peter Quan and Yosub Lee on 06/15/14.
//  Copyright (c) 2014 Matthew Bailey, Peter Quan and Yosub Lee. All rights reserved.
//

#ifndef PINREAD
#define PINREAD
 
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace std;

class pinRead{
public:
	pinRead();
	pinRead(string newPath);
	~pinRead();
	int connectRead();

private:
	FILE *fp;
	int adc_out;
	char adcVal2[6];
	char read_value[4];
	string pinPath;
	bool initialized;
};

#endif /* defined(PINREAD) */