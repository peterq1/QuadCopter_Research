//
//  comm_UART.h
//  comm_UART
//
//  Created by Matthew Bailey, Peter Quan & Yosub Lee on 7/10/14.
//  Copyright (c) 2014 Matthew Bailey, Peter Quan & Yosub Lee. All rights reserved.
//

#ifndef COMM_UART
#define COMM_UART

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>
#include <cstring>
#include <string>

using namespace std;

class communication_UART{
public: 
        communication_UART();
        communication_UART(string newPath);
        ~communication_UART();
        void readLoop_UART();
        void writeLoop_UART();

private:
        int Set_Interface_Attribs (int, int, int);

        int fd;
        char buf[128];
        int n;
        string response;
        string portPath;
        int breaker;
        bool flag;      // only for testing
        bool initialized;
}; 


#endif /* defined(COMM_UART) */

