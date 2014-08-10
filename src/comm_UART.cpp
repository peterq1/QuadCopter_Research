//
//  comm_UART.cpp
//  comm_UART
//
//  Created by Matthew Bailey, Peter Quan and Yosub Lee on 7/10/14.
//  Copyright (c) 2014 Matthew Bailey, Peter Quan and Yosub Lee. All rights reserved.
//

/**
    Will implement with vision recognition system. The 
    camera Gumstix will send data via UART to the flight
    controller Gumstix. 

    Create a thread to read data every 5 Hz
*/

#include "comm_UART.h"

using namespace std;

communication_UART::communication_UART(){
    initialized = false;
    flag = false;
    breaker = 0;
    n = 0;
    for (int i =0;i < 128; i++)
        buf[i] = NULL;
}

communication_UART::communication_UART(string newPath){
    initialized = true;
    portPath = newPath;
}

communication_UART::~communication_UART(){
}

/**
Write data to cmd[] to send via UART
*/
void communication_UART::writeLoop_UART(){
    unsigned char cmd[] = "INIT \r";
    int n_written = 0;

    do{
        n_written = write(fd, cmd, sizeof(cmd) - 1);
    } 
    while(cmd[n_written-1] != '\r' && n_written > 0);
}

/**
Reads from fd and stores in a buffer
*/
void communication_UART::readLoop_UART(){
    if(!initialized){ 
        fd = open("/dev/ttyO0", O_RDWR | O_SYNC | O_NOCTTY);
    }

    if(initialized){
        fd = open(portPath.c_str(), O_RDWR | O_SYNC | O_NOCTTY);
    }

    if (fd == -1){
        cout << " fd = " << fd << endl;
    }
    else
        cout << " good and fd = " << fd << endl;

    Set_Interface_Attribs (fd, B115200, 0);

    while(flag){
        n = read(fd, buf, 1);
        cout << " n is " << n << endl;
        response.append(buf);
 
        // For Testing
        if(breaker > 20){
            flag = false;
        }
        breaker++;
    }

    if(n < 0){
        cout << "Error Reading " << endl;
    }
    else if(n == 0){
        cout << "Read Nothing ! " << endl;
    }
    else{
        cout << "Response: " << response;
    }
    cout << "Response : " <<  response << endl;
    close(fd);

}

/*
    Pulled from open source code
    Specifies the file directory, transfer speed and parity bit
    @param fd, speed, par
*/
int communication_UART::Set_Interface_Attribs (int fd, int speed, int par) {
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        //error_message ("error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive breaas \000 chars
    tty.c_iflag &= ~IGNBRK;         // ignore break signal
    tty.c_lflag = 0;                // no signaling chars, no echo,
// no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
 //   tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        //error_message ("error %d from tcsetattr", errno);
        return -1;
    }
}

