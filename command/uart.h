/*
 * uart.h
 *
 *  Created on: Sep 8, 2016
 *      Author: mono
 */

#ifndef UART_H_
#define UART_H_

#include "Serial.h"
#include <iostream>
#include <errno.h>

extern int faule;

class Uart {
private:
	int uartFd;
	static char *pathDev;
	static const int baudRate = 38400;
public:
	Uart();
	~Uart();
	void SendMessage(uint8_t *data, int length);
	void ReciveMessage(uint8_t *data, int length);
	bool MessageAvail(void);
	int BufferNum(void);
	void EraseBuffer(void);
};

class Pose {
public:
	Pose(uint8_t *rawData);
	Pose(void);
	float x;
	float y;
	float theta;
    bool isReal;
};

class Encoder {
public:
	Encoder(void);
	Encoder(uint16_t *rawData);
	uint16_t leftEncoder;
	uint16_t rightEncoder;
};

class CompCom {
private:
	time_t startTime;
	uint8_t forward;
	uint8_t left;
	uint8_t right;
	uint8_t pose;
	uint8_t encoder;
	uint8_t stream;
public:
	CompCom();
	double TimeOut(void);
	void StartTimeOut(void);
	void Send(uint8_t * data, int lenght);
	void SendMotionCommand(uint16_t cm,char direction); //direction Forward = 'F' Left = 'L' Right = 'R'
	Pose SendPoseRequest(void);
	Encoder SendEncoderRequest(void);
	uint16_t Sum(uint8_t *data, int lenght);
	void StreamRequest(void);
	void Stream(void);
	Uart com;
	bool isMove;
};
#endif /* UART_H_ */
