/*
 * uart.cpp
 *
 *  Created on: Sep 8, 2016
 *      Author: mono
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>

#include "uart.h"
#include "Serial.h"

#define TIME_OUT 0.01

//#define DEBUG 1

char* Uart::pathDev = "/dev/ttyAMA0";


Uart::Uart() {
	uartFd = serialOpen(pathDev,baudRate);
	if(uartFd < 0) {
		std::cout<<"Serial Open Fail: "<<strerror(errno)<<std::endl;
	}
	EraseBuffer();
}

void Uart::EraseBuffer() {
	serialFlush(uartFd);
}

bool Uart::MessageAvail(void) {
	if( serialDataAvail(uartFd) == -1 ) {
		std::cerr<<"Uart Buffer error"<<std::endl;
	}
	if( serialDataAvail(uartFd) == 0 ) {
		return false;
	}
	else {
		return true;
	}
}

void Uart::SendMessage(uint8_t *data, int length) {
	write(uartFd, data,length );
}


// Before call ReciveMassage, you need check received number of char with BufferNum()
void Uart::ReciveMessage(uint8_t* data, int lenght) {
	if ( read(uartFd, data, lenght) == -1 ) {
		std::cout<<"Error ReciveMessage: "<<strerror(errno)<<std::endl;
	}
}

int Uart::BufferNum(void) {
	int buf = serialDataAvail(uartFd);
	if ( buf == -1 ) {
		std::cout<<uartFd;
		std::cout<<"Error BufferNum: "<<strerror(errno)<<std::endl;
		return 0;
	}
	else {
		return buf;
	}
}

Uart::~Uart() {
	close(uartFd);
}


Pose::Pose(uint8_t *rawData) {
	float tempBuf[3];
	memcpy(&tempBuf[0],rawData,12);
	x = tempBuf[0];
	y = tempBuf[1];
	theta = tempBuf[2];
}

Pose::Pose(void):x(0), y(0), theta(0), isReal(true) {

}

Encoder::Encoder(void) {
	leftEncoder = 0;
	rightEncoder = 0;
}

Encoder::Encoder(uint16_t *rawData) {
	leftEncoder = rawData[0];
	rightEncoder = rawData[1];
}

CompCom::CompCom():com(), forward(1), left(2), right(3), pose(8), encoder(5), stream(6), isMove(false) {
	if( com.BufferNum() != 0 ) {
		std::cerr<<"Initiate Buffer Num Not Zero"<<std::endl;
		com.EraseBuffer();
	}
}

void CompCom::StartTimeOut() {
	startTime = time(0);
}

double CompCom::TimeOut() {
	return difftime( time(0), startTime);
}

using namespace std;

void CompCom::Send(uint8_t *data, int lenght) {
#ifdef DEBUG
	int Num;
	char *debug;
#endif
	uint8_t ack;
	int buffNum;
	com.SendMessage(data,lenght);
	char ackMesg[2];
	StartTimeOut();
	while( true ) {
		buffNum = com.BufferNum();
		if( buffNum > 0 ) {
			break;
		}
		if( TimeOut() > TIME_OUT) {
			std::cerr<<"TimeOut during sending"<<buffNum<<std::endl;
			break;
		}
	}
	usleep(2400);
	if( com.BufferNum() == 1 ) {
		com.ReciveMessage(&ack,1);
#ifdef DEBUG
		cout<<"Ack: "<<(int)ack<<endl;
#endif
		com.EraseBuffer();
		return;
	}
#ifdef DEBUG
	Num = com.BufferNum();
	debug = new char[Num];
	com.ReciveMessage((uint8_t*)debug,Num);
	cout<<"NotAck: "<<debug<<endl;
	cout<<"NotAck: "<<(int)debug[0]<<" "<<(int)debug[1]<<endl;
	delete[] debug;
#endif
	usleep(2400);
	cout<<"Resend"<<endl;
	com.EraseBuffer();
	com.SendMessage(data,lenght);
	StartTimeOut();
	while( true ) {
		if( com.BufferNum() > 0 ) {
			break;
		}
		if( TimeOut() > TIME_OUT) {
			std::cerr<<"TimeOut during sending: "<<com.BufferNum()<<std::endl;
			break;
		}
	}
	usleep(2400);
	if( com.BufferNum() == 1 ) {
		com.EraseBuffer();//rec 1 byte = ACK
		return;
	}

	//com.EraseBuffer();
	std::cerr<<"Uart com is dead: "<<com.BufferNum()<<std::endl;
}

void CompCom::SendMotionCommand(uint16_t cm,char direction) {
	uint8_t cmSend[5];
	uint8_t crcSend[2];
	uint16_t tempcm = cm;
	uint8_t *tempPoint = (uint8_t*)&tempcm;
	uint16_t crc;
	memcpy(&cmSend[3],tempPoint,2);
	crc = (uint16_t)cmSend[3] + (uint16_t)cmSend[4];
	uint8_t *tempCrc = (uint8_t*)&crc;
	memcpy(&cmSend[1],tempCrc,2);
	switch(direction) {
		case 'F': cmSend[0] = forward; break;
		case 'L': cmSend[0] = left; break;
		case 'R': cmSend[0] = right; break;
		default: std::cerr<<"Wrong direction"<<std::endl; break;
	}
	/*std::cout<<"command"<<(int)cmSend[0]<<std::endl;
	std::cout<<"cm:"<<cm<<std::endl;
	std::cout<<"crc:"<<crc<<std::endl;
	uint16_t recast;
	memcpy(&recast,&cmSend[3],2);
	std::cout<<"Recast cm: "<<recast<<std::endl;
	uint16_t recrc;
	memcpy(&recrc,&cmSend[1],2);
	recrc = recrc & 0b0000111111111111;
	std::cout<<"Recast crc: "<<recrc<<std::endl;
	uint8_t command = 0;
	memcpy(&command,&cmSend[0],1);
	std::cout<<"re command: "<<(int)command<<std::endl;*/
	Send(&cmSend[0],1);
	Send(&cmSend[1],4);
	isMove = true;
}

uint16_t CompCom::Sum(uint8_t *data, int lenght) {
	uint16_t crc = 0;
	for(int i = 0; i < lenght; i++) {
		crc += (uint16_t)data[i];
	}
	return crc;
}

Encoder CompCom::SendEncoderRequest(void) {
	uint16_t tempBuf[3];
	uint8_t receiveBuf[6];
	Encoder err;
	for(int i = 0; i < 2; i++) {
		com.SendMessage(&encoder,1);
		StartTimeOut();
		while (true) {
			if(com.BufferNum() == 6) {
				com.ReciveMessage(&receiveBuf[0],6);
				memcpy(&tempBuf[0],&receiveBuf[0],6);
				if(tempBuf[2] == Sum(&receiveBuf[0],4)){
					Encoder enc(&tempBuf[0]);
					com.EraseBuffer();
					return enc;
				}
			}
			if( TimeOut() > TIME_OUT*2 ) {
				cerr<<"Time Out during encoder request"<<endl;
				break;
			}
		}
		com.EraseBuffer();
	}
	cerr<<"Uart dead"<<endl;
	return err;
}

void CompCom::StreamRequest(void) {
	Send(&stream,1);
}

void CompCom::Stream(void) {
	/*int Num;
	char* stream;
	Num = com.BufferNum();
	stream = new char[Num];
	com.ReciveMessage((uint8_t*)stream,Num);
	cout<<stream;
	delete[] stream;*/
	char m;
	if(com.BufferNum() > 0) {
		com.ReciveMessage((uint8_t*)&m,1);
		cout<<m;
	}
}


Pose CompCom::SendPoseRequest(void) {
	uint16_t tempBuf[1];
	uint8_t receiveBuf[14];
	Pose err;
	for(int i = 0; i < 2; i++) {
		com.SendMessage(&pose,1);
		StartTimeOut();
		while (true) {
			if(com.BufferNum() == 15) {
				com.ReciveMessage(&receiveBuf[0],15);
				memcpy(&tempBuf[0],&receiveBuf[12],2);
				if(tempBuf[0] == Sum(&receiveBuf[0],12)){
					Pose pose(&receiveBuf[0]);
					if((char)receiveBuf[14] == 's' ) {
						isMove = false;
					}
					com.EraseBuffer();
					return pose;
				}
			}
			if( TimeOut() > TIME_OUT*10 ) {
				cerr<<"Time Out during pose request"<<com.BufferNum()<<endl;
				break;
			}
		}
		com.EraseBuffer();
	}
	cerr<<"Uart dead"<<endl;
    err.isReal = false;
	return err;
}


