#ifndef SERIAL_MSGS
#define SERIAL_MSGS
#include <string.h>
#include "arduino-serial-lib.h"

extern char MSG_ID;
extern int  USB_FD;

int OpenCom(const char* dev, int baud);
int EncMsg(void* msgOut, void* data, unsigned short len);
int TxMsg(void* msg, unsigned short len);
int RxMsg(void* msgOut, unsigned short len);

#endif