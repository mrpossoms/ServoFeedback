#include "SerialMessages.h"
#include "stdio.h"

#define MSG_BUFF_SIZE 128

char MSG_ID = 0x00;
int  USB_FD = -1;

int  MSG_INDEX = 0, MSG_EXP_SIZE = -1;
char MSG_CURR_BUFF[MSG_BUFF_SIZE];

int OpenCom(const char* dev, int baud){
	USB_FD = serialport_init(dev, baud);
	return USB_FD;
}

char* CharMatch(char* data, char* substr, int len, int sublen){
	int i = 0, j = 0;

	for(i = 0; i < len; i++){
		char* c = &data[i];
		if(strncmp(c, substr, sublen) == 0) return c;
	}

	return NULL;
}

int EncMsg(void* msgOut, void* data, unsigned short len){
	char* b = msgOut;
	memcpy(b, "HEAD", 4); b += 4;
	memcpy(b, &len, sizeof(short)); b += sizeof(short);
	memcpy(b, data, len);

	return len + sizeof(short) + 4;
}

int TxMsg(void* msg, unsigned short len){
	char buff[MSG_BUFF_SIZE] = {0};
	int i = 0, bytes = EncMsg(buff, msg, len);

	serialport_writebuffer(USB_FD, buff, bytes);

	return bytes;
}

int RxMsg(void* msgOut, unsigned short len){
	int msgSize = len + 4 + sizeof(short), i;
	char* buf = &MSG_CURR_BUFF[MSG_INDEX];
	int bytesRx = serialport_read(USB_FD, buf, msgSize - MSG_INDEX, 10);

    for(i=0;i<bytesRx;i++) printf("%d ", buf[i]);
    printf("\n");

	MSG_INDEX += bytesRx;

	if(MSG_EXP_SIZE < 0){
		void* hdr = CharMatch(MSG_CURR_BUFF, "HEAD", MSG_INDEX, 4);
		if(hdr){
			memcpy(MSG_CURR_BUFF, hdr, bytesRx);
			MSG_EXP_SIZE = (short)((int*)hdr)[1] + 4 + sizeof(short); // get the expected msg size
		}
	}

	if(MSG_BUFF_SIZE < MSG_EXP_SIZE || MSG_EXP_SIZE != msgSize){
		serialport_flush(USB_FD);

		// reset some stuff
		bzero(MSG_CURR_BUFF, MSG_BUFF_SIZE);
		MSG_EXP_SIZE = -1;
		MSG_INDEX    = 0;
	}

	printf("EXP=%d MSG_INDEX=%d bytesRx=%d\n", MSG_EXP_SIZE, MSG_INDEX, bytesRx );

	if(MSG_EXP_SIZE > 0 && bytesRx + MSG_INDEX >= MSG_EXP_SIZE){
		int finalSize = MSG_EXP_SIZE;

		// snatch the message
		memcpy(msgOut, &MSG_CURR_BUFF[4 + sizeof(short)], MSG_EXP_SIZE);

		// reset some stuff
		bzero(MSG_CURR_BUFF, MSG_BUFF_SIZE);
		MSG_EXP_SIZE = -1;
		MSG_INDEX    = 0;

		return finalSize;
	}

	return 0;
}
