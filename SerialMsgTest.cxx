#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>

#include "SerialMessages.h"

int main(int argc, char *argv[]){
	char data[128];
	char* message = "Hello pizza";
	USB_FD  = creat("./testData.dat", S_IRWXU);
	
	TxMsg(message, strlen(message));

	close(USB_FD);
	return 0;
}
