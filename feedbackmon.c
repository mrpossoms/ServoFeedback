#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>

#include "arduino-serial-lib.h"
#include "filtering.h"

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <GL/freeglut_ext.h>

#define SAMPLES 100
#define SERVOS 8

typedef struct{
	short Servos[8];
} StateMsg;

pthread_t RXthread, TXthread;
float* servoSamples[SERVOS];
KalmanFilter servoFilters[SERVOS];

int USB;
int currentSample[SERVOS], currentServo = 0;

StateMsg machineState, currentState;

float transformSample(int sample){
	return (sample * 2 / 1024.0f) - 1.0f;
}

int rxStart = 0;

int oscillating = 0;
float oscTime = 0;
void* serialTX(void* params){
	int i = 0;
	while(1){
		int servo = -1, value = -1;
		char buf0[128], buf1[128];

		if(oscillating){
			char command[128];
			oscTime += 0.016f;
			int angle = 10 + (int)(50 * (cos(oscTime) + 1) / 2.0f);
			for(i = 0; i < 4; i++)
				machineState.Servos[i] = angle;

			printf("%d\n", angle);
			TxMsg(&machineState, sizeof(StateMsg));

			continue;
		}

		printf("Servo: "); scanf("%d", &servo);
		if(servo > 0 && servo <= SERVOS){
			printf("Value: "); scanf("%d", &value);
			if(value >= 0 && value <= 180){
				// send
				StateMsg msg = {
					{90,90,90,90,90,90,90,90}
				};

				msg.Servos[servo] = value;
				TxMsg(&msg, sizeof(StateMsg));
				//serialport_flush(USB);
			}
			rxStart = 1;
		}
		else if(servo == -1){
			StateMsg stand = {
				{0,0,0, (short)random(), 0,0,0,0}
			};

			TxMsg(&stand, sizeof(StateMsg));
			rxStart = 1;
		}
		else if(servo == -2){
			serialport_write(USB, "1\r\n50\r\n2\r\n50\r\n3\r\n50\r\n4\r\n50\r\n");
		}
		else if(servo == -3){
			char command[128];
			int value, i;

			printf("Value: "); scanf("%d", &value);
			for(i = SERVOS; i--; machineState.Servos[i] = value);
			TxMsg(&machineState, sizeof(StateMsg));
		}
		else if(servo == -4) oscillating = 1;
	}
}

void* serialRX(void* params){
	char buf[1024];
	float dt = 0.016f;

	while(1){
		int i = -1,rx = 0;
		if(rxStart)
		if((rx = RxMsg(&currentState, sizeof(StateMsg))) > 0){
		//printf("-----------------------\n");
			for(i = 0; i < SERVOS; i++){
					//printf("%d -> %d\n", currentServo, i);
					float samp = Update(&servoFilters[i], transformSample(currentState.Servos[i]), dt);
					servoSamples[i][currentSample[i]] = samp;
					++currentSample[i];
					currentSample[i] %= SAMPLES;
					printf("%d ", (int)currentState.Servos[i]);
			}
			printf("\n");
		}
		//else
		//	printf("Failed to read... %d\n", rx);
	}	
}

void drawSamples(float* samples, float* color){
	int i = SAMPLES - 1;
	float dx = 2.0f / SAMPLES;

	glBegin(GL_LINE_STRIP);
		glColor3f(color[0], color[1], color[2]);
		for(;i--;){
			float x = i * dx, xo = (i + 1) * dx;
			glVertex3f(xo - 1, samples[i+1], 0);
			glVertex3f(x  - 1, samples[i], 0);
		}
	glEnd();
}

void keyboard( unsigned char key, int x, int y ){
    switch ( key ) {
    case 033:
        case 'q':
        exit( EXIT_SUCCESS );
        break;
    }
}

void display( void ){
	float colors[24] = {
		1, 0, 0,
		0, 1, 0,
		0, 0, 1,
		1, 1, 0,

		0.5f, 0, 0,
		0, 0.5f, 0,
		0, 0, 0.5f,
		0.5f, 0.5f, 0
	};
	float x = currentSample[0] * (2.0f / SAMPLES) - 1.0f;
	int i = SERVOS;
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	for(;i--;)
		drawSamples(servoSamples[i], colors + i * 3);

	glBegin(GL_LINES);
	glColor3f(1, 1, 1);
	glVertex3f(x, -1, 0);

	glColor3f(1, 1, 1);
	glVertex3f(x, 1, 0);
	glEnd();

	glFlush();
	glutPostRedisplay();
}

int main(int argc, char *argv[] ){
	int i, j;

	glutInit( &argc, argv ); 
	glutInitDisplayMode( GLUT_RGBA | GLUT_DEPTH ); 
	glutInitWindowSize( 512, 512 ); 
	glutCreateWindow( "OpenGL example" ); 

	// setup serial comms
	if(!OpenCom("/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_6493832333135180F002-if00", 9600))
	//if(!OpenCom("/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_85231363236351810222-if00", 9600))
		return;

	// allocate sample space
	for(i = SERVOS; i--;){
		servoSamples[i] = (float*)malloc(sizeof(float) * SAMPLES);
		bzero(servoSamples[i], sizeof(float) * SAMPLES);
		bzero(&servoFilters[i], sizeof(KalmanFilter));

		Reset(&servoFilters[i],
			0.5f, 0.5f,
			1.0f,
			0.5f, 1
		);

		machineState.Servos[i] = 60;
	}

	pthread_create(
		&RXthread,
		NULL,
		serialRX,
		NULL
	);

	pthread_create(
		&TXthread,
		NULL,
		serialTX,
		NULL
	);

    // initialize glew: 
    int glew_err = glewInit(); 
    if(glew_err != GLEW_OK) 
            fprintf(stderr, "GLEW Error: %s\n", glewGetErrorString(glew_err)); 

    glutDisplayFunc( display ); 
    glutKeyboardFunc( keyboard ); 
 
    glutMainLoop();
    return 0;

}
