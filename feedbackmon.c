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
	short Angle[SERVOS];
} ServoStates;

pthread_t RXthread, TXthread;
float* rawServoSamples[SERVOS];
float* servoSamples[SERVOS];
KalmanFilter servoFilters[SERVOS];

int USB;
int currentSample[SERVOS], currentServo = 0;
int overloaded[SERVOS] = {0};

float transformSample(int sample){
	return (sample * 2 / 1024.0f) - 1.0f;
}

int oscillating = 0;
float oscTime = 0;

int ReadRX(ServoStates* st){
	int retVal = 0;
	short* a = st->Angle;
	char buf[128];
	
	if(retVal = serialport_read_until(USB, buf, '\n', 128, 1000)){
		sscanf(buf, "%d,%d,%d,%d,%d,%d,%d,%d",
			&a[0], &a[1], &a[2], &a[3],
			&a[4], &a[5], &a[6], &a[7]
		);
		//printf("%s\n", buf);

		return retVal;
	}

	return -1;
}

int WriteTX(ServoStates* st){
	short* a = st->Angle;
	char buf[128];
	
	sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d\r\n",
		a[0], a[1], a[2], a[3],
		a[4], a[5], a[6], a[7]
	);
	return serialport_write(USB, buf);
}

ServoStates txState = {
	{
		60, 60, 60, 60,
		60, 60, 60, 60
	}
};
void* serialTX(void* params){
	while(1){
		int servo = -1, value = -1;
		char buf0[128], buf1[128];

		if(oscillating){
			int i = 0;
			oscTime += 0.016f;
			int angle = 10 + (int)(50 * (cos(oscTime) + 1) / 2.0f);

			for(i = 4; i--;){
				if(overloaded[i]){
					txState.Angle[i] = 10;
					continue;
				}
				txState.Angle[i] = angle;
			}

			WriteTX(&txState);
			usleep(35000);

			continue;
		}

		printf("Servo: "); scanf("%d", &servo);
		if(servo > 0 && servo <= SERVOS){
			printf("Value: "); scanf("%d", &value);
			if(value >= 0 && value <= 180){
				// send
				txState.Angle[servo - 1] = value;

				if(WriteTX(&txState) < 0)
					serialport_flush(USB);
			}
		}
		else if(servo == -1){
			ServoStates txState = {
				{90,90,90,90 ,90,90,90,90}
			};
			WriteTX(&txState);
		}
		else if(servo == -2){
			ServoStates txState = {
				{50,50,50,50 ,50,50,50,50}
			};
			WriteTX(&txState);

		}
		else if(servo == -3){
			char command[128];
			int value;
			printf("Value: "); scanf("%d", &value);
			
			{
				ServoStates txState = {
					{value,value,value,value ,value,value,value,value}
				};
				WriteTX(&txState);
			}
		}
		else if(servo == -4) oscillating = 1;
	}
}

void* serialRX(void* params){
	ServoStates state;
	char buf[1024];
	float dt = 1.6f;
	int started = 0;

	while(1){
		int i = -1;
		if(ReadRX(&state)){
			for(i = 0; i < 8; i++){
				int j = 0;
				int sampInd = currentSample[i];
				float var = 0;
				float samp = Update(
					&servoFilters[i],
					rawServoSamples[i][sampInd] = transformSample(state.Angle[i]),
					dt
				);
				servoSamples[i][sampInd] = samp;
				++currentSample[i];
				currentSample[i] %= SAMPLES;

				// determine variance
				for(j = 0; j < 20; j++){
					int ind = sampInd - j < 0 ? sampInd - j + SAMPLES : sampInd - j;
					float raw = rawServoSamples[i][ind];
					float flt = servoSamples[i][ind];

					register float term = raw - flt;
					var += term * term;
				}
				var *= 1/20.0f;

				if(var < 0.0006f) started = 1;
				if(var >= 0.0006f && started) overloaded[i] = 1;//!overloaded[i];

				printf("%+f ", var);
			}
			printf("\n");
		}
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
	//USB = serialport_init("/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_85231363236351810222-if00", 9600);
	USB = serialport_init("/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_6493832333135180F002-if00", 9600);
	if(USB){
		printf("Connected!\n");
		serialport_flush(USB);
		// serialport_write(USB, "0");
		// serialport_write(USB, "20");
	}
	else
		return;

	// allocate sample space
	for(i = SERVOS; i--;){
		servoSamples[i] = (float*)malloc(sizeof(float) * SAMPLES);
		rawServoSamples[i] = (float*)malloc(sizeof(float) * SAMPLES);

		bzero(servoSamples[i], sizeof(float) * SAMPLES);
		bzero(rawServoSamples[i], sizeof(float) * SAMPLES);
		bzero(&servoFilters[i], sizeof(KalmanFilter));

		Reset(&servoFilters[i],
			0.5f, 0.5f,
			1.0f,
			0.5f, 1
		);
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
