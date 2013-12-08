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

pthread_t RXthread, TXthread;
float* servoSamples[SERVOS];
KalmanFilter servoFilters[SERVOS];

int USB;
int currentSample[SERVOS], currentServo = 0;

float transformSample(int sample){
	return (sample * 2 / 1024.0f) - 1.0f;
}

int oscillating = 0;
float oscTime = 0;
void* serialTX(void* params){
	while(1){
		int servo = -1, value = -1;
		char buf0[128], buf1[128];

		if(oscillating){
			char command[128];
			oscTime += 0.016f;
			int angle = 10 + (int)(50 * (cos(oscTime) + 1) / 2.0f);
			sprintf(
				command,
				"1\r\n%d\r\n2\r\n%d\r\n3\r\n%d\r\n4\r\n%d\r\n",
				angle, angle, angle, angle
			);
			printf("%d\n", angle);
			serialport_write(USB, command);
			serialport_flush(USB);
			usleep(60000);
			continue;
		}

		printf("Servo: "); scanf("%d", &servo);
		if(servo > 0 && servo <= SERVOS){
			printf("Value: "); scanf("%d", &value);
			if(value >= 0 && value <= 180){
				// send
				sprintf(buf0, "%d\r\n", servo);
				serialport_write(USB, buf0); // select servo
				printf("%s\n", buf0);

				sprintf(buf1, "%d\r\n", value);
				serialport_write(USB, buf1); // write value
				printf("%s\n", buf1);
				serialport_flush(USB);
			}
		}
		else if(servo == -1){
			serialport_write(USB, "1\r\n90\r\n2\r\n90\r\n3\r\n90\r\n4\r\n90\r\n");
		}
		else if(servo == -2){
			serialport_write(USB, "1\r\n50\r\n2\r\n50\r\n3\r\n50\r\n4\r\n50\r\n");
		}
		else if(servo == -3){
			char command[128];
			int value;
			printf("Value: "); scanf("%d", &value);
			sprintf(
				command,
				"5\r\n%d\r\n6\r\n%d\r\n7\r\n%d\r\n8\r\n%d\r\n",
				value, value, value, value
			);
			serialport_write(USB, command);
		}
		else if(servo == -4) oscillating = 1;
	}
}

void* serialRX(void* params){
	char buf[1024];
	float dt = 0.016f;

	while(1){
		int i = -1;
		serialport_read_until(USB, buf, '\n', 1024, 10);
		//printf("-----------------------\n");
		sscanf(buf, "%d", &i);
		if(i >= 0){
			if(i < 8){
				currentServo = i;
			}
			else{
				//printf("%d -> %d\n", currentServo, i);
				float samp = Update(&servoFilters[currentServo], transformSample(i), dt);
				servoSamples[currentServo][currentSample[currentServo]] = samp;
				++currentSample[currentServo];
				currentSample[currentServo] %= SAMPLES;
			}
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
		bzero(servoSamples[i], sizeof(float) * SAMPLES);
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
