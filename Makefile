all:
	gcc *.c -o feedbackmon -lpthread -lm -lGLEW -lglut -lX11 -lGL -lGLU -lm -lstdc++
