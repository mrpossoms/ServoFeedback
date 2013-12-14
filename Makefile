all:
	gcc *.c -o feedbackmon -lpthread -lm -lGLEW -lglut -lX11 -lGL -lGLU -lm -lstdc++
test:
	gcc SerialMessages.c arduino-serial-lib.c SerialMsgTest.cxx -lpthread -lm -lstdc++ -o msgTest
