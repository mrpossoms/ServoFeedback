all:
	gcc *.c -o feedbackmon libfeedback.a -lpthread -lm -lGLEW -lglut -lX11 -lGL -lGLU -lm -lstdc++

fblib:
	gcc -c ./FeedBackLib/feedbacklib.c -o ./FeedBackLib/feedbacklib.o
	gcc -c arduino-serial-lib.c -o ./FeedBackLib/asl.o
	ar rcs libfeedback.a ./FeedBackLib/*.o
	rm ./FeedBackLib/*.o
