CXX=g++
CC=gcc
INC=./libusb/libusb

all:test_feynman

test_feynman:main.o feynman_sdk.o ring_queue.o yuv_rgb.o
	$(CXX) $^ -o $@ -pthread -lusb-1.0 -L. -ludev

main.o:main.cpp
	$(CXX) -c $^ -o $@ -pthread

feynman_sdk.o:feynman_sdk.cpp
	$(CXX) -I$(INC) -c $^ -o $@ -pthread

ring_queue.o:ring_queue.cpp
	$(CXX) -c $^ -o $@ -pthread

yuv_rgb.o:yuv_rgb.c
	$(CC) -c $^ -o $@ -pthread
clean:
	rm -f *.o test_feynman
