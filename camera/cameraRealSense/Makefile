SRC=src/main.cpp


all: build/test build/debug
CFLAGS=-Wall -Wextra -Werror -Wpedantic
IFLAGS = -I/usr/include/opencv4/ -I/usr/include/librealsense2/

clean:
	rm -rf build/*

build/test: src/main.cpp src/RealSense.cpp
	mkdir -p build
	g++ src/main.cpp src/RealSense.cpp $(CFLAGS) $(IFLAGS)`pkg-config --libs opencv4` -o $@ -lrealsense2

build/debug: src/main_debug.cpp src/RealSense_debug.cpp
	g++ src/main_debug.cpp src/RealSense_debug.cpp $(CFLAGS) $(IFLAGS)`pkg-config --libs opencv4` -o $@ -lrealsense2