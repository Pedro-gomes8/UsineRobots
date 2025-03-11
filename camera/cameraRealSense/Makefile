SRC=src/main.cpp src/RealSense.cpp


all: build/test
CFLAGS=-Wall -Wextra -Werror -Wpedantic
IFLAGS = -I/usr/include/opencv4/ -I/usr/include/librealsense2/

clean:
	rm -rf build/*

build/test: $(SRC)
	mkdir -p build
	g++ $(SRC) $(CFLAGS) $(IFLAGS)`pkg-config --libs opencv4` -o $@ -lrealsense2