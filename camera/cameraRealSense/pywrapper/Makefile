CFLAGS=-Wall -Wextra -Werror -Wpedantic
IFLAGS = -I/usr/include/opencv4/ -I/usr/include/librealsense2/ -I/usr/include/python3.12/
EXT=$(shell python3-config --extension-suffix)

all:
	g++ -shared $(CFLAGS) -std=c++17 -fPIC \
	$(IFLAGS) \
	$(shell python3 -m pybind11 --includes) \
	$(shell python3-config --cflags) \
	RealSense_bindings.cpp ../src/RealSense.cpp -o ../lib/realsense$(EXT) \
	-lrealsense2 -lopencv_core -lopencv_imgproc -lopencv_highgui -lpython3.12 \
	$(shell python3-config --ldflags)
