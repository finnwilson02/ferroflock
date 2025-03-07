# Makefile for ferroflock helper functions
CC = g++
CFLAGS = -g -Wall -I./include -I/usr/include/opencv4 -I../libs/vrpn -std=c++17
LDFLAGS = -lvrpn -lquat -pthread $(shell pkg-config --libs opencv4)

BUILD_DIR = build
SRC_DIR = src
TEST_DIR = tests
INCLUDE_DIR = include

# Ensure build directory exists
$(shell mkdir -p $(BUILD_DIR))

# Object files for main libraries
OBJECTS = $(BUILD_DIR)/optitrack.o $(BUILD_DIR)/logger.o $(BUILD_DIR)/tello_controller.o $(BUILD_DIR)/tello_imu_handler.o $(BUILD_DIR)/menu.o $(BUILD_DIR)/calibration.o

# Test targets
$(BUILD_DIR)/test_optitrack_logging: $(TEST_DIR)/test_optitrack_logging.cpp $(BUILD_DIR)/optitrack.o $(BUILD_DIR)/logger.o
	$(CC) $(CFLAGS) $(TEST_DIR)/test_optitrack_logging.cpp $(BUILD_DIR)/optitrack.o $(BUILD_DIR)/logger.o -o $@ $(LDFLAGS)

$(BUILD_DIR)/test_imu: $(TEST_DIR)/test_imu.cpp $(BUILD_DIR)/tello_controller.o $(BUILD_DIR)/tello_imu_handler.o $(BUILD_DIR)/logger.o $(BUILD_DIR)/optitrack.o $(BUILD_DIR)/menu.o $(BUILD_DIR)/calibration.o
	$(CC) $(CFLAGS) $(TEST_DIR)/test_imu.cpp $(BUILD_DIR)/tello_controller.o $(BUILD_DIR)/tello_imu_handler.o $(BUILD_DIR)/logger.o $(BUILD_DIR)/optitrack.o $(BUILD_DIR)/menu.o $(BUILD_DIR)/calibration.o -o $@ $(LDFLAGS)

# Object files
$(BUILD_DIR)/optitrack.o: $(SRC_DIR)/optitrack.cpp $(INCLUDE_DIR)/optitrack.h
	$(CC) $(CFLAGS) -c $(SRC_DIR)/optitrack.cpp -o $@

$(BUILD_DIR)/logger.o: $(SRC_DIR)/logger.cpp $(INCLUDE_DIR)/logger.h
	$(CC) $(CFLAGS) -c $(SRC_DIR)/logger.cpp -o $@

$(BUILD_DIR)/tello_controller.o: $(SRC_DIR)/tello_controller.cpp $(INCLUDE_DIR)/tello_controller.h
	$(CC) $(CFLAGS) -c $(SRC_DIR)/tello_controller.cpp -o $@

$(BUILD_DIR)/tello_imu_handler.o: $(SRC_DIR)/tello_imu_handler.cpp $(INCLUDE_DIR)/tello_imu_handler.h
	$(CC) $(CFLAGS) -c $(SRC_DIR)/tello_imu_handler.cpp -o $@

$(BUILD_DIR)/menu.o: $(SRC_DIR)/menu.cpp $(INCLUDE_DIR)/menu.h
	$(CC) $(CFLAGS) -c $(SRC_DIR)/menu.cpp -o $@

$(BUILD_DIR)/calibration.o: $(SRC_DIR)/calibration.cpp $(INCLUDE_DIR)/calibration.h
	$(CC) $(CFLAGS) -c $(SRC_DIR)/calibration.cpp -o $@

# Phony targets
.PHONY: clean tests test_optitrack test_imu

# Main targets
tests: test_optitrack test_imu
	@echo "All test targets built"

test_optitrack: $(BUILD_DIR)/test_optitrack_logging
	@echo "Test target built: $(BUILD_DIR)/test_optitrack_logging"
	@echo "Run with: ./$(BUILD_DIR)/test_optitrack_logging"

test_imu: $(BUILD_DIR)/test_imu
	@echo "Test target built: $(BUILD_DIR)/test_imu"
	@echo "Run with: ./$(BUILD_DIR)/test_imu"

clean:
	rm -f $(BUILD_DIR)/*.o $(BUILD_DIR)/test_optitrack_logging $(BUILD_DIR)/test_imu

# Help target
help:
	@echo "Available targets:"
	@echo "  tests         - Build all test targets"
	@echo "  test_optitrack - Build the test_optitrack_logging test"
	@echo "  test_imu      - Build the test_imu test"
	@echo "  clean         - Remove built object files and executables"
	@echo "  help          - Display this help message"