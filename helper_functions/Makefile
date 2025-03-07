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
OBJECTS = $(BUILD_DIR)/optitrack.o $(BUILD_DIR)/logger.o

# Test target
$(BUILD_DIR)/test_optitrack_logging: $(TEST_DIR)/test_optitrack_logging.cpp $(OBJECTS)
	$(CC) $(CFLAGS) $(TEST_DIR)/test_optitrack_logging.cpp $(OBJECTS) -o $@ $(LDFLAGS)

# Object files
$(BUILD_DIR)/optitrack.o: $(SRC_DIR)/optitrack.cpp $(INCLUDE_DIR)/optitrack.h
	$(CC) $(CFLAGS) -c $(SRC_DIR)/optitrack.cpp -o $@

$(BUILD_DIR)/logger.o: $(SRC_DIR)/logger.cpp $(INCLUDE_DIR)/logger.h
	$(CC) $(CFLAGS) -c $(SRC_DIR)/logger.cpp -o $@

# Phony targets
.PHONY: clean tests

# Main targets
tests: $(BUILD_DIR)/test_optitrack_logging
	@echo "Test target built: $(BUILD_DIR)/test_optitrack_logging"
	@echo "Run with: ./$(BUILD_DIR)/test_optitrack_logging"

clean:
	rm -f $(BUILD_DIR)/*.o $(BUILD_DIR)/test_optitrack_logging

# Help target
help:
	@echo "Available targets:"
	@echo "  tests  - Build the test_optitrack_logging test"
	@echo "  clean  - Remove built object files and executables"
	@echo "  help   - Display this help message"