CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
INCLUDES = -I./include `pkg-config --cflags opencv4` 
LDFLAGS = `pkg-config --libs opencv4` -lpthread

SRC_DIR = src
BUILD_DIR = build
INCLUDE_DIR = include
BIN_DIR = bin

TARGET = $(BIN_DIR)/drone_controller

SOURCES = $(wildcard $(SRC_DIR)/*.cpp)
OBJECTS = $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(SOURCES))

all: directories $(TARGET)

directories:
	@mkdir -p $(BUILD_DIR)
	@mkdir -p $(BIN_DIR)

$(TARGET): $(OBJECTS)
	$(CXX) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

run: all
	$(TARGET)

clean:
	rm -rf $(BUILD_DIR) $(BIN_DIR)

.PHONY: all directories clean run