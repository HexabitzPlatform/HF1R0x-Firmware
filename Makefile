CXX = g++
CXXFLAGS = -Wall -std=c++17 -g

# Add include paths for header files
INCLUDES = -I./BOS -I./Pi_library -I./Porting -I./src

# Source files from all folders
SRC = $(wildcard src/*.cpp) \
      $(wildcard BOS/*.cpp) \
      $(wildcard Pi_library/*.cpp) \
      $(wildcard Porting/*.cpp)

# Object files
OBJ = $(SRC:.cpp=.o)

# Output binary
OUT = src/app

# External libraries
LIBS = -lgpiod -pthread


# Default target
all: $(OUT)

# Link objects into executable
$(OUT): $(OBJ)
	$(CXX) $(CXXFLAGS) $(OBJ) -o $(OUT) $(LIBS)

# Compile each .cpp to .o
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Clean build artifacts
clean:
	rm -f $(OBJ) $(OUT)

.PHONY: all clean
