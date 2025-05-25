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

# Default target: build and install service
all: $(OUT) install-service

# Link objects into executable
$(OUT): $(OBJ)
	$(CXX) $(CXXFLAGS) $(OBJ) -o $(OUT) $(LIBS)

# Compile each .cpp to .o
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Clean build artifacts
clean:
	rm -f $(OBJ) $(OUT)

# Install systemd user service
#install-service:
#	@mkdir -p ~/.config/systemd/user
#	@cp bos.service ~/.config/systemd/user/
#	@systemctl --user daemon-reload
#	@systemctl --user enable bos.service
#	@systemctl --user restart bos.service

# Uninstall systemd user service
#uninstall-service:
#	@systemctl --user disable bos.service || true
#	@rm -f ~/.config/systemd/user/bos.service
#	@systemctl --user daemon-reload

# Start the systemd user service
#start-service:
#	@systemctl --user start bos.service

# Stop the systemd user service
#stop-service:
#	@systemctl --user stop bos.service || true

# Restart the systemd user service
#restart-service:
#	@systemctl --user restart bos.service

# Check status of systemd user service
#status-service:
#	@systemctl --user status bos.service

# Debug: stop the service and run gdb
#debug: stop-service $(OUT)
#	@gdb $(OUT)

#stop:
#	systemctl --user stop bos.service || true
#	-pkill -f HF1R0x-Firmware/src/app

.PHONY: all clean install-service uninstall-service start-service stop-service restart-service status-service debug
