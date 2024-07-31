##---------------------------------------------------------------------
## armgui Makefile
## tested on: 
## 		g++, ubuntu 20.04 # not after 6/23 changes
##		mingw, windows 10 # :)
##---------------------------------------------------------------------

# Compiler
CXX := g++
# CXX := clang++
# CXX := x86_64-w64-mingw32-g++

# Executable name
EXE := armgui

# build directory
BUILDDIR := ../build

# imgui directory
IMGUI_DIR = ../imgui

SOURCES = main.cpp #

## windows/linux
# ifeq ($(UNAME_S), Linux) #LINUX #TODO make sure 
# 	DYNAMIXEL_INCLUDE := /usr/local/include/dynamixel_sdk 

# endif
# ifeq ($(OS), Windows_NT)
# 	DYNAMIXEL_INCLUDE := include/windows

# endif

# library includes added to compiler search path 
# DYNAMIXEL_INCLUDE := /usr/local/include/dynamixel_sdk # #
# DYNAMIXEL_INCLUDE := include/windows #
#EIGEN_INCLUDE := /usr/local/include/EIGEN_INCLUDE

# Source files
# SOURCES = main.cpp #
SOURCES += gui.cpp get_ports.cpp use_dxl.cpp 
SOURCES += $(IMGUI_DIR)/imgui/imgui.cpp $(IMGUI_DIR)/imgui/imgui_demo.cpp $(IMGUI_DIR)/imgui/imgui_draw.cpp $(IMGUI_DIR)/imgui/imgui_tables.cpp $(IMGUI_DIR)/imgui/imgui_widgets.cpp
SOURCES += $(IMGUI_DIR)/backends/imgui_impl_glfw.cpp $(IMGUI_DIR)/backends/imgui_impl_opengl3.cpp

# Transforms a list of source file paths into a list of object file names
OBJS = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(notdir $(SOURCES)))))

# Compiler flags. -I is for compiler search paths, -g provides debugging, -Wall all warnings shown, -Wformat format string warnings are enabled 
# CXXFLAGS := -std=c++11 -I$(DYNAMIXEL_INCLUDE) -I$(IMGUI_DIR) -I$(IMGUI_DIR)/backends 
CXXFLAGS := -std=c++11 -I$(IMGUI_DIR) -I$(IMGUI_DIR)/backends 
CXXFLAGS += -g -Wall -Wformat

# Libraries
# LIBS = -ldxl_x64_cpp #
LINUX_DXL_LIBS = -ldxl_x64_cpp
WIN_DXL_LIBS = -L../include -ldynamixel_sdk 
# WIN_DXL_LIBS = -L../build -ldxl_x64_cpp 
LINUX_GL_LIBS = -lGL

# Detect operating system
UNAME_S := $(shell uname -s)

##---------------------------------------------------------------------
## BUILD FLAGS PER PLATFORM
##---------------------------------------------------------------------

ifeq ($(UNAME_S), Linux) #LINUX
	ECHO_MESSAGE = "Linux"
	LIBS += $(LINUX_GL_LIBS) $(LINUX_DXL_LIBS) `pkg-config --static --libs glfw3`

	CXXFLAGS += `pkg-config --cflags glfw3`
	CFLAGS = $(CXXFLAGS)
endif

ifeq ($(UNAME_S), Darwin) #APPLE
	ECHO_MESSAGE = "Mac OS X"
	LIBS += -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo
	LIBS += -L/usr/local/lib -L/opt/local/lib -L/opt/homebrew/lib
	#LIBS += -lglfw3	
	LIBS += -lglfw

	CXXFLAGS += -I/usr/local/include -I/opt/local/include -I/opt/homebrew/include
	CFLAGS = $(CXXFLAGS)
endif

ifeq ($(OS), Windows_NT)
	ECHO_MESSAGE = "MinGW"
	LIBS += $(WIN_DXL_LIBS) -lglfw3 -lgdi32 -lopengl32 -limm32

	CXXFLAGS += `pkg-config --cflags glfw3` -I/mingw64/include/eigen3
	CFLAGS = $(CXXFLAGS)
endif

##---------------------------------------------------------------------
## BUILD RULES
##---------------------------------------------------------------------

# Rule to compile source files into object files
$(BUILDDIR)/%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# Rule to compile ImGui source files into object files
$(BUILDDIR)/%.o: $(IMGUI_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# Rule to compile backends source files into object files
$(BUILDDIR)/%.o: $(IMGUI_DIR)/backends/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# Default target
all: $(BUILDDIR)/$(EXE)
	@echo Build complete for $(ECHO_MESSAGE)

# Rule to link object files into executable
$(BUILDDIR)/$(EXE): $(OBJS)
	$(CXX) -o $@ $^ $(CXXFLAGS) $(LIBS)

# Clean rule
clean:
	rm -f $(BUILDDIR)/$(EXE) $(BUILDDIR)/*.o