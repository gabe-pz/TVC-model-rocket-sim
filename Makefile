CXX = g++
CXXFLAGS = -Wall -g
LDFLAGS = -lraylib -lGL -lm -lpthread -ldl -lrt -lX11
TARGET = bin/sim
SRC = src/sim.cpp $(wildcard include/*.cpp)
HDR = $(wildcard include/*.h) 

.PHONY: run all clean

run: all
	./$(TARGET)

all: $(TARGET)

$(TARGET): $(SRC) $(HDR)
	mkdir -p bin
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS)

clean:
	rm -rf bin