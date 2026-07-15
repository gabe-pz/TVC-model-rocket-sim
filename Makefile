CXX = g++
CXXFLAGS = -Wall -g
LDFLAGS = -lraylib -lGL -lm -lpthread -ldl -lrt -lX11
TARGET = bin/sim
SRC = src/sim.cpp 
INC = $(wildcard include/*.cpp)

.PHONY: run all clean

run: all
	./$(TARGET)

all: $(TARGET)

$(TARGET): $(SRC)
	mkdir -p bin
	$(CXX) $(CXXFLAGS) $(SRC) $(INC) -o $(TARGET) $(LDFLAGS)

clean:
	rm -rf bin