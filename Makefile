CXX = g++
CXXFLAGS = -Wall -g
TARGET = bin/sim
SRC = sim.cpp 

all: $(SRC)
	mkdir -p bin
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET)
	cd bin
	./$(TARGET)

clean:
	rm -rf bin/*