#include "log.h"

void initCSV(const std::string& filename, int axis) {
    std::ofstream file(filename); 
    if(axis == 0){
        if (file.is_open()) {
            file << "t,rotX\n"; //Write header
        }
    }
    else{
        if (file.is_open()) {
            file << "t,rotY\n"; //Write header
        }
    }
}

void logToCSV(double t, double x, const std::string& filename) {
    std::ofstream file(filename, std::ios::app); //Append mode
    if (file.is_open()) {
        file << t << "," << x << "\n";
    }
}