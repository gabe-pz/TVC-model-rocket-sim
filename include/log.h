#pragma once

#include <iostream>
#include <fstream>
#include <string>

void initCSV(const std::string& filename, int axis);
void logToCSV(double t, double x, const std::string& filename);