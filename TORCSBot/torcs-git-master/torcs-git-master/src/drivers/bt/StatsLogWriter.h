// writing on a text file
#include <iostream>
#include <fstream>

class StatsLogWriter {
public:
	static bool writeToLog(char* logName,std::string line);
	static bool closeLog(char* logName);
};