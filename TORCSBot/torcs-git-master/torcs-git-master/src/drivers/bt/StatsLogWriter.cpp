#include "StatsLogWriter.h"


bool StatsLogWriter::writeToLog(char* logName, std::string line) {
	std::string logPath = std::string("./searchStatistics/") + std::string(logName) + std::string(".txt");
	std::ofstream logFile = std::ofstream(logPath, std::ios::in | std::ios::out | std::ios::ate); //write at the end of file (doesnt delete content)

	if (!logFile.is_open())
	{
		logFile.open(logPath);
	}
	if (logFile.is_open())
	{
		logFile << (line + std::string("\n")).c_str();
		return true;
	}
		return false;
}
bool StatsLogWriter::closeLog(char* logName){
	std::string logPath = std::string("./searchStatistics/") + std::string(logName) + std::string(".txt");
	std::ofstream logFile = std::ofstream(logPath);
	if (logFile.is_open()){
		logFile.close();
		return true;
	}
	return false;
}