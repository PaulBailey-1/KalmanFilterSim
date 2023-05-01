#pragma once
#include <string>
#include <iostream>
#include <fstream>

class Logger {
public:

	Logger();

	void open(std::string name);
	void close();

	static void log(std::string, double value);
	void nextLine();

private:

	static std::ofstream m_file;

	static int m_line;

	static std::string m_labels;
	static std::string m_values;

};