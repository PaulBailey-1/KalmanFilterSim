#include "Logger.h"

std::ofstream Logger::m_file;
int Logger::m_line;
std::string Logger::m_labels;
std::string Logger::m_values;

Logger::Logger() {
	m_line = 0;
}

void Logger::open(std::string name) {
	m_file.open(name);
}

void Logger::close() {
	m_file.close();
}

void Logger::log(std::string name, double value) {
	if (m_line == 0) {
		m_labels += name + ", ";
	}
	m_values += std::to_string(value) + ", ";
}

void Logger::nextLine() {
	if (m_line == 0) {
		m_file << m_labels << '\n';
	}
	m_file << m_values << '\n';
	m_values = "";
	m_line++;
}