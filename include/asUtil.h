#ifndef __ASUTIL_H
#define __ASUTIL_H

#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>

// *****************************************************************************
// networking functions

std::string getMyIPaddress();
std::string getMyBroadcastAddress();
bool isMulticastAddress(std::string s);
bool isBroadcastAddress(std::string s);

// *****************************************************************************
// string handling functions

template <class T> bool fromString(T& t, const std::string& s)
{
	std::istringstream iss(s);
	return !(iss >> t).fail();
}

/*
template <typename T> bool fromString(T &aValue, const std::string &aStr)
{
	std::stringstream ss(aStr);
	return ss >> aValue;
}
*/

std::string stringify(float x);

std::string leadingSpaces(int n);


std::vector<std::string> tokenize(const std::string& str, const std::string& delimiters = " ");
std::vector<float> floatsFromString (std::string theString);

// *****************************************************************************
// file helpers

bool fileExists(const std::string& fileName);



#endif
