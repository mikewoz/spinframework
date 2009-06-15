#ifndef __MSGUTIL_H
#define __MSGUTIL_H



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


std::vector<std::string> tokenize(const std::string& str, const std::string& delimiters = " ");
std::vector<float> floatsFromString (std::string theString);


// *****************************************************************************
// file helpers

bool fileExists(const std::string& fileName);


#endif