#include <arpa/inet.h>
#include <ifaddrs.h>
#include <string>
#include <stdlib.h>

#include "asUtil.h"

using namespace std;


// *****************************************************************************
// networking functions

std::string getMyIPaddress()
{

	struct ifaddrs *interfaceArray = NULL, *tempIfAddr = NULL;
	void *tempAddrPtr = NULL;
	int rc = 0;
	char addressOutputBuffer[INET6_ADDRSTRLEN];

	//char *IPaddress;
	string IPaddress;

	
	rc = getifaddrs(&interfaceArray);  /* retrieve the current interfaces */
	if (rc == 0)
	{    
		for (tempIfAddr = interfaceArray; tempIfAddr != NULL; tempIfAddr = tempIfAddr->ifa_next)
		{
			if (tempIfAddr->ifa_addr->sa_family == AF_INET) // check if it is IP4
			{
				tempAddrPtr = &((struct sockaddr_in *)tempIfAddr->ifa_addr)->sin_addr;
				
				if (string(tempIfAddr->ifa_name).find("lo")==string::npos) // skip loopback
				{
					IPaddress = inet_ntop(tempIfAddr->ifa_addr->sa_family, tempAddrPtr, addressOutputBuffer, sizeof(addressOutputBuffer));
					
					//printf("Internet Address: [%s] %s \n", tempIfAddr->ifa_name, IPaddress.c_str());
				
					// TODO: for now we just return the first address found. Eventually, we could ask for a specific address (eg, "eth0" vs "eth1")
					break;					
				}
			}
		}
	}
	return IPaddress;
}

std::string getMyBroadcastAddress()
{
	string myIP = getMyIPaddress();
	return myIP.substr(0,myIP.rfind(".")) + ".255";
}

bool isMulticastAddress(std::string s)
{
	bool b = false;
	try {
		int i = atoi(s.substr(0,s.find(".")).c_str());
		if ((i>=224) && (i<=239)) b = true;	
	}
	catch (int i)
	{
		b = false;
	}
	return b;
}

bool isBroadcastAddress(std::string s)
{
	bool b = false;
	try {
		if (s.substr(s.rfind(".")+1) == "255") b = true;
	}
	catch (int i)
	{
		b = false;
	}
	return b;
}

// *****************************************************************************
// string handling functions

std::string stringify(float x)
{
	std::ostringstream o;
	if (!(o << x)) return "";
	return o.str();
}

std::string leadingSpaces(int n)
{
	return std::string(n, '\t');
}


vector<string> tokenize(const string& str, const string& delimiters)
{
	vector<string> tokens;
	
	// skip delimiters at beginning:
	string::size_type lastPos = str.find_first_not_of(delimiters, 0);
	// find first "non-delimiter":
	string::size_type pos = str.find_first_of(delimiters, lastPos);

	if (lastPos == string::npos)
	{
		// this is an empty string, so return empty vector:
		return tokens;
	}
	
	else if (pos == string::npos)
	{
		// no delimiter could be found (ie, there is just one token)
		tokens.push_back(str);
		return tokens;
	}

	else {
		while (string::npos != pos || string::npos != lastPos)
		{
			// found a token, add it to the vector:
			tokens.push_back(str.substr(lastPos, pos - lastPos));
			// skip delimiters (Note the "not_of"):
			lastPos = str.find_first_not_of(delimiters, pos);
			// find next "non-delimiter":
			pos = str.find_first_of(delimiters, lastPos);
		}	
		return tokens;
	}
}

vector<float> floatsFromString (string theString)
{
	// This function takes an std::string and uses spaces to
	// tokenize the string into a vector of floats. If the
	// tokens are symbolic instead of numeric, they are ignored.
	
	vector<string> in_Tokens = tokenize(theString);
	vector<float> out_Tokens;
	float num;
  
	for (unsigned int i = 0; i < in_Tokens.size(); i++)
	{
		// only add to vector if token is a number:
		if (fromString<float>(num, in_Tokens[i])) out_Tokens.push_back(num);
		//if (fromString(num, in_Tokens[i])) out_Tokens.push_back(num);
	}

	return out_Tokens;
}

// *****************************************************************************
// file helpers

bool fileExists(const std::string& fileName)
{
	std::fstream fin;
	fin.open(fileName.c_str(),std::ios::in);
	if( fin.is_open() )
	{
		fin.close();
		return true;
	}
	fin.close();
	return false;
}

