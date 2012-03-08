// -----------------------------------------------------------------------------
// |    ___  ___  _  _ _     ___                                        _      |
// |   / __>| . \| || \ |   | __>_ _  ___ ._ _ _  ___  _ _ _  ___  _ _ | |__   |
// |   \__ \|  _/| ||   |   | _>| '_><_> || ' ' |/ ._>| | | |/ . \| '_>| / /   |
// |   <___/|_|  |_||_\_|   |_| |_|  <___||_|_|_|\___.|__/_/ \___/|_|  |_\_\   |
// |                                                                           |
// |---------------------------------------------------------------------------|
//
// http://spinframework.sourceforge.net
// Copyright (C) 2009 Mike Wozniewski, Zack Settel
//
// Developed/Maintained by:
//    Mike Wozniewski (http://www.mikewoz.com)
//    Zack Settel (http://www.sheefa.net/zack)
// 
// Principle Partners:
//    Shared Reality Lab, McGill University (http://www.cim.mcgill.ca/sre)
//    La Societe des Arts Technologiques (http://www.sat.qc.ca)
//
// Funding by:
//    NSERC/Canada Council for the Arts - New Media Initiative
//    Heritage Canada
//    Ministere du Developpement economique, de l'Innovation et de l'Exportation
//
// -----------------------------------------------------------------------------
//  This file is part of the SPIN Framework.
//
//  SPIN Framework is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  SPIN Framework is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  along with SPIN Framework. If not, see <http://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/utsname.h>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <pwd.h>
#include <unistd.h>

#include <osgDB/ReadFile>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include "spinUtil.h"


namespace spin
{

// *****************************************************************************
// networking functions

std::string getHostname()
{
	struct utsname ugnm;
    std::string hostname;

	if (uname(&ugnm) < 0) return "";
	hostname = std::string(ugnm.nodename);
	
	// for OSX, remove .local
	size_t pos = hostname.rfind(".local");
	if (pos!=std::string::npos) hostname = hostname.substr(0,pos);
	
	return hostname;
}

std::string getMyIPaddress()
{
	struct ifaddrs *interfaceArray = NULL, *tempIfAddr = NULL;
	void *tempAddrPtr = NULL;
	int rc = 0;
	char addressOutputBuffer[INET6_ADDRSTRLEN];

	//char *IPaddress;
    std::string IPaddress;

	rc = getifaddrs(&interfaceArray);  /* retrieve the current interfaces */
	if (rc == 0)
	{    
		for (tempIfAddr = interfaceArray; tempIfAddr != NULL; tempIfAddr = tempIfAddr->ifa_next)
		{
			if (tempIfAddr->ifa_addr && (tempIfAddr->ifa_addr->sa_family == AF_INET)) // check if it is IP4
			{
				tempAddrPtr = &((struct sockaddr_in *)tempIfAddr->ifa_addr)->sin_addr;
				
				if (std::string(tempIfAddr->ifa_name).find("lo")==std::string::npos) // skip loopback
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
    std::string myIP = getMyIPaddress();
	return myIP.substr(0,myIP.rfind(".")) + ".255";
}

bool isMulticastAddress(const std::string &s)
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

bool isBroadcastAddress(const std::string &s)
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

std::string stringify(int x)
{
    std::ostringstream o;
    if (!(o << x)) return "";
    return o.str();
}
    
std::string stringify(osg::Quat q)
{
    std::ostringstream o;
    o << q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w();
    if (!o) return "0 0 0 0";
    return o.str();
}

std::string stringify(osg::Vec3f v)
{
    std::ostringstream o;
    o << v.x()<<" "<<v.y()<<" "<<v.z();
    if (!o) return "0 0 0";
    return o.str();
}

std::string stringify(osg::Vec3d v)
{
    osg::Vec3f vf(v);
    return stringify(vf);
}

std::string leadingSpaces(int n)
{
	//return std::string(n, '\t');
	return std::string(n, ' ');
}

std::vector<std::string> tokenize(const std::string& str, const std::string& delimiters)
{
    using std::vector;
    using std::string;
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

std::vector<float> floatsFromString (const std::string &theString)
{
    using std::string;
    using std::vector;
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
	
bool wildcardMatch(const char *pat, const char *str)
{
	switch (*pat)
	{
		case '\0':
			return *str=='\0';
		case '*':
			return wildcardMatch(pat+1, str) || (*str && wildcardMatch(pat, str+1));
		case '?':
			return *str && wildcardMatch(pat+1, str+1);
		default:
			return *pat==*str && wildcardMatch(pat+1, str+1);
	}
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

std::string getRelativePath(const std::string &path)
{
	using std::string;
	
	string relPath;
	
	if (path.substr(0,7) == string("/Users/"))
	{
		relPath = path.substr(8);
	}
	else if (path.substr(0,6) == string("/home/"))
	{
		relPath = path.substr(7);
	}
	else {
		return path;
	}
		
	size_t pos = relPath.find("/");
	relPath = "~/" + relPath.substr(pos+1);

	return relPath;
}

std::string getAbsolutePath(const std::string &path)
{
	using std::string;
    
	// TODO: also deal with: ./ ../
	
	if (path.substr(0,1) == string("~")) // look for "~"
	{
        std::string homePath;
        
        struct passwd* pwd = getpwuid(getuid());
        if (pwd)
        {
            homePath = pwd->pw_dir;
        }
        else
        {
            // try the $HOME environment variable
            homePath = getenv("HOME");
        }
        
        // fall back to current directory:
        if (homePath.empty()) homePath = "./";
        
		return homePath + path.substr(1);    
        
	} else return path;
}

bool isVideoPath(const std::string &path)
{
	using std::string;
	
	string extension = osgDB::getLowerCaseFileExtension(path);
	
	if ((osgDB::getDirectoryContents(getAbsolutePath(path)).size()))
	{
		return true;
	}
	else if ((extension=="mp4") ||
			 (extension=="avi") ||
			 (extension=="mpg") ||
			 (extension=="mpeg") ||
			 (extension=="mov") ||
			 (extension=="wmv") ||
			 (extension=="qt") ||
			 (extension=="ogm") ||
			 (extension=="m4v") ||
			 (extension=="dv") ||
			 (extension=="3gp") ||
			 (extension=="vob") ||
			 (extension=="tgas") ||
			 (extension=="divx") ||
			 (extension=="flv"))
	{
		return true;
	}
	
	return false;
}

bool isImagePath(const std::string &path)
{
    using std::string;
    string extension = osgDB::getLowerCaseFileExtension(path);
    if  ((extension=="jpg") ||
         (extension=="jpeg") ||
         (extension=="gif") ||
         (extension=="png") ||
         (extension=="tif") ||
         (extension=="tiff") ||
         (extension=="bmp") ||
         (extension=="rgb") ||
         (extension=="tga") ||
         (extension=="pic") ||
         (extension=="dds") ||
         (extension=="sgi"))
    {
        return true;
    }
    
    return false;
}
    

/**
 * This checks the file/path name to see there is encoded path information, and
 * if it doesn't we assume the user wants to put it in the SPIN_DIRECTORY
 */
std::string getSpinPath(const std::string &path)
{
	using std::string;

	string filename(path);

	// check if the filename has specific path information:
	if ( (filename.substr(0,1)==string("~")) || 
		 (filename.substr(0,1)==string("/")) || 
		 (filename.substr(0,2)==string("./")) || 
		 (filename.substr(0,3)==string("../")) )
	{
		filename = getAbsolutePath(path);
	}
	
	// if not, then put it in the local SPIN_DIRECTORY:
	else
	{
		filename = SPIN_DIRECTORY + "/" + string(path);
	}

	return filename;
}
    
std::vector<char*> getUserArgs() 
{
    std::vector<char*> args;
    
    std::string path = SPIN_DIRECTORY + "/args";
    if (fileExists(path))
    {
        std::stringstream ss;
        ss << std::ifstream( path.c_str() ).rdbuf();
        
        std::string token;
        while (ss >> token)
        {
            char *arg = new char[token.size() + 1];
            copy(token.begin(), token.end(), arg);
            arg[token.size()] = '\0';
            args.push_back(arg);
        }
    }
    
    args.push_back(0); // needs to end with a null item
    return args;
}
    
// *****************************************************************************
// gensym stuff

#define HASHSIZE 1024
static t_symbol *symhash[HASHSIZE];

void *getbytes(size_t nbytes)
{
    void *ret;
    if (nbytes < 1) nbytes = 1;
    ret = (void *)calloc(nbytes, 1);
    //if (!ret) post("pd: getbytes() failed -- out of memory");
    return (ret);
}

void *copybytes(void *src, size_t nbytes)
{
    void *ret;
    ret = getbytes(nbytes);
    if (nbytes)
        memcpy(ret, src, nbytes);
    return (ret);
}

void freebytes(void *fatso, size_t nbytes)
{
    if (nbytes == 0)
        nbytes = 1;
    free(fatso);
}

t_symbol *dogensym(const char *s, t_symbol *oldsym)
{
    t_symbol **sym1, *sym2;
    unsigned int hash1 = 0,  hash2 = 0;
    int length = 0;
    const char *s2 = s;
    while (*s2)
    {
        hash1 += *s2;
        hash2 += hash1;
        length++;
        s2++;
    }
    sym1 = symhash + (hash2 & (HASHSIZE-1));
    while ((sym2 = *sym1))
    {
        if (!strcmp(sym2->s_name, s)) return(sym2);
        sym1 = &sym2->s_next;
    }
    if (oldsym) sym2 = oldsym;
    else
    {
        sym2 = (t_symbol *)getbytes(sizeof(*sym2));
        sym2->s_name = (char*) getbytes(length+1);
        sym2->s_next = 0;
        sym2->s_thing = 0;
        strcpy(sym2->s_name, s);
    }
    *sym1 = sym2;
    return (sym2);
}

t_symbol *gensym(const char *s)
{
    return(dogensym(s, 0));
}

} // end of namespace spin

