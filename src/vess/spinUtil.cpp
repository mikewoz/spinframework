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

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <string>
#include <stdlib.h>
#include <sys/utsname.h>


#include <osgIntrospection/Reflection>
#include <osgIntrospection/Value>
#include <osgIntrospection/variant_cast>
#include <osgIntrospection/Exceptions>
#include <osgIntrospection/PropertyInfo>


#include "spinUtil.h"



// *****************************************************************************
// networking functions

std::string getHostname()
{
	struct utsname ugnm;

	 if (uname(&ugnm) < 0) return "";

	 return std::string(ugnm.nodename);
	
}

std::string getMyIPaddress()
{
	using namespace std;
	
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
	using namespace std;
	
	string myIP = getMyIPaddress();
	return myIP.substr(0,myIP.rfind(".")) + ".255";
}

bool isMulticastAddress(std::string s)
{
	using namespace std;
	
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


std::vector<std::string> tokenize(const std::string& str, const std::string& delimiters)
{
	using namespace std;
	
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

std::vector<float> floatsFromString (std::string theString)
{
	using namespace std;
	
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
	using namespace std;
	
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

std::string getRelativePath(std::string path)
{
	using namespace std;
	
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

std::string getAbsolutePath(std::string path)
{
	using namespace std;
	
	if (path.substr(0,1) == string("~")) // look for "~"
	{
		return getenv("HOME") + path.substr(1);
	} else return path;
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
    while (sym2 = *sym1)
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





// *****************************************************************************
// introspection helpers

bool introspect_type_order(const osgIntrospection::Type *v1, const osgIntrospection::Type *v2)
{
	using namespace osgIntrospection;
	
    if (!v1->isDefined()) return v2->isDefined();
    if (!v2->isDefined()) return false;
    return v1->getQualifiedName().compare(v2->getQualifiedName()) < 0;
}

void introspect_print_method(const osgIntrospection::MethodInfo &mi)
{
	using namespace osgIntrospection;
	
    std::cout << "\t    ";

    // display if the method is virtual
    if (mi.isVirtual())
        std::cout << "virtual ";

    // display the method's return type if defined
    if (mi.getReturnType().isDefined())
        std::cout << mi.getReturnType().getQualifiedName() << " ";
    else
        std::cout << "[UNDEFINED TYPE] ";

    // display the method's name
    std::cout << mi.getName() << "(";

    // display method's parameters
    const ParameterInfoList &params = mi.getParameters();
    for (ParameterInfoList::const_iterator k=params.begin(); k!=params.end(); ++k)
    {
        // get the ParameterInfo object that describes the 
        // current parameter
        const ParameterInfo &pi = **k;

        // display the parameter's modifier
        if (pi.isIn())
            std::cout << "IN";
        if (pi.isOut())
            std::cout << "OUT";
        if (pi.isIn() || pi.isOut())
            std::cout << " ";

        // display the parameter's type name
        if (pi.getParameterType().isDefined())
            std::cout << pi.getParameterType().getQualifiedName();

        // display the parameter's name if defined
        if (!pi.getName().empty())
            std::cout << " " << pi.getName();

        if ((k+1)!=params.end())
            std::cout << ", ";
    }
    std::cout << ")";
    if (mi.isConst())
        std::cout << " const";
    if (mi.isPureVirtual())
        std::cout << " = 0";
    std::cout << "\n";
}

void introspect_print_type(const osgIntrospection::Type &type)
{
	using namespace osgIntrospection;
	
    // ignore pointer types and undefined types
    if (!type.isDefined() || type.isPointer() || type.isReference())
    {
    	std::cout << "Error: Type is pointer or undefined." << std::endl;
    	return;
    }
    
    // print the type name
    std::cout << "  " << type.getQualifiedName() << "\n";

    // check whether the type is abstract
    if (type.isAbstract()) std::cout << "\t[abstract]\n";

    // check whether the type is atomic
    if (type.isAtomic()) std::cout << "\t[atomic]\n";

    // check whether the type is an enumeration. If yes, display
    // the list of enumeration labels
    if (type.isEnum()) 
    {
        std::cout << "\t[enum]\n";
        std::cout << "\tenumeration values:\n";
        const EnumLabelMap &emap = type.getEnumLabels();
        for (EnumLabelMap::const_iterator j=emap.begin(); j!=emap.end(); ++j)
        {
            std::cout << "\t\t" << j->second << " = " << j->first << "\n";
        }
    }

    // if the type has one or more base types, then display their
    // names
    if (type.getNumBaseTypes() > 0)
    {
        std::cout << "\tderived from: ";
        for (int j=0; j<type.getNumBaseTypes(); ++j)
        {
            const Type &base = type.getBaseType(j);
            if (base.isDefined())
                std::cout << base.getQualifiedName() << "    ";
            else
                std::cout << "[undefined type]    ";
        }
        std::cout << "\n";
    }

    // display a list of public methods defined for the current type
    const MethodInfoList &mil = type.getMethods();
    if (!mil.empty())
    {
        std::cout << "\t* public methods:\n";
        for (MethodInfoList::const_iterator j=mil.begin(); j!=mil.end(); ++j)
        {
            // get the MethodInfo object that describes the current
            // method
            const MethodInfo &mi = **j;

            introspect_print_method(mi);
        }
    }

    // display a list of protected methods defined for the current type
    const MethodInfoList &bmil = type.getMethods(Type::PROTECTED_FUNCTIONS);
    if (!bmil.empty())
    {
        std::cout << "\t* protected methods:\n";
        for (MethodInfoList::const_iterator j=bmil.begin(); j!=bmil.end(); ++j)
        {
            // get the MethodInfo object that describes the current
            // method
            const MethodInfo &mi = **j;

            introspect_print_method(mi);
        }
    }

    // display a list of properties defined for the current type
    const PropertyInfoList &pil = type.getProperties();
    if (!pil.empty())
    {
        std::cout << "\t* properties:\n";
        for (PropertyInfoList::const_iterator j=pil.begin(); j!=pil.end(); ++j)
        {
            // get the PropertyInfo object that describes the current
            // property
            const PropertyInfo &pi = **j;

            std::cout << "\t    ";

            std::cout << "{";
            std::cout << (pi.canGet()? "G": " ");
            std::cout << (pi.canSet()? "S": " ");
            std::cout << (pi.canCount()? "C": " ");
            std::cout << (pi.canAdd()? "A": " ");
            std::cout << "}  ";

            // display the property's name
            std::cout << pi.getName();

            // display the property's value type if defined
            std::cout << " (";
            if (pi.getPropertyType().isDefined())
                std::cout << pi.getPropertyType().getQualifiedName();
            else
                std::cout << "UNDEFINED TYPE";
            std::cout << ") ";

            // check whether the property is an array property
            if (pi.isArray())
            {
                std::cout << "  [ARRAY]";
            }

            // check whether the property is an indexed property
            if (pi.isIndexed())
            {
                std::cout << "  [INDEXED]\n\t\t       indices:\n";

                const ParameterInfoList &ind = pi.getIndexParameters();

                // print the list of indices
                int num = 1;
                for (ParameterInfoList::const_iterator k=ind.begin(); k!=ind.end(); ++k, ++num)
                {
                    std::cout << "\t\t           " << num << ") ";
                    const ParameterInfo &par = **k;
                    std::cout << par.getParameterType().getQualifiedName() << " " << par.getName();
                    std::cout << "\n";
                }
            }

            std::cout << "\n";
        }
    }
    std::cout << "\n" << std::string(75, '-') << "\n";
}

void introspect_print_all_types()
{
	using namespace osgIntrospection;
	
    // get the map of types that have been reflected
    const TypeMap &tm = Reflection::getTypes();
    
    // create a sortable list of types
    TypeList types(tm.size());
    TypeList::iterator j = types.begin();
    for (TypeMap::const_iterator i=tm.begin(); i!=tm.end(); ++i, ++j)
        *j = i->second;
    
    // sort the map
    std::sort(types.begin(), types.end(), &introspect_type_order);

    // iterate through the type map and display some
    // details for each type
    for (TypeList::const_iterator i=types.begin(); i!=types.end(); ++i)
    {
        introspect_print_type(**i);
    }
}
