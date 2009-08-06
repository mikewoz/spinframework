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
//  You should have received a copy of the Lesser GNU General Public License
//  along with SPIN Framework. If not, see <http://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------

#ifndef __ASUTIL_H
#define __ASUTIL_H

#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>

// *****************************************************************************
// networking functions

std::string getHostname();
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
std::string getRelativePath(std::string path);
std::string getAbsolutePath(std::string path);

#endif
