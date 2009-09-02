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

#include <string>
#include <stdio.h>
#include <stdlib.h>
#include "MediaManager.h"
#include "spinUtil.h"

using namespace std;


// ***********************************************************
// constructor
MediaManager::MediaManager (string p)
{
	this->dataPath = p;
}

// ***********************************************************
// destructor
MediaManager::~MediaManager()
{
}

// ***********************************************************
string MediaManager::getImagePath(string s)
{
	/*
	if (s=="NULL") return "";
	else return this->projectPath + "/sounds/" + s;
	*/
	
	if (s=="NULL") return "";
	else if (s.substr(0,1) == string("~")) // look for "~"
	{
		return getenv("HOME") + s.substr(1);
	} else return s;
}

string MediaManager::getModelPath(string s)
{
	/*
	if (s=="NULL") return "";
	
	else
	{
		string f = this->projectPath + "/models/" + s + "/" + s + ".ive";
		if (!fileExists(f)) f = this->projectPath + "/models/" + s + "/" + s + ".osg";
	
		return f;
	}
	*/
	if (s=="NULL") return "";
	else if (s.substr(0,1) == string("~")) // look for "~"
	{
		return getenv("HOME") + s.substr(1);
	} else return s;

}
string MediaManager::getSoundPath(string s)
{
	/*
	if (s=="NULL") return "";
	else return this->projectPath + "/sounds/" + s;
	*/
	
	if (s=="NULL") return "";
	else if (s.substr(0,1) == string("~")) // look for "~"
	{
		return getenv("HOME") + s.substr(1);
	} else return s;
}

// TODO: connect to database and get media given the id:
string MediaManager::getImagePath(int id) { return ""; }
string MediaManager::getModelPath(int id) { return ""; }
string MediaManager::getSoundPath(int id) { return ""; }

string MediaManager::getImageName(int id) { return "NULL"; }
string MediaManager::getModelName(int id) { return "NULL"; }
string MediaManager::getSoundName(int id) { return "NULL"; }

