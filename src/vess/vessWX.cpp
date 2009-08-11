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

#include "vessWX.h"

#include <iostream>

#include <osgDB/ReadFile>
#include <wx/log.h>
#include <wx/ffile.h>
#include <wx/string.h>
#include <wx/stdpaths.h>

#include "vessThreads.h"

//(*AppHeaders
#include "wxVessMain.h"
#include <wx/image.h>
//*)

IMPLEMENT_APP(vessWX);


vessThread *vess; // global
wxString resourcesPath; // global

bool vessWX::OnInit()
{
    // call default behaviour (mandatory)
    if (!wxApp::OnInit())
        return false;

	vess = new vessThread();

	// override infoPort based on environment variable:
	/*
	std::cout << "WX GetConfigDir: " << wxStandardPaths::Get().GetConfigDir().mb_str() << std::endl;
	std::cout << "WX GetDataDir: " << wxStandardPaths::Get().GetDataDir().mb_str() << std::endl;
	std::cout << "WX GetDocumentsDir: " << wxStandardPaths::Get().GetDocumentsDir().mb_str() << std::endl;
	std::cout << "WX GetExecutablePath: " << wxStandardPaths::Get().GetExecutablePath().mb_str() << std::endl;
	//std::cout << "WX GetInstallPrefix: " << wxStandardPaths::Get().GetInstallPrefix().mb_str() << std::endl;
	std::cout << "WX GetLocalDataDir: " << wxStandardPaths::Get().GetLocalDataDir().mb_str() << std::endl;
	std::cout << "WX GetPluginsDir: " << wxStandardPaths::Get().GetPluginsDir().mb_str() << std::endl;
	std::cout << "WX GetResourcesDir: " << wxStandardPaths::Get().GetResourcesDir().mb_str() << std::endl;
	*/
#ifdef _SPINDEBUG
	resourcesPath = wxStandardPaths::Get().GetExecutablePath();
	resourcesPath = resourcesPath.SubString(0,resourcesPath.Find('/',true)); // remove executable name
	resourcesPath = resourcesPath + _T("../images");
#else
    resourcesPath = wxStandardPaths::Get().GetResourcesDir();
#endif


    //(*AppInitialize
    bool wxsOK = true;
    wxInitAllImageHandlers();
    if ( wxsOK )
    {
    	wxVessMain* Frame = new wxVessMain(0);
    	Frame->Show();
    	SetTopWindow(Frame);
    }
    //*)

    return wxsOK;

}

void vessWX::OnInitCmdLine(wxCmdLineParser& parser)
{
    parser.SetDesc (g_cmdLineDesc);
    parser.SetSwitchChars (wxT("-"));
}

bool vessWX::OnCmdLineParsed(wxCmdLineParser& parser)
{
    if (parser.Found(wxT("c"))) startMode = 1;
    else if (parser.Found(wxT("s"))) startMode = 2;
	else startMode = 0;

    // to accept scene files as arguments:
    wxArrayString files;
    for (int i = 0; i < parser.GetParamCount(); i++)
    {
		files.Add(parser.GetParam(i));
    }

    return true;
}
