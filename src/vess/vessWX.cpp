/***************************************************************
 * Name:      vessWX.cpp
 * Purpose:   Code for Application Class
 * Author:    Mike Wozniewski (mike@mikewoz.com)
 * Created:   2009-03-11
 * Copyright: Mike Wozniewski (www.mikewoz.com)
 * License:
 **************************************************************/

#include "vessWX.h"

#include <iostream>

#include <osgDB/ReadFile>
#include <wx/log.h>
#include <wx/ffile.h>
#include <wx/string.h>

#include "vessThreads.h"

//(*AppHeaders
#include "wxVessMain.h"
#include <wx/image.h>
//*)

IMPLEMENT_APP(vessWX);


vessMaster *vess; // global

bool vessWX::OnInit()
{


    // Make sure we can load the libAudioscape library:
	osgDB::Registry *reg = osgDB::Registry::instance();
	osgDB::DynamicLibrary::loadLibrary(reg->createLibraryNameForNodeKit("libAudioscape"));


	vess = new vessMaster();

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
