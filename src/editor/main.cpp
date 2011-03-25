/*
 * This file is part of the SPIN Framework.
 *
 * Copyright (c) 2009 Mike Wozniewski
 * Copyright (c) 2009 Zack Settel
 * Copyright (c) 2011 Alexandre Quessy
 *
 * SPIN Framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * SPIN Framework is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with SPIN Framework. If not, see <http://www.gnu.org/licenses/>.
 */

/** \file
 * The main SpinEditorApp class and its implementation, as well as the main function of the application.
 */



#include "config.h"
#include "main.h"
#include "main_window.h"
#include "MainFrame.h"
#include "spinApp.h"

#include "SceneManager.h"

#ifdef __WXMAC__
#include <ApplicationServices/ApplicationServices.h>
#endif // __WXMAC__

#include <iostream>


namespace spineditor {

bool SpinEditorApp::OnInit()
{
    spinApp &spin = spinApp::Instance();

    // call parent init (mandatory)
    if (!wxApp::OnInit())
        return false;

#ifdef __WXMAC__
    // need to give focus to the process (for development; should be fixed when
    // using an .app bundle):
    ProcessSerialNumber PSN;
    GetCurrentProcess(&PSN);
    TransformProcessType(&PSN, kProcessTransformToForegroundApplication);
#endif

    if (! spinListener.start())
    {
        std::cout << "ERROR: could not start SPIN listener" << std::endl;
        return false;
    }

    // initialize image handlers (for toolbar iconse, etc)
    // TODO: use only the image handlers required
    wxInitAllImageHandlers();


    int width = 400;
    int height = 500;
    //MainWindow *frame = new MainWindow(_("SPIN Editor"), wxPoint(50, 50), wxSize(width, height));
    //MainFrame *frame = new MainFrame( NULL, wxID_ANY, _("SPIN Editor"), wxPoint(50, 50), wxSize(width, height), wxDEFAULT_FRAME_STYLE );
    MainFrame *frame = new MainFrame( NULL );

    frame->Show(true);
    SetTopWindow(frame);

    // start a timer to act as a periodic polling function (eg, to check that
    // the spinListener thread is still running
    spinPollTimer_ = new wxTimer(this, SpinPollTimer_ID);
    spinPollTimer_->Start(500); // milliseconds

    // ask for refresh:
    spin.SceneMessage("s", "refresh", LO_ARGS_END);
    return true;
}

int SpinEditorApp::OnExit()
{
    std::cout << "Got SpinEditorApp::OnExit()" << std::endl;
    // ??
    // spinApp::Instance().getContext()->stop();

    return 1;
}

void SpinEditorApp::OnSpinPollTimer(wxTimerEvent& event)
{
    if (!spinListener.isRunning())
    {
        // tell wx to quit
        GetTopWindow()->Close();
    }
}

// -----------------------------------------------------------------------------

void SpinEditorApp::OnInitCmdLine(wxCmdLineParser& parser)
{
    parser.SetDesc(g_cmdLineDesc);
    parser.SetSwitchChars(wxT("-"));
}

bool SpinEditorApp::OnCmdLineError(wxCmdLineParser & parser)
{
    parser.Usage();
    return false;
}

bool SpinEditorApp::OnCmdLineParsed(wxCmdLineParser& parser)
{
    spinApp &spin = spinApp::Instance();

    if (parser.Found(wxT("v")))
    {
        std::cout << "SPIN Framework: version " << VERSION << std::endl;
        return false;
    }

    wxString sceneID;
    if (parser.Found(wxT("s"), &sceneID))
        spin.setSceneID(sceneID.ToStdString());

    wxString userID;
    if (parser.Found(wxT("u"), &userID))
        spin.setUserID(userID.ToStdString());

    wxString serverAddr;
    if (parser.Found(wxT("r"), &serverAddr))
        spinListener.lo_txAddr= lo_address_new_from_url(serverAddr.c_str());

    return true;
}

} // end of namespace spineditor

// -----------------------------------------------------------------------------
/**
 * These are required macros for wx. The event table binds event ids to methods
 * in SpinEditorApp, and the IMPLEMENT_APP macro expands to create a main()
 */

BEGIN_EVENT_TABLE(spineditor::SpinEditorApp, wxApp)
    EVT_TIMER(SpinPollTimer_ID, spineditor::SpinEditorApp::OnSpinPollTimer)
END_EVENT_TABLE()

IMPLEMENT_APP(spineditor::SpinEditorApp)
