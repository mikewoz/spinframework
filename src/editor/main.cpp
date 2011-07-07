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
    spin::spinApp &spin = spin::spinApp::Instance();

    // call parent init (mandatory)
    if (!wxApp::OnInit())
        return false;

    // needs to be called before the MainFrame is constructed (because
    // wxSpinTreeCtrl needs to set OSC callbacks)
    this->start();

#ifdef __WXMAC__
    // need to give focus to the process (for development; should be fixed when
    // using an .app bundle):
    ProcessSerialNumber PSN;
    GetCurrentProcess(&PSN);
    TransformProcessType(&PSN, kProcessTransformToForegroundApplication);
#endif

    // initialize image handlers (for toolbar iconse, etc)
    // TODO: use only the image handlers required
    wxInitAllImageHandlers();



    // create the main window:
    MainFrame *frame = new MainFrame( NULL );

    // tell the app to exit automatically when the frame closes:
    wxApp::SetExitOnFrameDelete(true);

    frame->Show(true);
    SetTopWindow(frame);

    // start a timer to act as a periodic polling function (eg, to check that
    // the spinListener thread is still running
    spinPollTimer_ = new wxTimer(this, SpinPollTimer_ID);
    spinPollTimer_->Start(500); // milliseconds

    return true;
}

int SpinEditorApp::OnExit()
{
    std::cout << "Got SpinEditorApp::OnExit()" << std::endl;

    // tell spin to stop
    spin::spinApp::Instance().getContext()->stop();
    return 1;
}

void SpinEditorApp::start()
{
    if (!spinListener.start())
    {
        std::cout << "ERROR: could not start SPIN listener" << std::endl;
        wxSafeShowMessage("ERROR", "Could not start SPIN listener. Quitting.");
        return;
    }


}

void SpinEditorApp::OnSpinPollTimer(wxTimerEvent& WXUNUSED(event))
{
    // This checks that the spin client thread is still running. If not, it
    // probably means that the user did a CTRL-C in the terminal window, or
    // something more ugly that I can't think of...

    if (!spinListener.isRunning())
    {
        std::cout << "SPIN stopped running (CTRL-C ?)" << std::endl;

        // Tell this timer that it can stop:
        spinPollTimer_->Stop();
        //delete spinPollTimer_;

        // Tell main frame to quit (with force=TRUE so that it doesn't ask for
        // any confirmation). Once that window closes, the wxapp will be told
        // to quit automatically.
        GetTopWindow()->Close(true);

        std::cout << "top window should be closed" << std::endl;

        if (GetTopWindow())
        {
            std::cout << "got topwindow" << GetTopWindow() << " why does it still exist?!" << std::endl;
            GetTopWindow()->Destroy();
            SetTopWindow(0);
        }

        //this->ExitMainLoop();
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
    spin::spinApp &spin = spin::spinApp::Instance();

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
    {
        spinListener.lo_txAddrs_.clear();
        spinListener.lo_txAddrs_.push_back(lo_address_new_from_url(serverAddr.c_str()));
    }

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
