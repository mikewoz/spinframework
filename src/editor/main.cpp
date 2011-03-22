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

#include <wx/wx.h>
#include <wx/cmdline.h>

#include "config.h"
#include "main_window.h"
#include "spinApp.h"
#include "spinClientContext.h"
#include "SceneManager.h"

#ifdef __WXMAC__
#include <ApplicationServices/ApplicationServices.h>
#endif // __WXMAC__

#include <iostream>

namespace spineditor
{

/**
 * The Spin Editor application.
 */
class SpinEditorApp: public wxApp
{
    public:
        /**
         * Constructor. We make it explicit.
         */
        SpinEditorApp() :
            spinListener()
        {
        }
        /**
         * Called when this application is launched.
         *
         * Creates the MainWindow instance.
         * We also make sure there is a SPIN context going on.
         * We ask for a refresh.
         */
        virtual bool OnInit();
        virtual int OnExit();

        /**
         * Initializes commandline arguments
         */
        virtual void OnInitCmdLine(wxCmdLineParser& parser);

        /**
         * Does the actual parsing of commandline arguments
         */
        virtual bool OnCmdLineParsed(wxCmdLineParser& parser);

        /**
         * If the user provides bad arguments, show the usage and exit
         */
        virtual bool OnCmdLineError(wxCmdLineParser & parser);

    private:
        spinClientContext spinListener;
};

static const wxCmdLineEntryDesc g_cmdLineDesc[] =
{
     { wxCMD_LINE_SWITCH,
             "h",
             "help",
             "displays command-line help",
             wxCMD_LINE_VAL_NONE, wxCMD_LINE_OPTION_HELP
     },
     { wxCMD_LINE_OPTION,
             "s",
             "scene-id",
             "Specify the scene ID to listen to",
             wxCMD_LINE_VAL_STRING, wxCMD_LINE_PARAM_OPTIONAL | wxCMD_LINE_NEEDS_SEPARATOR
     },
     { wxCMD_LINE_OPTION,
             "r",
             "server-addr",
             "Specify the remote server address (Default is osc.udp://239.0.0.1:54323)",
             wxCMD_LINE_VAL_STRING, wxCMD_LINE_PARAM_OPTIONAL | wxCMD_LINE_NEEDS_SEPARATOR
     },
     { wxCMD_LINE_OPTION,
             "u",
             "user-id",
             "Specify a user ID for this editor (Default is the local hostname)",
             wxCMD_LINE_VAL_STRING, wxCMD_LINE_PARAM_OPTIONAL | wxCMD_LINE_NEEDS_SEPARATOR
     },
     { wxCMD_LINE_SWITCH,
             "v",
             "version",
             "Display the version number and exit",
             wxCMD_LINE_VAL_NONE, wxCMD_LINE_PARAM_OPTIONAL
     },
     { wxCMD_LINE_NONE }
};


void SpinEditorApp::OnInitCmdLine(wxCmdLineParser& parser)
{

    parser.SetDesc(g_cmdLineDesc);
    // must refuse '/' as parameter starter or cannot use "/path" style paths
    parser.SetSwitchChars (wxT("-"));
}

bool SpinEditorApp::OnCmdLineError(wxCmdLineParser & parser)
{
    parser.Usage();
    return false;
}

bool SpinEditorApp::OnCmdLineParsed(wxCmdLineParser& parser)
{
    spinApp &spin = spinApp::Instance();

    if ( parser.Found(wxT("v")) )
    {
        std::cout << "SPIN Framework: version " << VERSION << std::endl;
        return false;
    }

    wxString sceneID;
    if (parser.Found(wxT("s"), &sceneID))
    {
        spin.setSceneID(sceneID.ToStdString());
    }

    wxString userID;
    if (parser.Found(wxT("u"), &userID))
    {
        spin.setUserID(userID.ToStdString());
    }

    wxString serverAddr;
    if (parser.Found(wxT("r"), &serverAddr))
    {
        spinListener.lo_txAddr= lo_address_new_from_url(serverAddr.c_str());
    }

    return true;
}

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


    // TODO: parse commandline args and allow overrides for server host/port,
    // user id, etc.
    if (! spinListener.start())
    {
        std::cout << "ERROR: could not start SPIN listener" << std::endl;
        return false;
    }

    int width = 600;
    int height = 600;
    MainWindow *frame = new MainWindow(_("SPIN Editor"), wxPoint(50, 50), wxSize(width, height));

    frame->Show(true);
    SetTopWindow(frame);

    // ask for refresh:
    spin.SceneMessage("s", "refresh", LO_ARGS_END);
    return true;
} 

int SpinEditorApp::OnExit()
{
    std::cout << "!!! Got SpinEditorApp::OnExit()" << std::endl;
    // ??
    // spinApp::Instance().getContext()->stop();
}

} // end of namespace spineditor

/**
 * This macros is expanded into the main() of this application.
 */
IMPLEMENT_APP(spineditor::SpinEditorApp)

