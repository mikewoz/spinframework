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
#include "main_window.h"
#include "spinApp.h"
#include "spinClientContext.h"
#include "SceneManager.h"

#ifdef __WXMAC__
#include <ApplicationServices/ApplicationServices.h>
#endif // __WXMAC__

namespace spineditor
{

/**
 * The Spin Editor application.
 */
class SpinEditorApp: public wxApp
{
	/**
	 * Called when this application is launched.
	 * Creates the MainWindow instance.
	 */
    virtual bool OnInit();

private:
    spinClientContext spinListener;
};

bool SpinEditorApp::OnInit()
{
#ifdef __WXMAC__
    // need to give focus to the process (for development; should be fixed when
    // using an .app bundle):
    ProcessSerialNumber PSN;
    GetCurrentProcess(&PSN);
    TransformProcessType(&PSN,kProcessTransformToForegroundApplication);
#endif

    spinApp &spin = spinApp::Instance();

    // TODO: parse commandline args and allow overrides for server host/port,
    // user id, etc.

    if (!spinListener.start())
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

} // end of namespace spineditor

/**
 * This macros is expanded into the main() of this application.
 */
IMPLEMENT_APP(spineditor::SpinEditorApp)

