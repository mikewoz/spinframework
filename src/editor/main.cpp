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

namespace spin
{
namespace editor
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
};

bool SpinEditorApp::OnInit()
{
    int width = 600;
    int height = 600;
    MainWindow *frame = new MainWindow(_("SPIN Editor"), wxPoint(50, 50), wxSize(width, height));
    frame->Show(true);
    SetTopWindow(frame);
    return true;
} 

} // end of namespace editor
} // end of namespace spin

/**
 * This macros is expanded into the main() of this application.
 */
IMPLEMENT_APP(spin::editor::SpinEditorApp)

