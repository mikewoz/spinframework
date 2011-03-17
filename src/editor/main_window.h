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
 * The spin::editor::MainWindow class.
 */

#ifndef __MAIN_WINDOW_H__
#define __MAIN_WINDOW_H__

#include "wx/wx.h" 

namespace spin
{
namespace editor
{

/**
 * The Spin Editor main GUI window.
 */
class MainWindow: public wxFrame
{
    public:
    	/**
    	 * Constructor.
    	 * @param title Title of the window.
    	 * @param pos Initial position of the window.
    	 * @param size Initial size of the window.
    	 */
        MainWindow(const wxString& title, const wxPoint& pos, const wxSize& size);
        void OnQuit(wxCommandEvent& event);
        void OnAbout(wxCommandEvent& event);
        DECLARE_EVENT_TABLE();
};

/**
 * Signals of this application.
 */
enum
{
    ID_Quit = 1,
    ID_About,
};

} // end of namespace editor
} // end of namespace spin

#endif

