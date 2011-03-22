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

#include <wx/wx.h>

namespace spineditor
{

/**
 * Logging levels
 */
enum LogLevel
{
    LOG_DEBUG,
    LOG_INFO,
    LOG_WARNING,
    LOG_ERROR,
    LOG_CRITICAL
};

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
        void OnHelp(wxCommandEvent& event);
        static void log(LogLevel level, const std::string &text);
        DECLARE_EVENT_TABLE();
    private:
        //wxTreeCtrl *treeControl_;
        wxString resourcesPath;
};

/**
 * Signals of this application.
 * Each has a unique number greater than zero.
 */
enum
{
    SIGNAL_MENU_QUIT = 1,
    SIGNAL_MENU_ABOUT,
    SIGNAL_MENU_HELP,
    SIGNAL_BUTTON_HELLO_0,
    SIGNAL_BUTTON_HELLO_1,
};

wxString stringToWxString(const std::string &text);

// const long SIGNAL_MENU_HELP;
// const long SIGNAL_MENU_ABOUT;
// const long SIGNAL_MENU_QUIT; // = wxNewId();

} // end of namespace spineditor

#endif

