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

#include <wx/wx.h>
#include "main_window.h"
#include "config.h"
#include "introspection.h"
#include <sstream>

namespace spin
{
namespace editor
{

/**
 * Declare signals events
 */
BEGIN_EVENT_TABLE(MainWindow, wxFrame)
    EVT_MENU(SIGNAL_MENU_QUIT, MainWindow::OnQuit)
    EVT_MENU(SIGNAL_MENU_ABOUT, MainWindow::OnAbout)
    EVT_MENU(SIGNAL_MENU_HELP, MainWindow::OnHelp)
END_EVENT_TABLE()

MainWindow::MainWindow(const wxString& title, const wxPoint& pos, const wxSize& size)
: 
    wxFrame(NULL, -1, title, pos, size)
{
    // Create menu bar:
    wxMenuBar *menu_bar = new wxMenuBar;
    // Create file menu:
    wxMenu *file_menu = new wxMenu;
    file_menu->Append(SIGNAL_MENU_QUIT, _("&Quit\tCtrl+Q"));
    menu_bar->Append(file_menu, _("&File"));
    // Create help menu:
    wxMenu *help_menu = new wxMenu;
    help_menu->Append(SIGNAL_MENU_ABOUT, _("&About"));
    help_menu->AppendSeparator();
    help_menu->Append(SIGNAL_MENU_HELP, _("SPIN Editor help\tF1"));
    menu_bar->Append(help_menu, _("&Help"));

    SetMenuBar(menu_bar);

    // Create status bar:
    CreateStatusBar();
    SetStatusText(_("Ready"));

    // -----------------------
	wxFlexGridSizer *sizer;
	sizer = new wxFlexGridSizer(/* rows: */ 1, /* cols: */ 2, 0, 0);
	sizer->SetFlexibleDirection(wxBOTH);
	sizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);
	
	//treeControl_ = new wxTreeCtrl(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTR_DEFAULT_STYLE);
	//sizer->Add(treeControl_, 0, wxALL, 5);
    wxButton *dummy0 = new wxButton(this, SIGNAL_BUTTON_HELLO_0, _("Hello"), wxDefaultPosition, wxDefaultSize, 0);
	sizer->Add(dummy0, wxEXPAND | 0);
    wxButton *dummy1 = new wxButton(this, SIGNAL_BUTTON_HELLO_1, _("Hello"), wxDefaultPosition, wxDefaultSize, 0);
	sizer->Add(dummy1, wxEXPAND | 0);
	
	this->SetSizer(sizer);
	this->Layout();
    // -----------------------

    // Listing node types:
    std::vector<std::string> nodeTypes = introspection::listSpinNodeTypes();
    std::cout << "SPIN Node types:" << std::endl;
    for (std::vector<std::string>::iterator iter = nodeTypes.begin(); iter != nodeTypes.end(); ++iter)
        std::cout << " * " << *iter << std::endl;
}

void MainWindow::OnQuit(wxCommandEvent& WXUNUSED(event))
{
    Close(TRUE);
}

wxString toString(const std::string &text)
{
    return wxString(text.c_str(), wxConvUTF8);
}

void MainWindow::OnAbout(wxCommandEvent& WXUNUSED(event))
{
    std::ostringstream os;
    os << PACKAGE_NAME << " " <<  wxString(_("version")).mb_str() << " " << PACKAGE_VERSION << std::endl << std::endl;
    os << wxString(_("Authors: ")).mb_str()  << "Mike Wozniewski, Zack Settel, Alexandre Quessy." << std::endl;
    os << wxString(_("License: ")).mb_str() << wxString(_("LGPL version 3")).mb_str() << std::endl;
    
    wxMessageBox(toString(os.str()),
        _("About the SPIN Editor"),
        wxOK | wxICON_INFORMATION, this);
}

const std::string HELP_URL = "http://www.spinframework.org/content/how_to_spin_editor";

void MainWindow::OnHelp(wxCommandEvent& WXUNUSED(event))
{
    bool success = wxLaunchDefaultBrowser(toString(HELP_URL));
    if (! success)
    {
        wxMessageBox(_("Could not launch a Web browser."),
            _("SPIN Editor Error"),
            wxOK | wxICON_ERROR, this);
    }
    else
    {
        wxMessageBox(_("Successfully launched a Web browser showing the documentation."),
            _("SPIN Editor Information"),
            wxOK | wxICON_INFORMATION, this);
    }
}

} // end of namespace editor
} // end of namespace spin

