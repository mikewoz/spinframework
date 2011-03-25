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
#include <wx/stdpaths.h>
#include <wx/aboutdlg.h>
#include <wx/artprov.h>
#include "main_window.h"
#include "config.h"
#include "introspection.h"
#include <sstream>

#include "wxSpinTreeCtrl.h"
#include "spinApp.h"
#include "spinBaseContext.h"
#include "SceneManager.h"

namespace spineditor
{

// mike: can we do this like this? or should we use the enum in the .h?
// 2011-03-22:aalex: well, I think so, but we still need to do the BEGIN_EVENT_TABLE thing
const long ID_SPIN_TREE = wxNewId();

/**
 * Connect handlers to the signal they handle.
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
    wxLog::SetActiveTarget(new wxLogStream(&std::cout));
    wxLog::SetVerbose(true);
    wxLogInfo(stringToWxString("XXXXXXXXXXXX set up logger"));
    wxLogInfo(stringToWxString("Running the MainWindow constructor."));

    // set icon
    wxIcon SPINIcon(spinApp::Instance().sceneManager->resourcesPath + wxT("/images/spin_48x48.png"), wxBITMAP_TYPE_PNG);
    SetIcon(SPINIcon);

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

    // Create add menu:
    //wxMenu *add_menu = new wxMenu;
    //add_menu->Append(SIGNAL_MENU_HELP, _("SPIN Editor help\tF1"));
    //add_bar->Append(add_menu, _("&Add"));

    // finally:
    SetMenuBar(menu_bar);

    // Create status bar:
    CreateStatusBar();
    SetStatusText(_("Ready"));
    wxLogInfo(stringToWxString("Ready\n"));

    // -----------------------
	wxFlexGridSizer *sizer;
	sizer = new wxFlexGridSizer(/* rows: */ 1, /* cols: */ 3, 0, 0);
	sizer->SetFlexibleDirection(wxBOTH);
	sizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);
	
	//treeControl_ = new wxTreeCtrl(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTR_DEFAULT_STYLE);
	//sizer->Add(treeControl_, 0, wxALL, 5);
    wxButton *dummy0 = new wxButton(this, SIGNAL_BUTTON_HELLO_0, _("Hello"), wxDefaultPosition, wxDefaultSize, 0);
	sizer->Add(dummy0, wxEXPAND | 0);
    wxButton *dummy1 = new wxButton(this, SIGNAL_BUTTON_HELLO_1, _("Hello"), wxDefaultPosition, wxDefaultSize, 0);
	sizer->Add(dummy1, wxEXPAND | 0);
	
	wxSpinTreeCtrl *tree = new wxSpinTreeCtrl(this,ID_SPIN_TREE,wxDefaultPosition,wxDefaultSize,wxTR_DEFAULT_STYLE,wxDefaultValidator,_T("ID_SPIN_TREE"));
    sizer->Add(tree, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    //sizer->Add(tree, wxEXPAND | 0);

	this->SetSizer(sizer);
	this->Layout();
    wxLogInfo(stringToWxString("Done laying out window contents."));
    // -----------------------

	tree->BuildTree(spinApp::Instance().sceneManager->worldNode.get());

    // Listing node types:
    std::vector<std::string> nodeTypes = introspection::listSpinNodeTypes();
    std::cout << "SPIN Node types:" << std::endl;
    for (std::vector<std::string>::iterator iter = nodeTypes.begin(); iter != nodeTypes.end(); ++iter)
        std::cout << " * " << (*iter) << std::endl;
}

void MainWindow::OnQuit(wxCommandEvent& WXUNUSED(event))
{
    std::cout << "Got MainWindow::OnQuit" << std::endl;
    spinApp::Instance().getContext()->stop();
    Close(TRUE);
}

wxString stringToWxString(const std::string &text)
{
    return wxString(text.c_str(), wxConvUTF8);
}

void MainWindow::log(LogLevel level, const std::string &text)
{
    switch (level)
    {
        case LOG_INFO:
            wxLogInfo(stringToWxString(text));
            break;
        case LOG_DEBUG:
            wxLogDebug(stringToWxString(text));
            break;
        default:
            wxLogInfo(stringToWxString(text));
            break;
    }
}

void MainWindow::OnAbout(wxCommandEvent& WXUNUSED(event))
{
    /*
    std::ostringstream os;
    os << PACKAGE_NAME << " " <<  wxString(_("version")).mb_str() << " " << PACKAGE_VERSION << std::endl << std::endl;
    os << wxString(_("Authors: ")).mb_str()  << "Mike Wozniewski, Zack Settel, Alexandre Quessy." << std::endl;
    os << wxString(_("License: ")).mb_str() << wxString(_("LGPL version 3")).mb_str() << std::endl;
    
    wxMessageBox(stringToWxString(os.str()),
        _("About the SPIN Editor"),
        wxOK | wxICON_INFORMATION, this);
*/

    wxIcon SPINIcon(spinApp::Instance().sceneManager->resourcesPath + wxT("/images/spin_48x48.png"), wxBITMAP_TYPE_PNG);
    wxAboutDialogInfo info;
    info.SetVersion(_(PACKAGE_VERSION));
    info.SetName(_("SPIN Editor"));
    info.SetDescription(_("Part of the SPIN Framework\nhttp://www.spinframework.org\n\nLicense: LGPL version 3"));
    info.SetCopyright(_T("Copyright (C) 2011 Mike Wozniewski, Zack Settel, Alexandre Quessy."));
    info.SetIcon(SPINIcon);

    wxAboutBox(info);
}

const std::string HELP_URL = "http://www.spinframework.org/content/how_to_spin_editor";

void MainWindow::OnHelp(wxCommandEvent& WXUNUSED(event))
{
    bool success = wxLaunchDefaultBrowser(stringToWxString(HELP_URL));
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

} // end of namespace spineditor

