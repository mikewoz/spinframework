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
    EVT_MENU(ID_Quit, MainWindow::OnQuit)
    EVT_MENU(ID_About, MainWindow::OnAbout)
END_EVENT_TABLE()

MainWindow::MainWindow(const wxString& title, const wxPoint& pos, const wxSize& size)
: 
    wxFrame(NULL, -1, title, pos, size)
{
    wxMenu *menuFile = new wxMenu;

    menuFile->Append(ID_About, _("&About..."));
    menuFile->AppendSeparator();
    menuFile->Append(ID_Quit, _("E&xit"));

    wxMenuBar *menuBar = new wxMenuBar;
    menuBar->Append(menuFile, _("&File"));

    SetMenuBar(menuBar);

    CreateStatusBar();
    SetStatusText(_("Starting the SPIN Editor"));

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

void MainWindow::OnAbout(wxCommandEvent& WXUNUSED(event))
{
    std::ostringstream os;
    os << PACKAGE_NAME << " " <<  wxString(_("version")).mb_str() << " " << PACKAGE_VERSION << std::endl << std::endl;
    os << wxString(_("Authors: ")).mb_str()  << "Mike Wozniewski, Zack Settel, Alexandre Quessy." << std::endl;
    os << wxString(_("License: ")).mb_str() << wxString(_("LGPL version 3")).mb_str() << std::endl;
    
    wxMessageBox(wxString(os.str().c_str(), wxConvUTF8),
                  _("About the SPIN Editor"),
                  wxOK | wxICON_INFORMATION, this);
}

} // end of namespace editor
} // end of namespace spin

