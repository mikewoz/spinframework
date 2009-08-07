// -----------------------------------------------------------------------------
// |    ___  ___  _  _ _     ___                                        _      |
// |   / __>| . \| || \ |   | __>_ _  ___ ._ _ _  ___  _ _ _  ___  _ _ | |__   |
// |   \__ \|  _/| ||   |   | _>| '_><_> || ' ' |/ ._>| | | |/ . \| '_>| / /   |
// |   <___/|_|  |_||_\_|   |_| |_|  <___||_|_|_|\___.|__/_/ \___/|_|  |_\_\   |
// |                                                                           |
// |---------------------------------------------------------------------------|
//
// http://spinframework.sourceforge.net
// Copyright (C) 2009 Mike Wozniewski, Zack Settel
//
// Developed/Maintained by:
//    Mike Wozniewski (http://www.mikewoz.com)
//    Zack Settel (http://www.sheefa.net/zack)
//
// Principle Partners:
//    Shared Reality Lab, McGill University (http://www.cim.mcgill.ca/sre)
//    La Societe des Arts Technologiques (http://www.sat.qc.ca)
//
// Funding by:
//    NSERC/Canada Council for the Arts - New Media Initiative
//    Heritage Canada
//    Ministere du Developpement economique, de l'Innovation et de l'Exportation
//
// -----------------------------------------------------------------------------
//  This file is part of the SPIN Framework.
//
//  SPIN Framework is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  SPIN Framework is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the Lesser GNU General Public License
//  along with SPIN Framework. If not, see <http://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------

#include <string>
#include <iostream>
#include <sstream>
#include <ostream>

#include "vessWX.h"
#include "wxVessMain.h"
#include "vessThreads.h"
#include "wxVessEditor.h"
#include "wxVessRenderer.h"

#include <wx/msgdlg.h>
#include <wx/log.h>
#include <wx/ffile.h>
#include <wx/aboutdlg.h>
#include <wx/artprov.h>
#include <wx/stdpaths.h>

#include "../images/icon_tree.xpm"
#include "../images/icon_3Dsphere.xpm"

extern vessThread *vess;


//(*InternalHeaders(wxVessFrame)
#include <wx/intl.h>
#include <wx/string.h>
//*)

//helper functions
enum wxbuildinfoformat {
    short_f, long_f };

wxString wxbuildinfo(wxbuildinfoformat format)
{
    wxString wxbuild(wxVERSION_STRING);

    if (format == long_f )
    {
#if defined(__WXMSW__)
        wxbuild << _T("-Windows");
#elif defined(__UNIX__)
        wxbuild << _T("-Linux");
#endif

#if wxUSE_UNICODE
        wxbuild << _T("-Unicode build");
#else
        wxbuild << _T("-ANSI build");
#endif // wxUSE_UNICODE
    }

    return wxbuild;
}

//(*IdInit(wxVessMain)
const long wxVessMain::ID_STATICTEXT1 = wxNewId();
const long wxVessMain::ID_RADIOBUTTON1 = wxNewId();
const long wxVessMain::ID_RADIOBUTTON2 = wxNewId();
const long wxVessMain::ID_TOGGLEBUTTON2 = wxNewId();
const long wxVessMain::ID_PANEL1 = wxNewId();
const long wxVessMain::ID_TEXTCTRL6 = wxNewId();
const long wxVessMain::ID_SCROLLEDWINDOW1 = wxNewId();
const long wxVessMain::ID_SPLITTERWINDOW1 = wxNewId();
const long wxVessMain::idMenuOpen = wxNewId();
const long wxVessMain::idMenuSave = wxNewId();
const long wxVessMain::idMenuQuit = wxNewId();
const long wxVessMain::idMenuShowConfig = wxNewId();
const long wxVessMain::idMenuShowEditor = wxNewId();
const long wxVessMain::idMenuShowRenderer = wxNewId();
const long wxVessMain::idMenuAbout = wxNewId();
const long wxVessMain::ID_STATUSBAR1 = wxNewId();
const long wxVessMain::wxVess_load = wxNewId();
const long wxVessMain::wxVess_Save = wxNewId();
const long wxVessMain::wxVess_showConfig = wxNewId();
const long wxVessMain::wxVess_showRenderer = wxNewId();
const long wxVessMain::wxVess_showEditor = wxNewId();
const long wxVessMain::ID_TOOLBAR1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(wxVessMain,wxFrame)
    //(*EventTable(wxVessMain)
    //*)
END_EVENT_TABLE()

wxVessMain::wxVessMain(wxWindow* parent,wxWindowID id)
{
    //(*Initialize(wxVessMain)
    wxMenuItem* MenuItem1;
    wxBoxSizer* BoxSizer2;
    wxMenu* Menu1;
    wxBoxSizer* BoxSizer1;
    wxFlexGridSizer* FlexGridSizer1;
    wxMenu* Menu2;
    wxMenuBar* wxVess_MenuBar;

    Create(parent, wxID_ANY, _("SPIN Framework"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("wxID_ANY"));
    SetClientSize(wxSize(500,400));
    {
    	wxIcon FrameIcon;
    	FrameIcon.CopyFromBitmap(wxBitmap(wxImage(_T("../images/logo_SPIN_simple.png"))));
    	SetIcon(FrameIcon);
    }
    mainSplitter = new wxSplitterWindow(this, ID_SPLITTERWINDOW1, wxDefaultPosition, wxDefaultSize, wxSP_NOBORDER|wxNO_BORDER, _T("ID_SPLITTERWINDOW1"));
    mainSplitter->SetMinSize(wxSize(40,40));
    mainSplitter->SetMinimumPaneSize(40);
    mainPanel = new wxPanel(mainSplitter, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
    BoxSizer1 = new wxBoxSizer(wxVERTICAL);
    FlexGridSizer1 = new wxFlexGridSizer(0, 4, 0, 0);
    StaticText1 = new wxStaticText(mainPanel, ID_STATICTEXT1, _("Run as:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
    FlexGridSizer1->Add(StaticText1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    vessRadio_master = new wxRadioButton(mainPanel, ID_RADIOBUTTON1, _("Server"), wxDefaultPosition, wxDefaultSize, wxRB_GROUP, wxDefaultValidator, _T("ID_RADIOBUTTON1"));
    FlexGridSizer1->Add(vessRadio_master, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    vessRadio_slave = new wxRadioButton(mainPanel, ID_RADIOBUTTON2, _("Client"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_RADIOBUTTON2"));
    vessRadio_slave->SetValue(true);
    FlexGridSizer1->Add(vessRadio_slave, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StartStop = new wxToggleButton(mainPanel, ID_TOGGLEBUTTON2, _("Start"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TOGGLEBUTTON2"));
    FlexGridSizer1->Add(StartStop, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer1->Add(FlexGridSizer1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    mainPanel->SetSizer(BoxSizer1);
    BoxSizer1->Fit(mainPanel);
    BoxSizer1->SetSizeHints(mainPanel);
    logPanel = new wxScrolledWindow(mainSplitter, ID_SCROLLEDWINDOW1, wxDefaultPosition, wxDefaultSize, wxHSCROLL|wxVSCROLL, _T("ID_SCROLLEDWINDOW1"));
    BoxSizer2 = new wxBoxSizer(wxHORIZONTAL);
    logTextCtrl = new wxTextCtrl(logPanel, ID_TEXTCTRL6, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE|wxTE_READONLY|wxHSCROLL|wxTE_DONTWRAP|wxVSCROLL, wxDefaultValidator, _T("ID_TEXTCTRL6"));
    BoxSizer2->Add(logTextCtrl, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    logPanel->SetSizer(BoxSizer2);
    BoxSizer2->Fit(logPanel);
    BoxSizer2->SetSizeHints(logPanel);
    mainSplitter->SplitHorizontally(mainPanel, logPanel);
    wxVess_MenuBar = new wxMenuBar();
    Menu1 = new wxMenu();
    MenuItem3 = new wxMenuItem(Menu1, idMenuOpen, _("&Load Scene\tCtrl-O"), _("Load a  scene from .xml"), wxITEM_NORMAL);
    Menu1->Append(MenuItem3);
    MenuItem4 = new wxMenuItem(Menu1, idMenuSave, _("&Save Scene\tCtrl-S"), _("Save the current scene to .xml"), wxITEM_NORMAL);
    Menu1->Append(MenuItem4);
    MenuItem1 = new wxMenuItem(Menu1, idMenuQuit, _("&Quit\tAlt-F4"), _("Quit the application"), wxITEM_NORMAL);
    Menu1->Append(MenuItem1);
    wxVess_MenuBar->Append(Menu1, _("&File"));
    Menu3 = new wxMenu();
    MenuItem5 = new wxMenuItem(Menu3, idMenuShowConfig, _("Show &Config\tF2"), _("Show the configuration panel"), wxITEM_NORMAL);
    Menu3->Append(MenuItem5);
    MenuItem6 = new wxMenuItem(Menu3, idMenuShowEditor, _("Show &Editor\tCtrl-E"), _("Show the editor"), wxITEM_NORMAL);
    Menu3->Append(MenuItem6);
    MenuItem7 = new wxMenuItem(Menu3, idMenuShowRenderer, _("Show &Renderer\tCtrl-R"), _("Show the 3D renderer window"), wxITEM_NORMAL);
    Menu3->Append(MenuItem7);
    wxVess_MenuBar->Append(Menu3, _("&View"));
    Menu2 = new wxMenu();
    wxVessMenu_About = new wxMenuItem(Menu2, idMenuAbout, _("&About"), wxEmptyString, wxITEM_NORMAL);
    Menu2->Append(wxVessMenu_About);
    wxVess_MenuBar->Append(Menu2, _("Help"));
    SetMenuBar(wxVess_MenuBar);
    wxVess_StatusBar = new wxStatusBar(this, ID_STATUSBAR1, 0, _T("ID_STATUSBAR1"));
    int __wxStatusBarWidths_1[1] = { -1 };
    int __wxStatusBarStyles_1[1] = { wxSB_NORMAL };
    wxVess_StatusBar->SetFieldsCount(1,__wxStatusBarWidths_1);
    wxVess_StatusBar->SetStatusStyles(1,__wxStatusBarStyles_1);
    SetStatusBar(wxVess_StatusBar);
    wxVess_ToolBar = new wxToolBar(this, ID_TOOLBAR1, wxDefaultPosition, wxDefaultSize, wxTB_HORIZONTAL|wxNO_BORDER, _T("ID_TOOLBAR1"));
    wxVess_ToolBar->SetToolBitmapSize(wxSize(24,24));
    ToolBarItem1 = wxVess_ToolBar->AddTool(wxVess_load, _("Load Scene"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FILE_OPEN")),wxART_TOOLBAR), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FILE_OPEN")),wxART_TOOLBAR), wxITEM_NORMAL, _("Load a scene from .xml file"), _("Load a scene from .xml file"));
    ToolBarItem2 = wxVess_ToolBar->AddTool(wxVess_Save, _("Save Scene"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FILE_SAVE")),wxART_TOOLBAR), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FILE_SAVE")),wxART_TOOLBAR), wxITEM_NORMAL, _("Save current scene"), _("Save current scene"));
    wxVess_ToolBar->AddSeparator();
    ToolBarItem3 = wxVess_ToolBar->AddTool(wxVess_showConfig, _("Configuration"), wxBitmap(wxImage(_T("../images/icon_network.gif"))), wxBitmap(wxImage(_T("../images/icon_network.gif"))), wxITEM_NORMAL, _("Show configuration panel"), _("Show configuration panel"));
    wxVess_ToolBar->AddSeparator();
    ToolBarItem4 = wxVess_ToolBar->AddTool(wxVess_showRenderer, _("Renderer"), wxBitmap(wxImage(_T("../images/icon_3Dview.gif"))), wxBitmap(wxImage(_T("../images/icon_3Dview.gif"))), wxITEM_NORMAL, _("Show the rendered 3D view"), _("Show the rendered 3D view"));
    ToolBarItem5 = wxVess_ToolBar->AddTool(wxVess_showEditor, _("Editor"), wxBitmap(wxImage(_T("../images/icon_tree2.gif"))), wxBitmap(wxImage(_T("../images/icon_tree2.gif"))), wxITEM_NORMAL, _("Show the editor"), _("Show the editor"));
    wxVess_ToolBar->Realize();
    SetToolBar(wxVess_ToolBar);

    Connect(ID_RADIOBUTTON1,wxEVT_COMMAND_RADIOBUTTON_SELECTED,(wxObjectEventFunction)&wxVessMain::OnVessModeChange);
    Connect(ID_RADIOBUTTON2,wxEVT_COMMAND_RADIOBUTTON_SELECTED,(wxObjectEventFunction)&wxVessMain::OnVessModeChange);
    Connect(ID_TOGGLEBUTTON2,wxEVT_COMMAND_TOGGLEBUTTON_CLICKED,(wxObjectEventFunction)&wxVessMain::OnStartStopToggle);
    Connect(idMenuOpen,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&wxVessMain::OnLoadScene);
    Connect(idMenuSave,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&wxVessMain::OnSaveScene);
    Connect(idMenuQuit,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&wxVessMain::OnQuit);
    Connect(idMenuShowConfig,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&wxVessMain::OnShowConfig);
    Connect(idMenuShowEditor,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&wxVessMain::OnShowEditor);
    Connect(idMenuShowRenderer,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&wxVessMain::OnShowRenderer);
    Connect(idMenuAbout,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&wxVessMain::OnAbout);
    Connect(wxVess_load,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&wxVessMain::OnLoadScene);
    Connect(wxVess_Save,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&wxVessMain::OnSaveScene);
    Connect(wxVess_showConfig,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&wxVessMain::OnShowConfig);
    Connect(wxVess_showRenderer,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&wxVessMain::OnShowRenderer);
    Connect(wxVess_showEditor,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&wxVessMain::OnShowEditor);
    Connect(wxID_ANY,wxEVT_CLOSE_WINDOW,(wxObjectEventFunction)&wxVessMain::OnClose);
    //*)

    // on the Mac,
    #ifdef __WXMAC__
    wxApp::s_macAboutMenuItemId = wxVessMenu_About->GetId();
    #endif

    // sash position doesn't seem to work in wxSmith, so do it manually:
    mainSplitter->SetSashPosition(40);

    vessSettingsFrame = new wxVessSettings(0);
    vessSettingsFrame->vessID->SetValue( wxString( vess->id.c_str(), wxConvUTF8 ));
    vessSettingsFrame->rxAddr->SetValue( wxString( vess->rxAddr.c_str(), wxConvUTF8 ));
    vessSettingsFrame->rxPort->SetValue( wxString( vess->rxPort.c_str(), wxConvUTF8 ));
    vessSettingsFrame->txAddr->SetValue( wxString( vess->txAddr.c_str(), wxConvUTF8 ));
    vessSettingsFrame->txPort->SetValue( wxString( vess->txPort.c_str(), wxConvUTF8 ));


    //wxFFile logFile(wxT("vessWX.log"),wxT("w+"));
    //wxLogTextCtrl w = new wxLogTextCtrl(logTextCtrl);
    //wxLog::SetActiveTarget(w);
    wxLog::SetActiveTarget(new wxLogTextCtrl(logTextCtrl));
    //wxLogChain *LC = new wxLogChain(new wxLogStderr(logFile.fp()));


    vessLog log("vess.log");
    log.enable_wxlog(true);
    log.enable_cout(false);

    log << "Started vessLog" << std::endl;



#if wxUSE_STD_IOSTREAM
    //redirector = new wxStreamToTextRedirector(logTextCtrl);
    oldstdout = std::cout.rdbuf();
    std::cout.rdbuf(logTextCtrl);
#endif

    wxLogMessage(wxT("vessWX started"));


#if wxUSE_STD_IOSTREAM
#else
    wxLogMessage(wxT("Oops. The log window is not yet supported on this platform...\nAll messages will be displayed in the console instead."));
#endif

}

wxVessMain::~wxVessMain()
{
    //(*Destroy(wxVessMain)
    //*)

#if wxUSE_STD_IOSTREAM
    std::cout.rdbuf(oldstdout);
#endif
}

void wxVessMain::OnQuit(wxCommandEvent& event)
{
    Close();
}

void wxVessMain::OnAbout(wxCommandEvent& event)
{
    wxString msg = wxbuildinfo(long_f);
   // wxMessageBox(msg, _("Welcome to..."));

    //wxIcon SPINIcon(wxT("images/logo_SPIN.tif"), wxBITMAP_TYPE_TIF);
    wxIcon SPINIcon(wxT("../images/logo_SPIN_simple.png"), wxBITMAP_TYPE_PNG);
    //wxIcon SPINIcon(wxT("@executable_path/../images/logo_SPIN_simple.png"), wxBITMAP_TYPE_PNG);

    wxAboutDialogInfo info;
    info.SetVersion(_("0.9"));
    info.SetName(_("SPIN Framework"));
    info.SetDescription(_("The Spatial Interaction Framework\nhttp://spinframework.sourceforge.net"));
    info.SetCopyright(_T("Copyright (C) 2009. Mike Wozniewski, Zack Settel"));
    info.SetIcon(SPINIcon);

    wxAboutBox(info);

}

void wxVessMain::OnLoadScene(wxCommandEvent& event)
{
    if (vess->isRunning())
    {
		wxFileDialog* d = new wxFileDialog( this, wxT("Load Scene"), wxT(""), wxT(""), wxT("*.xml"), wxOPEN, wxDefaultPosition);

		if ( d->ShowModal() == wxID_OK )
		{

		    std::cout << "Loading scene from file: " << d->GetPath().mb_str() << std::endl;
		   	//vess->sceneManager->loadXML( d->GetPath().mb_str() );
		   	vess->sendSceneMessage("ss", "load", (const char*) d->GetPath().mb_str(), LO_ARGS_END);
		}
    }
    else {
	    wxMessageDialog *dlg = new wxMessageDialog(this, wxT("The VESS server needs to be started before you can load scenes."), wxT("VESS not running"), wxOK|wxICON_ERROR|wxSTAY_ON_TOP);
	    dlg->ShowModal();
	}

}

void wxVessMain::OnSaveScene(wxCommandEvent& event)
{
    if (vess->isRunning())
    {
		wxFileDialog* d = new wxFileDialog( this, wxT("Save Scene"), wxT(""), wxT(""), wxT("*.xml"), wxSAVE, wxDefaultPosition);

		if ( d->ShowModal() == wxID_OK )
		{
			vess->sendSceneMessage("ss", "save", (const char*) d->GetPath().mb_str(), LO_ARGS_END);
			/*
		   	if (vess->sceneManager->saveXML( d->GetPath().mb_str() ))
		   	{
		   		std::cout << "Saving scene to: " << d->GetPath().mb_str() << std::endl;
		   	} else {
		   		std::cout << "Error when saving " << d->GetPath().mb_str() << std::endl;
		   	}
		   	*/
		}
    }
    else {
	    wxMessageDialog *dlg = new wxMessageDialog(this, wxT("VESS server is not running, so no scene to save."), wxT("VESS not running"), wxOK|wxICON_ERROR|wxSTAY_ON_TOP);
	    dlg->ShowModal();
	}
}

void wxVessMain::OnShowConfig(wxCommandEvent& event)
{
    vessSettingsFrame->Show();
}

void wxVessMain::OnShowEditor(wxCommandEvent& event)
{
    if (vess->isRunning())
    {
        wxVessEditor* vessEditor = new wxVessEditor(0);
        vessEditor->Show();
    } else {
        wxMessageDialog *dlg = new wxMessageDialog(this, wxT("The VESS server needs to be started before you can launch the editor."), wxT("VESS not running"), wxOK|wxICON_ERROR|wxSTAY_ON_TOP);
        dlg->ShowModal();
    }
}

void wxVessMain::OnShowRenderer(wxCommandEvent& event)
{
    if (vess->isRunning())
    {
        wxVessRenderer* vessRenderer = new wxVessRenderer(0);
        vessRenderer->Show();
    } else {
        wxMessageDialog *dlg = new wxMessageDialog(this, wxT("The VESS server needs to be started before you can launch the viewer."), wxT("VESS not running"), wxOK|wxICON_ERROR|wxSTAY_ON_TOP);
        dlg->ShowModal();
    }
}


void wxVessMain::OnStartStopToggle(wxCommandEvent& event)
{
    if (event.IsChecked())
    {
        vess->id = std::string(vessSettingsFrame->vessID->GetValue().mb_str());
        vess->rxAddr = std::string(vessSettingsFrame->rxAddr->GetValue().mb_str());
        vess->rxPort = std::string(vessSettingsFrame->rxPort->GetValue().mb_str());
        vess->txAddr = std::string(vessSettingsFrame->txAddr->GetValue().mb_str());
        vess->txPort = std::string(vessSettingsFrame->txPort->GetValue().mb_str());

        if (vessRadio_master->GetValue()) vess->setMode(vessThread::SERVER_MODE);
        else vess->setMode(vessThread::LISTENER_MODE);

        vess->start();

        // we want the viewer to be able to show all graphical items:
        vess->sceneManager->isGraphical = true;

        StartStop->SetLabel(wxT("Stop"));

    }

    else {
        // TODO: close all other frames (viewer, editor)
        vess->stop();
        StartStop->SetLabel(wxT("Start"));
    }
}



void wxVessMain::OnClose(wxCloseEvent& event)
{
    wxMessageDialog *dlg = new wxMessageDialog(this, wxT("Are you sure that you want to quit?"), wxT("Quit?"), wxOK|wxCANCEL|wxICON_ERROR|wxSTAY_ON_TOP);

    if ( dlg->ShowModal() == wxID_OK )
    {
        vess->stop();

        if (vessSettingsFrame) vessSettingsFrame->Destroy();
        //if (vessRenderer) vessRenderer->Destroy();
        //if (vessEditor) vessEditor->Destroy();

        this->Destroy();

    } else event.Veto();
}

void wxVessMain::OnVessModeChange(wxCommandEvent& event)
{

    std::cout << "Changed modes for VESS. You must stop and restart before this takes effect." << std::endl;
}
