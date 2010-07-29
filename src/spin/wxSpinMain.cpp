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
//  You should have received a copy of the GNU Lesser General Public License
//  along with SPIN Framework. If not, see <http://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------

#include <string>
#include <iostream>
#include <sstream>
#include <ostream>

#include "spinWX.h"
#include "wxSpinMain.h"
#include "spinContext.h"
#include "wxSpinEditor.h"
#include "wxSpinRenderer.h"

#include <wx/msgdlg.h>
#include <wx/log.h>
#include <wx/ffile.h>
#include <wx/aboutdlg.h>
#include <wx/artprov.h>

extern spinContext *spin;
extern wxString resourcesPath;

//(*InternalHeaders(wxSpinFrame)
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

//(*IdInit(wxSpinMain)
const long wxSpinMain::ID_STATICTEXT1 = wxNewId();
const long wxSpinMain::ID_RADIOBUTTON1 = wxNewId();
const long wxSpinMain::ID_RADIOBUTTON2 = wxNewId();
const long wxSpinMain::ID_TOGGLEBUTTON2 = wxNewId();
const long wxSpinMain::ID_PANEL1 = wxNewId();
const long wxSpinMain::ID_TEXTCTRL6 = wxNewId();
const long wxSpinMain::ID_SCROLLEDWINDOW1 = wxNewId();
const long wxSpinMain::ID_SPLITTERWINDOW1 = wxNewId();
const long wxSpinMain::idMenuOpen = wxNewId();
const long wxSpinMain::idMenuSave = wxNewId();
const long wxSpinMain::idMenuQuit = wxNewId();
const long wxSpinMain::idMenuShowConfig = wxNewId();
const long wxSpinMain::idMenuShowEditor = wxNewId();
const long wxSpinMain::idMenuShowRenderer = wxNewId();
const long wxSpinMain::idMenuAbout = wxNewId();
const long wxSpinMain::ID_STATUSBAR1 = wxNewId();
const long wxSpinMain::wxSpin_load = wxNewId();
const long wxSpinMain::wxSpin_Save = wxNewId();
const long wxSpinMain::wxSpin_showConfig = wxNewId();
const long wxSpinMain::wxSpin_showRenderer = wxNewId();
const long wxSpinMain::wxSpin_showEditor = wxNewId();
const long wxSpinMain::ID_TOOLBAR1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(wxSpinMain,wxFrame)
    //(*EventTable(wxSpinMain)
    //*)
END_EVENT_TABLE()

wxSpinMain::wxSpinMain(wxWindow* parent,wxWindowID id)
{



    //(*Initialize(wxSpinMain)
    wxMenuBar* wxSpin_MenuBar;
    wxMenuItem* MenuItem1;
    wxBoxSizer* BoxSizer2;
    wxMenu* Menu1;
    wxBoxSizer* BoxSizer1;
    wxFlexGridSizer* FlexGridSizer1;
    wxMenu* Menu2;
    
    Create(parent, wxID_ANY, _("SPIN Framework"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("wxID_ANY"));
    SetClientSize(wxSize(500,400));
    mainSplitter = new wxSplitterWindow(this, ID_SPLITTERWINDOW1, wxDefaultPosition, wxDefaultSize, wxSP_NOBORDER|wxNO_BORDER, _T("ID_SPLITTERWINDOW1"));
    mainSplitter->SetMinSize(wxSize(40,40));
    mainSplitter->SetMinimumPaneSize(40);
    mainPanel = new wxPanel(mainSplitter, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
    BoxSizer1 = new wxBoxSizer(wxVERTICAL);
    FlexGridSizer1 = new wxFlexGridSizer(0, 4, 0, 0);
    StaticText1 = new wxStaticText(mainPanel, ID_STATICTEXT1, _("Run as:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
    FlexGridSizer1->Add(StaticText1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    spinRadio_master = new wxRadioButton(mainPanel, ID_RADIOBUTTON1, _("Server"), wxDefaultPosition, wxDefaultSize, wxRB_GROUP, wxDefaultValidator, _T("ID_RADIOBUTTON1"));
    FlexGridSizer1->Add(spinRadio_master, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    spinRadio_slave = new wxRadioButton(mainPanel, ID_RADIOBUTTON2, _("Client"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_RADIOBUTTON2"));
    spinRadio_slave->SetValue(true);
    FlexGridSizer1->Add(spinRadio_slave, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
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
    wxSpin_MenuBar = new wxMenuBar();
    Menu1 = new wxMenu();
    MenuItem3 = new wxMenuItem(Menu1, idMenuOpen, _("&Load Scene\tCtrl-O"), _("Load a  scene from .xml"), wxITEM_NORMAL);
    Menu1->Append(MenuItem3);
    MenuItem4 = new wxMenuItem(Menu1, idMenuSave, _("&Save Scene\tCtrl-S"), _("Save the current scene to .xml"), wxITEM_NORMAL);
    Menu1->Append(MenuItem4);
    MenuItem1 = new wxMenuItem(Menu1, idMenuQuit, _("&Quit\tAlt-F4"), _("Quit the application"), wxITEM_NORMAL);
    Menu1->Append(MenuItem1);
    wxSpin_MenuBar->Append(Menu1, _("&File"));
    Menu3 = new wxMenu();
    MenuItem5 = new wxMenuItem(Menu3, idMenuShowConfig, _("Show &Config\tF2"), _("Show the configuration panel"), wxITEM_NORMAL);
    Menu3->Append(MenuItem5);
    MenuItem6 = new wxMenuItem(Menu3, idMenuShowEditor, _("Show &Editor\tCtrl-E"), _("Show the editor"), wxITEM_NORMAL);
    Menu3->Append(MenuItem6);
    MenuItem7 = new wxMenuItem(Menu3, idMenuShowRenderer, _("Show &Renderer\tCtrl-R"), _("Show the 3D renderer window"), wxITEM_NORMAL);
    Menu3->Append(MenuItem7);
    wxSpin_MenuBar->Append(Menu3, _("&View"));
    Menu2 = new wxMenu();
    wxSpinMenu_About = new wxMenuItem(Menu2, idMenuAbout, _("&About"), wxEmptyString, wxITEM_NORMAL);
    Menu2->Append(wxSpinMenu_About);
    wxSpin_MenuBar->Append(Menu2, _("Help"));
    SetMenuBar(wxSpin_MenuBar);
    wxSpin_StatusBar = new wxStatusBar(this, ID_STATUSBAR1, 0, _T("ID_STATUSBAR1"));
    int __wxStatusBarWidths_1[1] = { -1 };
    int __wxStatusBarStyles_1[1] = { wxSB_NORMAL };
    wxSpin_StatusBar->SetFieldsCount(1,__wxStatusBarWidths_1);
    wxSpin_StatusBar->SetStatusStyles(1,__wxStatusBarStyles_1);
    SetStatusBar(wxSpin_StatusBar);
    wxSpin_ToolBar = new wxToolBar(this, ID_TOOLBAR1, wxDefaultPosition, wxDefaultSize, wxTB_HORIZONTAL|wxNO_BORDER, _T("ID_TOOLBAR1"));
    wxSpin_ToolBar->SetToolBitmapSize(wxSize(24,24));
    ToolBarItem1 = wxSpin_ToolBar->AddTool(wxSpin_load, _("Load Scene"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FILE_OPEN")),wxART_TOOLBAR), wxNullBitmap, wxITEM_NORMAL, _("Load a scene from .xml file"), _("Load a scene from .xml file"));
    ToolBarItem2 = wxSpin_ToolBar->AddTool(wxSpin_Save, _("Save Scene"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FILE_SAVE")),wxART_TOOLBAR), wxNullBitmap, wxITEM_NORMAL, _("Save current scene"), _("Save current scene"));
    wxSpin_ToolBar->AddSeparator();
    ToolBarItem3 = wxSpin_ToolBar->AddTool(wxSpin_showConfig, _("Configuration"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_ERROR")),wxART_TOOLBAR), wxNullBitmap, wxITEM_NORMAL, _("Show configuration panel"), _("Show configuration panel"));
    wxSpin_ToolBar->AddSeparator();
    ToolBarItem4 = wxSpin_ToolBar->AddTool(wxSpin_showRenderer, _("Renderer"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_ERROR")),wxART_TOOLBAR), wxNullBitmap, wxITEM_NORMAL, _("Show the rendered 3D view"), _("Show the rendered 3D view"));
    ToolBarItem5 = wxSpin_ToolBar->AddTool(wxSpin_showEditor, _("Editor"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_ERROR")),wxART_TOOLBAR), wxNullBitmap, wxITEM_NORMAL, _("Show the editor"), _("Show the editor"));
    wxSpin_ToolBar->Realize();
    SetToolBar(wxSpin_ToolBar);
    
    Connect(ID_RADIOBUTTON1,wxEVT_COMMAND_RADIOBUTTON_SELECTED,(wxObjectEventFunction)&wxSpinMain::OnSpinModeChange);
    Connect(ID_RADIOBUTTON2,wxEVT_COMMAND_RADIOBUTTON_SELECTED,(wxObjectEventFunction)&wxSpinMain::OnSpinModeChange);
    Connect(ID_TOGGLEBUTTON2,wxEVT_COMMAND_TOGGLEBUTTON_CLICKED,(wxObjectEventFunction)&wxSpinMain::OnStartStopToggle);
    Connect(idMenuOpen,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&wxSpinMain::OnLoadScene);
    Connect(idMenuSave,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&wxSpinMain::OnSaveScene);
    Connect(idMenuQuit,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&wxSpinMain::OnQuit);
    Connect(idMenuShowConfig,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&wxSpinMain::OnShowConfig);
    Connect(idMenuShowEditor,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&wxSpinMain::OnShowEditor);
    Connect(idMenuShowRenderer,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&wxSpinMain::OnShowRenderer);
    Connect(idMenuAbout,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&wxSpinMain::OnAbout);
    Connect(wxSpin_load,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&wxSpinMain::OnLoadScene);
    Connect(wxSpin_Save,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&wxSpinMain::OnSaveScene);
    Connect(wxSpin_showConfig,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&wxSpinMain::OnShowConfig);
    Connect(wxSpin_showRenderer,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&wxSpinMain::OnShowRenderer);
    Connect(wxSpin_showEditor,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&wxSpinMain::OnShowEditor);
    Connect(wxID_ANY,wxEVT_CLOSE_WINDOW,(wxObjectEventFunction)&wxSpinMain::OnClose);
    //*)

    // on the Mac,
    #ifdef __WXMAC__
    wxApp::s_macAboutMenuItemId = wxSpinMenu_About->GetId();
    #endif

    wxIcon SPINIcon(resourcesPath + wxT("/logo_SPIN_simple.png"), wxBITMAP_TYPE_PNG);
    SetIcon(SPINIcon);

	// need to redo the icons with resourcePath:
	wxImage icon_network(resourcesPath + _T("/icon_network.gif"));
	icon_network.Rescale(32,32);
    wxSpin_ToolBar->SetToolNormalBitmap( wxSpin_showConfig, wxBitmap(icon_network) );

	wxImage icon_3Dview(resourcesPath + _T("/icon_3Dview.gif"));
	icon_3Dview.Rescale(24,24);
	wxSpin_ToolBar->SetToolNormalBitmap( wxSpin_showRenderer, wxBitmap(icon_3Dview) );

	wxImage icon_tree2(resourcesPath + _T("/icon_tree2.gif"));
	icon_tree2.Rescale(24,24);
    wxSpin_ToolBar->SetToolNormalBitmap( wxSpin_showEditor, wxBitmap(icon_tree2) );

	wxSpin_ToolBar->Realize();


    // sash position doesn't seem to work in wxSmith, so do it manually:
    mainSplitter->SetSashPosition(40);

    spinSettingsFrame = new wxSpinSettings(0);
    spinSettingsFrame->spinID->SetValue( wxString( spin->id.c_str(), wxConvUTF8 ));
    spinSettingsFrame->rxAddr->SetValue( wxString( spin->rxAddr.c_str(), wxConvUTF8 ));
    spinSettingsFrame->rxPort->SetValue( wxString( spin->rxPort.c_str(), wxConvUTF8 ));
    spinSettingsFrame->txAddr->SetValue( wxString( spin->txAddr.c_str(), wxConvUTF8 ));
    spinSettingsFrame->txPort->SetValue( wxString( spin->txPort.c_str(), wxConvUTF8 ));


    //wxFFile logFile(wxT("spinWX.log"),wxT("w+"));
    //wxLogTextCtrl w = new wxLogTextCtrl(logTextCtrl);
    //wxLog::SetActiveTarget(w);
    wxLog::SetActiveTarget(new wxLogTextCtrl(logTextCtrl));
    //wxLogChain *LC = new wxLogChain(new wxLogStderr(logFile.fp()));

/*

// TODO
    spinLogWX log(SPIN_DIRECTORY + "/log/spin.log");
    log.enable_wxlog(true);
    log.enable_cout(false);

    log << "Started spinLog" << std::endl;

    log << "Resources path: " << resourcesPath.mb_str() << std::endl;
*/

#if wxUSE_STD_IOSTREAM
    //redirector = new wxStreamToTextRedirector(logTextCtrl);
    oldstdout = std::cout.rdbuf();
    std::cout.rdbuf(logTextCtrl);
#endif

    wxLogMessage(wxT("spinWX started"));


#if wxUSE_STD_IOSTREAM
#else
    wxLogMessage(wxT("Oops. The log window is not yet supported on this platform...\nAll messages will be displayed in the console instead."));
#endif

}

wxSpinMain::~wxSpinMain()
{
    //(*Destroy(wxSpinMain)
    //*)

#if wxUSE_STD_IOSTREAM
    std::cout.rdbuf(oldstdout);
#endif
}

void wxSpinMain::OnQuit(wxCommandEvent& event)
{
    Close();
}

void wxSpinMain::OnAbout(wxCommandEvent& event)
{
    wxString msg = wxbuildinfo(long_f);
   // wxMessageBox(msg, _("Welcome to..."));

    //wxIcon SPINIcon(wxT("s/logo_SPIN.tif"), wxBITMAP_TYPE_TIF);
    wxIcon SPINIcon(resourcesPath + wxT("/logo_SPIN_simple.png"), wxBITMAP_TYPE_PNG);
    //wxIcon SPINIcon(wxT("@executable_path/../images/logo_SPIN_simple.png"), wxBITMAP_TYPE_PNG);

    wxAboutDialogInfo info;
    info.SetVersion(_("0.9"));
    info.SetName(_("SPIN Framework"));
    info.SetDescription(_("The Spatial Interaction Framework\nhttp://spinframework.sourceforge.net"));
    info.SetCopyright(_T("Copyright (C) 2009. Mike Wozniewski, Zack Settel"));
    info.SetIcon(SPINIcon);

    wxAboutBox(info);

}

void wxSpinMain::OnLoadScene(wxCommandEvent& event)
{
    if (spin->isRunning())
    {
		wxFileDialog* d = new wxFileDialog( this, wxT("Load Scene"), wxT(""), wxT(""), wxT("*.xml"), wxOPEN, wxDefaultPosition);

		if ( d->ShowModal() == wxID_OK )
		{

		    std::cout << "Loading scene from file: " << d->GetPath().mb_str() << std::endl;
		   	//spin->sceneManager->loadXML( d->GetPath().mb_str() );
		   	spin->sendSceneMessage("ss", "load", (const char*) d->GetPath().mb_str(), LO_ARGS_END);
		}
    }
    else {
	    wxMessageDialog *dlg = new wxMessageDialog(this, wxT("The server needs to be started before you can load scenes."), wxT("SPIN not running"), wxOK|wxICON_ERROR|wxSTAY_ON_TOP);
	    dlg->ShowModal();
	}

}

void wxSpinMain::OnSaveScene(wxCommandEvent& event)
{
    if (spin->isRunning())
    {
		wxFileDialog* d = new wxFileDialog( this, wxT("Save Scene"), wxT(""), wxT(""), wxT("*.xml"), wxSAVE, wxDefaultPosition);

		if ( d->ShowModal() == wxID_OK )
		{
			spin->sendSceneMessage("ss", "save", (const char*) d->GetPath().mb_str(), LO_ARGS_END);
			/*
		   	if (spin->sceneManager->saveXML( d->GetPath().mb_str() ))
		   	{
		   		std::cout << "Saving scene to: " << d->GetPath().mb_str() << std::endl;
		   	} else {
		   		std::cout << "Error when saving " << d->GetPath().mb_str() << std::endl;
		   	}
		   	*/
		}
    }
    else {
	    wxMessageDialog *dlg = new wxMessageDialog(this, wxT("The server is not running, so no scene to save."), wxT("SPIN not running"), wxOK|wxICON_ERROR|wxSTAY_ON_TOP);
	    dlg->ShowModal();
	}
}

void wxSpinMain::OnShowConfig(wxCommandEvent& event)
{
    spinSettingsFrame->Show();
}

void wxSpinMain::OnShowEditor(wxCommandEvent& event)
{
    if (spin->isRunning())
    {
        wxSpinEditor* spinEditor = new wxSpinEditor(0);
        spinEditor->Show();
    } else {
        wxMessageDialog *dlg = new wxMessageDialog(this, wxT("The server needs to be started before you can launch the editor."), wxT("SPIN not running"), wxOK|wxICON_ERROR|wxSTAY_ON_TOP);
        dlg->ShowModal();
    }
}

void wxSpinMain::OnShowRenderer(wxCommandEvent& event)
{
    if (spin->isRunning())
    {
        wxSpinRenderer* spinRenderer = new wxSpinRenderer(0);
        spinRenderer->Show();
    } else {
        wxMessageDialog *dlg = new wxMessageDialog(this, wxT("The server needs to be started before you can launch the viewer."), wxT("SPIN not running"), wxOK|wxICON_ERROR|wxSTAY_ON_TOP);
        dlg->ShowModal();
    }
}


void wxSpinMain::OnStartStopToggle(wxCommandEvent& event)
{
    if (event.IsChecked())
    {
        spin->id = std::string(spinSettingsFrame->spinID->GetValue().mb_str());
        spin->rxAddr = std::string(spinSettingsFrame->rxAddr->GetValue().mb_str());
        spin->rxPort = std::string(spinSettingsFrame->rxPort->GetValue().mb_str());
        spin->txAddr = std::string(spinSettingsFrame->txAddr->GetValue().mb_str());
        spin->txPort = std::string(spinSettingsFrame->txPort->GetValue().mb_str());

        if (spinRadio_master->GetValue()) spin->setMode(spinContext::SERVER_MODE);
        else spin->setMode(spinContext::LISTENER_MODE);

        spin->start();

        // we want the viewer to be able to show all graphical items:
        spin->sceneManager->setGraphical(true);

        StartStop->SetLabel(wxT("Stop"));

    }

    else {
        // TODO: close all other frames (viewer, editor)
        spin->stop();
        StartStop->SetLabel(wxT("Start"));
    }
}



void wxSpinMain::OnClose(wxCloseEvent& event)
{
    wxMessageDialog *dlg = new wxMessageDialog(this, wxT("Are you sure that you want to quit?"), wxT("Quit?"), wxOK|wxCANCEL|wxICON_ERROR|wxSTAY_ON_TOP);

    if ( dlg->ShowModal() == wxID_OK )
    {
        spin->stop();

        if (spinSettingsFrame) spinSettingsFrame->Destroy();
        //if (spinRenderer) spinRenderer->Destroy();
        //if (spinEditor) spinEditor->Destroy();

        this->Destroy();

    } else event.Veto();
}

void wxSpinMain::OnSpinModeChange(wxCommandEvent& event)
{

    std::cout << "Changed modes for SPIN. You must stop and restart before this takes effect." << std::endl;
}
