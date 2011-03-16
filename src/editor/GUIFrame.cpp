///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Sep 12 2010)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "wx/wxprec.h"

#ifdef __BORLANDC__
#pragma hdrstop
#endif //__BORLANDC__

#ifndef WX_PRECOMP
#include <wx/wx.h>
#endif //WX_PRECOMP

#include "GUIFrame.h"

GuiFrame::GuiFrame(wxWindow *parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style) :
    wxFrame(parent, id, title, pos, size, style)
{
	this->SetSizeHints(wxDefaultSize, wxDefaultSize);
	
	mbar = new wxMenuBar(0);
	fileMenu = new wxMenu();
	wxMenuItem* menuFileQuit;
	menuFileQuit = new wxMenuItem(fileMenu, idMenuQuit, wxString(wxT("&Quit")) + wxT('\t') + wxT("Alt+F4"), wxT("Quit the application"), wxITEM_NORMAL);
	fileMenu->Append(menuFileQuit);
	
	mbar->Append(fileMenu, wxT("&File")); 
	
	helpMenu = new wxMenu();
	wxMenuItem* menuHelpAbout;
	menuHelpAbout = new wxMenuItem(helpMenu, idMenuAbout, wxString(wxT("&About")) + wxT('\t') + wxT("F1"), wxT("Show info about this application"), wxITEM_NORMAL);
	helpMenu->Append(menuHelpAbout);
	
	mbar->Append(helpMenu, wxT("&Help")); 
	
	this->SetMenuBar(mbar);
	
	statusBar = this->CreateStatusBar(2, wxST_SIZEGRIP, wxID_ANY);
	wxFlexGridSizer *fgSizer2;
	fgSizer2 = new wxFlexGridSizer(2, 1, 0, 0);
	fgSizer2->SetFlexibleDirection(wxBOTH);
	fgSizer2->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);
	
	m_treeCtrl2 = new wxTreeCtrl(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTR_DEFAULT_STYLE);
	fgSizer2->Add(m_treeCtrl2, 0, wxALL, 5);
	
	m_textCtrl1 = new wxTextCtrl(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
	fgSizer2->Add(m_textCtrl1, 0, wxALL, 5);
	
	this->SetSizer(fgSizer2);
	this->Layout();
	
	// Connect Events
	this->Connect(wxEVT_CLOSE_WINDOW, wxCloseEventHandler(GuiFrame::OnClose));
	this->Connect(menuFileQuit->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(GuiFrame::OnQuit));
	this->Connect(menuHelpAbout->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(GuiFrame::OnAbout));
}

GuiFrame::~GuiFrame()
{
	// Disconnect Events
	this->Disconnect(wxEVT_CLOSE_WINDOW, wxCloseEventHandler(GuiFrame::OnClose));
	this->Disconnect(idMenuQuit, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(GuiFrame::OnQuit));
	this->Disconnect(idMenuAbout, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(GuiFrame::OnAbout));
}
