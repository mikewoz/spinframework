#include <iostream>
#include <wx/aboutdlg.h>
#include <wx/msgdlg.h>
#include "config.h"
#include "spinApp.h"
#include "SceneManager.h"
#include "MainFrame.h"


MainFrame::MainFrame( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : MainFrame_base( parent, id, title, pos, size, style )
{



}
MainFrame::~MainFrame()
{

}

void MainFrame::OnNewNode( wxCommandEvent& event )
{
    std::cout << "Got OnNewNode" << std::endl;
    event.Skip();
}

void MainFrame::OnAbout(wxCommandEvent& WXUNUSED(event))
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

void MainFrame::OnHelp(wxCommandEvent& WXUNUSED(event))
{
    bool success = wxLaunchDefaultBrowser(HELP_URL);
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
