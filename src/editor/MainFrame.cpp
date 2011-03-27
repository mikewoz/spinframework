#include <iostream>
#include <wx/aboutdlg.h>
#include <wx/msgdlg.h>
#include "config.h"
#include "spinApp.h"
#include "SceneManager.h"
#include "MainFrame.h"
#include "dialog_NewNode.h"
#include "wxSpinTreeCtrl.h"

using namespace spineditor;

MainFrame::MainFrame( wxWindow* parent) : MainFrame_base( parent) {}
MainFrame::~MainFrame() {}


void MainFrame::OnAbout(wxCommandEvent& WXUNUSED(event))
{
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

void MainFrame::OnNewNode( wxCommandEvent& WXUNUSED(event) )
{
    // creating nodes is handled in a dialog:
    dialog_NewNode* d = new dialog_NewNode( this);
    if ( d->ShowModal() == wxID_OK )
    {
        // how do we get the created node id from the dialog? eg, so that we can
        // select it in the tree automatically?
    }
}

void MainFrame::OnDeleteNode( wxCommandEvent& WXUNUSED(event) )
{
    // get currently selected node in the tree and tell the server to delete it:

    ReferencedNode* n = spinTreeCtrl->GetSelectedNode();

    if (n)
    {
        // TODO: need confirmation dialog here
        int answer = wxMessageBox("Delete node "+n->getID()+"?", "Delete Node",
                                  wxYES | wxCANCEL, this);
        if (answer == wxYES)
        {
            std::cout << "Attempting to delete node: " << n->getID() << std::endl;
            spinApp::Instance().SceneMessage("ss", "deleteNode", n->getID().c_str(), LO_ARGS_END);
        }
    }
}

void MainFrame::OnRefresh( wxCommandEvent& WXUNUSED(event) )
{
    spinApp::Instance().SceneMessage("s", "refresh", LO_ARGS_END);
}

void MainFrame::OnToggleGrid( wxCommandEvent& WXUNUSED(event) )
{
    // TODO
}

void MainFrame::OnToggleViewer( wxCommandEvent& WXUNUSED(event) )
{
    // TODO
}

void MainFrame::OnSceneDebug( wxCommandEvent& WXUNUSED(event) )
{
    spinApp::Instance().SceneMessage("s", "debug", LO_ARGS_END);
}
