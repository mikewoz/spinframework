#include <iostream>
#include <wx/aboutdlg.h>
#include <wx/msgdlg.h>
#include "config.h"
#include "spinApp.h"
#include "SceneManager.h"
#include "spinBaseContext.h"
#include "MainFrame.h"
#include "dialog_NewNode.h"
#include "wxSpinTreeCtrl.h"

using namespace spineditor;

MainFrame::MainFrame( wxWindow* parent) : MainFrame_base( parent)
{
    // some OSX specific things (eg, bind the special apple about menu)
#ifdef __WXMAC__
    wxApp::s_macAboutMenuItemId = wxID_ABOUT;
#endif

    // set the target for log messages to the text ctrl
    wxLog::SetActiveTarget(new wxLogTextCtrl(logTextCtrl));

    // redirect std::cout to the logTextCtrl (but make sure to save a pointer to
    // the old streambuf object so we can direct it back to std::cout). This
    // means that we don't have to use wxLogMessage!
#ifndef __WXMAC__
#if wxUSE_STD_IOSTREAM
    redirector = new wxStreamToTextRedirector(logTextCtrl);
#else
    std::cout << "Oops. The log window is not yet supported on this platform...\nAll messages will be displayed in the console instead." << std::endl;
#endif
#endif
    // start spinListener:
    // can't do this here, because wxSpinTreeCtrl needs it to be running already
    // when the constructor is called
    //wxGetApp().start();


    // ask for refresh:
    spinApp::Instance().SceneMessage("s", "refresh", LO_ARGS_END);
}


MainFrame::~MainFrame()
{
#ifndef __WXMAC__
#if wxUSE_STD_IOSTREAM
    delete redirector;
#endif
#endif
}

void MainFrame::OnClose(wxCloseEvent& event)
{
    std::cout << "Got MainFrame::OnClose" << std::endl;

    if ( event.CanVeto() )
    {
        if ( wxMessageBox("Are you sure you want to Quit?", "Quit", wxICON_QUESTION | wxYES_NO) != wxYES )
        {
            event.Veto();
            return;
        }
    }


    // destroy the window. Note: we could also do event.Skip() since the default
    // event handler does call Destroy(), too

    //this->Destroy();
    event.Skip();
}

void MainFrame::OnQuit(wxCommandEvent& WXUNUSED(event))
{
    // called when user selects Quit from File menu
    std::cout << "Got MainFrame::OnQuit" << std::endl;

    // tell the window to close (with force=TRUE, which will not allow us to
    // confirm with a message box, which is appropriate if they chose Quit from
    // the menu)
    Close(true); // this calls the OnClose handler
}



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
        int answer = wxMessageBox("Delete node "+n->getID()+"?", "Delete Node",
                                  wxYES | wxCANCEL, this);
        if (answer == wxYES)
        {
            std::cout << "Attempting to delete node: " << n->getID() << std::endl;
            spinApp::Instance().SceneMessage("ss", "deleteNode", n->getID().c_str(), LO_ARGS_END);
        }
    }
}

void MainFrame::OnRefreshScene( wxCommandEvent& WXUNUSED(event) )
{
    spinApp::Instance().SceneMessage("s", "refresh", LO_ARGS_END);
}

void MainFrame::OnClearScene( wxCommandEvent& WXUNUSED(event) )
{
    // TODO: need confirmation dialog here
    int answer = wxMessageBox("You are about to clear the entire scene. Continue?", "Clear Scene",
                              wxYES | wxCANCEL, this);
    if (answer == wxYES)
    {
        std::cout << "Attempting to clear scene" << std::endl;
        spinApp::Instance().SceneMessage("s", "clear", LO_ARGS_END);
    }
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
