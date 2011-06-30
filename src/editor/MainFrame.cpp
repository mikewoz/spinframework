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
    // register this class with spin to receiver INFO channel messages:
    spin::spinApp::Instance().getContext()->addInfoHandler(this);

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
    spin::spinApp::Instance().SceneMessage("s", "refresh", SPIN_ARGS_END);
}


MainFrame::~MainFrame()
{
#ifndef __WXMAC__
#if wxUSE_STD_IOSTREAM
    delete redirector;
#endif
#endif
}

void MainFrame::onInfoMessage(spin::InfoMessage *msg)
{
    /*
    std::cout << "Got info message:"
            << " scene=" << msg->sceneID
            << " server=" << msg->serverAddr
            << " (UDP: " << msg->serverUDPPort
            << " TCP: " << msg->serverTCPport
            << ") multicast=" << msg->multicastAddr
            << " (DATA: " << msg->multicastDataPort
            << " SYNC: " << msg->multicastSyncPort
            << ")" << std::endl;
            */
}

void MainFrame::onServerChange(std::vector<spin::InfoMessage*> serverList)
{
    // get the currently selected item:
    wxString prevServer = serverChooser->GetStringSelection();
    std::cout << "MainFrame got updated server list (prev=" << prevServer << "): ";

    //spin::InfoMessage *prevServerInfo;


    // replace choices with new list:
    Freeze();
    serverChooser->Clear();
    std::vector<spin::InfoMessage*>::iterator sIt;
    serverChooser->Append("<not-connected>");
    for (sIt=serverList.begin(); sIt!=serverList.end(); ++sIt)
    {
        serverChooser->Append((*sIt)->sceneID);
        std::cout << " " << (*sIt)->sceneID;
    }
    std::cout << std::endl;
    Thaw();

    // if previous server was <not-connected>, let's automatically connect
    if ((prevServer=="<not-connected>") && (serverChooser->GetCount()>1))
    {
        serverChooser->SetSelection(1);
    }

    // otherwise, select the same selection as we had previously, or the
    // <not-connected> option if the previous server is not online anymore:
    else
    {
        if (!serverChooser->SetStringSelection(prevServer))
            serverChooser->SetStringSelection("<not-connected>");
    }

    // SetSelection and SetStringSelection do not emit an event, so we simulate
    // an event here:
    //serverChooser->Command(wxEVT_COMMAND_CHOICE_SELECTED, this->GetId());
    //serverChooser->Command(wxEVT_COMMAND_CHOICE_SELECTED);

    wxCommandEvent event(wxEVT_COMMAND_CHOICE_SELECTED, this->GetId());
    event.SetEventObject(serverChooser);
    //event.SetText(serverChooser->GetStringSelection());
    GetEventHandler()->ProcessEvent( event );

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
    wxIcon SPINIcon(spin::spinApp::Instance().sceneManager->resourcesPath + wxT("/images/spin_48x48.png"), wxBITMAP_TYPE_PNG);
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

    spin::ReferencedNode* n = spinTreeCtrl->GetSelectedNode();

    if (n)
    {
        int answer = wxMessageBox("Delete node "+n->getID()+"?", "Delete Node",
                                  wxYES | wxCANCEL, this);
        if (answer == wxYES)
        {
            std::cout << "Attempting to delete node: " << n->getID() << std::endl;
            spin::spinApp::Instance().SceneMessage("ss", "deleteNode", n->getID().c_str(), SPIN_ARGS_END);
        }
    }
}

void MainFrame::OnRefreshScene( wxCommandEvent& WXUNUSED(event) )
{
    spin::spinApp::Instance().SceneMessage("s", "refresh", SPIN_ARGS_END);
}

void MainFrame::OnClearScene( wxCommandEvent& WXUNUSED(event) )
{
    // TODO: need confirmation dialog here
    int answer = wxMessageBox("You are about to clear the entire scene. Continue?", "Clear Scene",
                              wxYES | wxCANCEL, this);
    if (answer == wxYES)
    {
        std::cout << "Attempting to clear scene" << std::endl;
        spin::spinApp::Instance().SceneMessage("s", "clear", SPIN_ARGS_END);
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
    spin::spinApp::Instance().SceneMessage("s", "debug", SPIN_ARGS_END);
}

void MainFrame::OnServerChange( wxCommandEvent& event )
{
    // TODO
    std::cout << "SELECTED NEW SERVER: " << event.GetString() << std::endl;

    Freeze();
    spin::spinApp::Instance().getContext()->stop();
    spin::spinApp::Instance().setSceneID(event.GetString().ToStdString());
    spin::spinApp::Instance().getContext()->start();
    Thaw();

    std::cout << "Changed scene id to: " << spin::spinApp::Instance().getSceneID() << std::endl;

    // need to reconnect any ui components that have callbacks which listen to
    // SPIN messages:
    spinTreeCtrl->connectToSpin();

    event.Skip();
}
