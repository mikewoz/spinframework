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

#include "wxSpinEditor.h"

#include "spinContext.h"
#include <wx/choicdlg.h>

extern spinContext *spin;
extern pthread_mutex_t pthreadLock;
extern wxString resourcesPath;


//(*InternalHeaders(wxSpinEditor)
#include <wx/artprov.h>
#include <wx/bitmap.h>
#include <wx/intl.h>
#include <wx/image.h>
#include <wx/string.h>
//*)

//(*IdInit(wxSpinEditor)
const long wxSpinEditor::ID_SPIN_TREE = wxNewId();
const long wxSpinEditor::ID_treePanel = wxNewId();
const long wxSpinEditor::ID_CUSTOM1 = wxNewId();
const long wxSpinEditor::ID_PANEL2 = wxNewId();
const long wxSpinEditor::ID_SPLITTERWINDOW1 = wxNewId();
const long wxSpinEditor::spinEditor_clear = wxNewId();
const long wxSpinEditor::spinEditor_refresh = wxNewId();
const long wxSpinEditor::spinEditor_debugPrint = wxNewId();
const long wxSpinEditor::spinEditor_newNode = wxNewId();
const long wxSpinEditor::spinEditor_deleteNode = wxNewId();
const long wxSpinEditor::ID_TOOLBAR1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(wxSpinEditor,wxFrame)
	//(*EventTable(wxSpinEditor)
	//*)
END_EVENT_TABLE()

wxSpinEditor::wxSpinEditor(wxWindow* parent,wxWindowID id)
{
	//(*Initialize(wxSpinEditor)
	wxBoxSizer* BoxSizer1;
	wxStaticBoxSizer* StaticBoxSizer1;
	
	Create(parent, wxID_ANY, _("SPIN :: Editor"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("wxID_ANY"));
	SetClientSize(wxSize(-1,600));
	spinEditor_splitter = new wxSplitterWindow(this, ID_SPLITTERWINDOW1, wxDefaultPosition, wxDefaultSize, wxSP_3D, _T("ID_SPLITTERWINDOW1"));
	spinEditor_splitter->SetMinSize(wxSize(50,50));
	spinEditor_splitter->SetMinimumPaneSize(50);
	treePanel = new wxPanel(spinEditor_splitter, ID_treePanel, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_treePanel"));
	BoxSizer1 = new wxBoxSizer(wxVERTICAL);
	spinTree = new wxSpinTreeCtrl(treePanel,ID_SPIN_TREE,wxDefaultPosition,wxDefaultSize,wxTR_DEFAULT_STYLE,wxDefaultValidator,_T("ID_SPIN_TREE"));
	BoxSizer1->Add( , 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	treePanel->SetSizer(BoxSizer1);
	BoxSizer1->Fit(treePanel);
	BoxSizer1->SetSizeHints(treePanel);
	editorPanel = new wxPanel(spinEditor_splitter, ID_PANEL2, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL2"));
	StaticBoxSizer1 = new wxStaticBoxSizer(wxHORIZONTAL, editorPanel, _("Editor:"));
	SpinPropGrid = new wxSpinPropGrid(editorPanel,ID_CUSTOM1,wxDefaultPosition,wxDefaultSize);
	StaticBoxSizer1->Add(SpinPropGrid, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	editorPanel->SetSizer(StaticBoxSizer1);
	StaticBoxSizer1->Fit(editorPanel);
	StaticBoxSizer1->SetSizeHints(editorPanel);
	spinEditor_splitter->SplitHorizontally(treePanel, editorPanel);
	wxSpinEditor_ToolBar = new wxToolBar(this, ID_TOOLBAR1, wxDefaultPosition, wxDefaultSize, wxTB_HORIZONTAL|wxNO_BORDER, _T("ID_TOOLBAR1"));
	wxSpinEditor_ToolBar->SetToolBitmapSize(wxSize(24,24));
	ToolBarItem1 = wxSpinEditor_ToolBar->AddTool(spinEditor_clear, _("Clear"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_ERROR")),wxART_TOOLBAR), wxNullBitmap, wxITEM_NORMAL, _("Clear the current scene"), _("Clear the current scene"));
	ToolBarItem2 = wxSpinEditor_ToolBar->AddTool(spinEditor_refresh, _("Refresh"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_ERROR")),wxART_TOOLBAR), wxNullBitmap, wxITEM_NORMAL, _("Refresh the scene"), _("This will resync with the SPIN server, ensuring that the scene is up to date."));
	wxSpinEditor_ToolBar->AddSeparator();
	ToolBarItem3 = wxSpinEditor_ToolBar->AddTool(spinEditor_debugPrint, _("DebugPrint"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_ERROR")),wxART_TOOLBAR), wxNullBitmap, wxITEM_NORMAL, _("Debug Print to Console"), _("Debug Print to Console"));
	wxSpinEditor_ToolBar->AddSeparator();
	ToolBarItem4 = wxSpinEditor_ToolBar->AddTool(spinEditor_newNode, _("New Node"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_NEW")),wxART_BUTTON), wxNullBitmap, wxITEM_NORMAL, _("Create a new node"), _("This will allow you to create a new node (a dialog will let you choose the type)"));
	ToolBarItem5 = wxSpinEditor_ToolBar->AddTool(spinEditor_deleteNode, _("Delete Selected Node"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_DELETE")),wxART_BUTTON), wxNullBitmap, wxITEM_NORMAL, _("Delete the currently selected node"), _("Delete the currently selected node"));
	wxSpinEditor_ToolBar->Realize();
	SetToolBar(wxSpinEditor_ToolBar);
	
	Connect(spinEditor_clear,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&wxSpinEditor::OnClear);
	Connect(spinEditor_refresh,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&wxSpinEditor::OnRefresh);
	Connect(spinEditor_debugPrint,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&wxSpinEditor::OnDebugPrint);
	Connect(spinEditor_newNode,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&wxSpinEditor::OnNewNode);
	Connect(spinEditor_deleteNode,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&wxSpinEditor::OnDeleteNode);
	//*)

    // sash position doesn't seem to work in wxSmith, so do it manually:
    spinEditor_splitter->SetSashPosition(150);

	wxImage icon_eraser(resourcesPath + _T("/icon_eraser.gif"));
	icon_eraser.Rescale(24,24);
    wxSpinEditor_ToolBar->SetToolNormalBitmap( spinEditor_clear, wxBitmap(icon_eraser) );

	wxImage icon_refresh(resourcesPath + _T("/icon_refresh.gif"));
	icon_refresh.Rescale(24,24);
    wxSpinEditor_ToolBar->SetToolNormalBitmap( spinEditor_refresh, wxBitmap(icon_refresh) );

	wxImage icon_question_text(resourcesPath + _T("/icon_question_text.gif"));
	icon_question_text.Rescale(24,24);
    wxSpinEditor_ToolBar->SetToolNormalBitmap( spinEditor_debugPrint, wxBitmap(icon_question_text) );

	wxSpinEditor_ToolBar->Realize();


    // set smaller font
    wxFont pFont = GetFont();
    pFont.SetPointSize(9.0f);
    spinTree->SetFont(pFont);

	spinTree->BuildTree(spin->sceneManager->worldNode.get());

	// provide SpinTree with a pointer to SpinPropGrid so that it can fill it:
	spinTree->SetPropGrid(SpinPropGrid);


    // We'll set up an OSC receiver to listen to messages so that we can update
    // spinTree and spinPropGrid. Note that we MUST listen to the broadcast
    // channel and not to spinChannel directly. This is because SPIN might
    // broadcast different messages than it receives (eg, it receives a 'move'
    // message, but will transmit a 'setTranslation' message).
    if (spin->sceneManager->isSlave())
    {
        // if this sceneManager is already a listener, then we can just hijack
        // the rxServ and add our own method callbacks.
        listeningServer = spin->sceneManager->rxServ;
    }
    else
    {
        // if this sceneManager is a server, then we have to create a new server
        // that listens to the messages it is broadcasting
        if (isMulticastAddress(spin->txAddr))
            listeningServer = lo_server_thread_new_multicast(spin->txAddr.c_str(), spin->txPort.c_str(), oscParser_error);
        else
            listeningServer = lo_server_thread_new(spin->txPort.c_str(), oscParser_error);
        lo_server_thread_start(listeningServer);
    }

    spinTree->setListeningServer(this->listeningServer);
    SpinPropGrid->setListeningServer(this->listeningServer);

}

wxSpinEditor::~wxSpinEditor()
{
	//(*Destroy(wxSpinEditor)
	//*)
}

void wxSpinEditor::OnNewNode(wxCommandEvent& event)
{
    bool done = false;
    wxString nodeID, nodeType;
    wxArrayString allTypes;

    osg::ref_ptr<ReferencedNode> n;



    std::vector<std::string>::iterator it;
    for ( it=spin->sceneManager->nodeTypes.begin(); it!=spin->sceneManager->nodeTypes.end(); it++)
	{
        allTypes.Add(wxString( (*it).c_str(), wxConvUTF8 ));
	}

    nodeType = wxGetSingleChoice( wxT("Select node type:"), wxT("New Node"), allTypes);
    if (nodeType.IsEmpty()) return;


    while (!done)
    {
        nodeID = wxGetTextFromUser( wxT("Provide a node ID:"), wxT("New Node"), nodeID);

        if (nodeID.IsEmpty()) return;

        if ( spin->sceneManager->getNode(std::string(nodeID.mb_str())) )
        {
            wxMessageBox(wxT("That node ID already exists. Please try another."), wxT("Oops"), wxOK|wxICON_ERROR);
        }

        else
        {
        	/*
            lo_message msg = lo_message_new();
            lo_message_add(msg, "sss", "createNode", (const char*)nodeID.mb_str(), (const char*)nodeType.mb_str());
            spin->sendSceneMessage(msg);
            */
        	spin->sendSceneMessage("sss", "createNode", (const char*)nodeID.mb_str(), (const char*)nodeType.mb_str(), LO_ARGS_END);

            done = true;

        }

    }

}

void wxSpinEditor::OnRefresh(wxCommandEvent& event)
{
	/*
    lo_message msg = lo_message_new();
    lo_message_add_string(msg, "refresh");
    spin->sceneMessage(msg);
    */
	spin->sendSceneMessage("s", "refresh", LO_ARGS_END);
}

void wxSpinEditor::OnDebugPrint(wxCommandEvent& event)
{
	/*
    lo_message msg = lo_message_new();
    lo_message_add_string(msg, "debug");
    spin->sceneMessage(msg);
    */

    // local print:
    spin->sceneManager->debug();

    // remote print:
    spin->sendSceneMessage("s", "debug", LO_ARGS_END);
}

void wxSpinEditor::OnClear(wxCommandEvent& event)
{
    wxMessageDialog *dlg = new wxMessageDialog(this, wxT("Are you sure that you want to clear the scene?"), wxT("Clear?"), wxOK|wxCANCEL|wxICON_ERROR|wxSTAY_ON_TOP);

    if ( dlg->ShowModal() == wxID_OK )
    {
    	/*
        lo_message msg = lo_message_new();
        lo_message_add_string(msg, "clear");
        spin->sceneMessage(msg);
        */
        spin->sendSceneMessage("s", "clear", LO_ARGS_END);

    }
}

void wxSpinEditor::OnDeleteNode(wxCommandEvent& event)
{
    ReferencedNode *n = spinTree->GetSelectedNode();
    if (n)
    {
        wxMessageDialog *dlg = new wxMessageDialog(this, wxT("Are you sure that you want to delete node '") + wxString(n->id->s_name, wxConvUTF8) + wxT("'?"), wxT("Delete Node?"), wxOK|wxCANCEL|wxICON_ERROR|wxSTAY_ON_TOP);
        if ( dlg->ShowModal() == wxID_OK )
        {
        	/*
            lo_message msg = lo_message_new();
            lo_message_add(msg, "ss", "deleteNode", (const char*)n->id->s_name);
            spin->sceneMessage(msg);
            */
            spin->sendSceneMessage("ss", "deleteNode", (const char*)n->id->s_name, LO_ARGS_END);
        }
    } else {
        wxMessageDialog *dlg = new wxMessageDialog(this, wxT("You must select a node from the tree first"), wxT("Delete Node?"), wxOK|wxICON_ERROR|wxSTAY_ON_TOP);
    }
}
