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
//    La SociŽtŽ des Arts Technologiques (http://www.sat.qc.ca)
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

#include "wxVessEditor.h"

#include "vessThreads.h"
#include <wx/choicdlg.h>

extern vessMaster *vess;

extern pthread_mutex_t pthreadLock;


//(*InternalHeaders(wxVessEditor)
#include <wx/artprov.h>
#include <wx/bitmap.h>
#include <wx/intl.h>
#include <wx/image.h>
#include <wx/string.h>
//*)

//(*IdInit(wxVessEditor)
const long wxVessEditor::ID_VESS_TREE = wxNewId();
const long wxVessEditor::ID_treePanel = wxNewId();
const long wxVessEditor::ID_CUSTOM1 = wxNewId();
const long wxVessEditor::ID_PANEL2 = wxNewId();
const long wxVessEditor::ID_SPLITTERWINDOW1 = wxNewId();
const long wxVessEditor::vessEditor_newNode = wxNewId();
const long wxVessEditor::vessEditor_clear = wxNewId();
const long wxVessEditor::vessEditor_refresh = wxNewId();
const long wxVessEditor::vessEditor_debugPrint = wxNewId();
const long wxVessEditor::ID_TOOLBAR1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(wxVessEditor,wxFrame)
	//(*EventTable(wxVessEditor)
	//*)
END_EVENT_TABLE()

wxVessEditor::wxVessEditor(wxWindow* parent,wxWindowID id)
{
	//(*Initialize(wxVessEditor)
	wxBoxSizer* BoxSizer1;
	wxStaticBoxSizer* StaticBoxSizer1;
	
	Create(parent, wxID_ANY, _("SPIN :: Editor"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("wxID_ANY"));
	SetClientSize(wxSize(-1,600));
	vessEditor_splitter = new wxSplitterWindow(this, ID_SPLITTERWINDOW1, wxDefaultPosition, wxDefaultSize, wxSP_3D, _T("ID_SPLITTERWINDOW1"));
	vessEditor_splitter->SetMinSize(wxSize(50,50));
	vessEditor_splitter->SetMinimumPaneSize(50);
	treePanel = new wxPanel(vessEditor_splitter, ID_treePanel, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_treePanel"));
	BoxSizer1 = new wxBoxSizer(wxVERTICAL);
	vessTree = new wxVessTreeCtrl(treePanel,ID_VESS_TREE,wxDefaultPosition,wxDefaultSize,wxTR_DEFAULT_STYLE,wxDefaultValidator,_T("ID_VESS_TREE"));
	BoxSizer1->Add(vessTree, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	treePanel->SetSizer(BoxSizer1);
	BoxSizer1->Fit(treePanel);
	BoxSizer1->SetSizeHints(treePanel);
	editorPanel = new wxPanel(vessEditor_splitter, ID_PANEL2, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL2"));
	StaticBoxSizer1 = new wxStaticBoxSizer(wxHORIZONTAL, editorPanel, _("Editor:"));
	VessPropGrid = new wxVessPropGrid(editorPanel,ID_CUSTOM1,wxDefaultPosition,wxDefaultSize);
	StaticBoxSizer1->Add(VessPropGrid, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	editorPanel->SetSizer(StaticBoxSizer1);
	StaticBoxSizer1->Fit(editorPanel);
	StaticBoxSizer1->SetSizeHints(editorPanel);
	vessEditor_splitter->SplitHorizontally(treePanel, editorPanel);
	wxVessEditor_ToolBar = new wxToolBar(this, ID_TOOLBAR1, wxDefaultPosition, wxDefaultSize, wxTB_HORIZONTAL|wxNO_BORDER, _T("ID_TOOLBAR1"));
	ToolBarItem1 = wxVessEditor_ToolBar->AddTool(vessEditor_newNode, _("New Node"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_NEW")),wxART_BUTTON), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_NEW")),wxART_BUTTON), wxITEM_NORMAL, _("Create a new node"), _("This will allow you to create a new node (a dialog will let you choose the type)"));
	ToolBarItem2 = wxVessEditor_ToolBar->AddTool(vessEditor_clear, _("Clear"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_DELETE")),wxART_TOOLBAR), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_DELETE")),wxART_TOOLBAR), wxITEM_NORMAL, _("Clear the current scene"), _("Clear the current scene"));
	ToolBarItem3 = wxVessEditor_ToolBar->AddTool(vessEditor_refresh, _("Refresh"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_UNDO")),wxART_BUTTON), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_UNDO")),wxART_BUTTON), wxITEM_NORMAL, _("Refresh the scene"), _("This will resync with the VESS server, ensuring that the scene is up to date."));
	ToolBarItem4 = wxVessEditor_ToolBar->AddTool(vessEditor_debugPrint, _("DebugPrint"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_HELP_PAGE")),wxART_BUTTON), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_HELP_PAGE")),wxART_BUTTON), wxITEM_NORMAL, _("Debug Print to Console"), _("Debug Print to Console"));
	wxVessEditor_ToolBar->Realize();
	SetToolBar(wxVessEditor_ToolBar);
	
	Connect(vessEditor_newNode,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&wxVessEditor::OnNewNode);
	Connect(vessEditor_clear,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&wxVessEditor::OnClear);
	Connect(vessEditor_refresh,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&wxVessEditor::OnRefresh);
	Connect(vessEditor_debugPrint,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&wxVessEditor::OnDebugPrint);
	//*)

    // sash position doesn't seem to work in wxSmith, so do it manually:
    vessEditor_splitter->SetSashPosition(150);


    // set smaller font
    wxFont pFont = GetFont();
    pFont.SetPointSize(9.0f);
    vessTree->SetFont(pFont);

	//wxTreeItemId vessTreeRoot = vessTree->AddRoot(wxT("root"));

    //wxTreeItemId vessTreeRoot = osgTree->AddRoot(wxT("root"));

	vessTree->BuildTree(vess->sceneManager->worldNode.get());


	// provide VessTree with a pointer to VessPropGrid so that it can fill it:
	vessTree->SetPropGrid(VessPropGrid);

/*
    wxTreeItemId vessTreeNode;
	vessTreeNode = osgTree->AppendItem(osgTree, wxT("foo"));
	vessTreeNode = osgTree->AppendItem(osgTree, wxT("sheefa"));
	osgTree->Expand(vessTreeRoot);
*/

}

wxVessEditor::~wxVessEditor()
{
	//(*Destroy(wxVessEditor)
	//*)
}

void wxVessEditor::OnNewNode(wxCommandEvent& event)
{
    bool done = false;
    wxString nodeID, nodeType;
    wxArrayString allTypes;

    osg::ref_ptr<asReferenced> n;



    std::vector<std::string>::iterator it;
    for ( it=vess->sceneManager->nodeTypes.begin(); it!=vess->sceneManager->nodeTypes.end(); it++)
	{
        allTypes.Add(wxString( (*it).c_str(), wxConvUTF8 ));
	}

    nodeType = wxGetSingleChoice( wxT("Select node type:"), wxT("New Node"), allTypes);
    if (nodeType.IsEmpty()) return;


    while (!done)
    {
        nodeID = wxGetTextFromUser( wxT("Provide a node ID:"), wxT("New Node"), nodeID);

        if (nodeID.IsEmpty()) return;

        if ( vess->sceneManager->getNode(std::string(nodeID.mb_str())) )
        {
            wxMessageBox(wxT("That node ID already exists. Please try another."), wxT("Oops"), wxOK|wxICON_ERROR);
        }

        else
        {

            lo_message msg = lo_message_new();
            lo_message_add_string(msg, "createNode");
            lo_message_add_string(msg, nodeID.mb_str());
            lo_message_add_string(msg, nodeType.mb_str());
            std::string OSCpath = "/vess/" + vess->id;
            lo_send_message(vess->sceneManager->rxAddr, OSCpath.c_str(), msg);
            lo_message_free(msg);

            done = true;

            /*
            pthread_mutex_lock(&pthreadLock);
            n = vess->sceneManager->createNode(std::string(nodeID.mb_str()), std::string(nodeType.mb_str()));
            pthread_mutex_unlock(&pthreadLock);

            if (!n.valid())
            {
                wxMessageBox(wxT("Could not create node."), wxT("Oops"), wxOK|wxICON_ERROR);
            }
            else {
                // refresh tree control:
                vessTree->Refresh();
            }
           */

        }

    }

}

void wxVessEditor::OnRefresh(wxCommandEvent& event)
{
	vessTree->BuildTree(vess->sceneManager->worldNode.get());
}


void wxVessEditor::OnVessSelectionChange(wxTreeEvent &event)
{
    //vessTree->SelectItem(event.GetItem());

    std::cout << "OnVessSelectionChange: " << std::string(event.GetLabel().mb_str()) << std::endl;

    std::cout << "VessTree has " << vessTree->GetChildrenCount(vessTree->GetRootItem(), true) << " children" << std::endl;

    //wxTreeItemId id = vessTree->GetSelection();
    //std::cout << "getSelection=" << std::string( vessTree->GetItemText(id).mb_str() ) << std::endl;

    wxTreeItemId id = event.GetItem();
    std::cout << "IsOk item? " << id.IsOk() << std::endl;
    //std::cout << "itemText=" << std::string( vessTree->GetItemText(event.GetItem()).mb_str() ) << std::endl;

    //SceneNodeData* pNode = (SceneNodeData*) vessTree->GetItemData(event.GetItem());
    //if (!pNode) std::cout << "asReferenced id: " << pNode->m_pObject->id << std::endl;


/*
    asReferenced* n = vessTree->GetSelectedObject();
    if (n)
    {
        std::cout << "clicked on " << n->id << std::endl;
    }

    else std::cout << "couldn't cast tree click as asReferenced" << std::endl;
*/

    event.Skip();
}


void wxVessEditor::OnDragBegin(wxTreeEvent &event)
{
	//iDraggedItem = oEvent.GetItem();
	event.Allow();
}

void wxVessEditor::OnDragEnd(wxTreeEvent &event)
{
    // TODO: change parents
}

void wxVessEditor::OnDebugPrint(wxCommandEvent& event)
{
    vess->sceneManager->debug();
}

void wxVessEditor::OnClear(wxCommandEvent& event)
{
    vess->sceneManager->clear();
}
