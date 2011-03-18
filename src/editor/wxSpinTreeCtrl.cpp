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
//
//  NOTE: This file is based on source code from the Orihalcon Framework Library
//  Copyright (C) 2005 by Toshiyuki Takahei <takahei@orihalcon.jp>
//  (Released under the GNU Lesser General Public License)
//
// -----------------------------------------------------------------------------


#include "wxSpinTreeCtrl.h"
#include "wxSpinTreeVisitor.h"

#include "spinApp.h"
#include "SceneManager.h"
#include "spinBaseContext.h"


BEGIN_EVENT_TABLE(wxSpinTreeCtrl, wxTreeCtrl)
    //EVT_TREE_SEL_CHANGED(wxSpinEditor::publicSpinTree, wxSpinEditor::OnSpinSelectionChange)
	//EVT_TREE_BEGIN_DRAG(wxSpinEditor::publicSpinTree, wxSpinEditor::OnDragBegin)
	//EVT_TREE_END_DRAG(wxSpinEditor::publicSpinTree, wxSpinEditor::OnDragEnd)
	//EVT_TREE_ITEM_EXPANDING(O_TREE, wxSpinTreeCtrl::OnItemExpanding)
	//EVT_TREE_ITEM_COLLAPSED(O_TREE, wxSpinTreeCtrl::OnItemCollapsed)
	//EVT_TREE_ITEM_ACTIVATED(O_TREE, wxSpinTreeCtrl::OnItemActivated)
END_EVENT_TABLE()



wxSpinTreeCtrl::wxSpinTreeCtrl(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size,
    long style, const wxValidator& validator, const wxString& name) : wxTreeCtrl(parent, id, pos, size, style, validator, name)
{
	/*
    int iconSize = 15;
    m_pImages = new wxImageList(iconSize, iconSize, true);

    m_pImages->Add(wxBitmap(tree_object_xpm));
    m_pImages->Add(wxBitmap(tree_geode_xpm));
    m_pImages->Add(wxBitmap(tree_group_xpm));
    m_pImages->Add(wxBitmap(tree_node_xpm));
    m_pImages->Add(wxBitmap(tree_drawable_xpm));
    m_pImages->Add(wxBitmap(tree_stateset_xpm));
    m_pImages->Add(wxBitmap(tree_stateatt_xpm));

    AssignImageList(m_pImages);
    */


    Connect(id,wxEVT_COMMAND_TREE_SEL_CHANGED,(wxObjectEventFunction)&wxSpinTreeCtrl::OnSpinSelectionChange);
    Connect(id,wxEVT_COMMAND_TREE_BEGIN_DRAG,(wxObjectEventFunction)&wxSpinTreeCtrl::OnSpinTreeDragBegin);
    Connect(id,wxEVT_COMMAND_TREE_END_DRAG,  (wxObjectEventFunction)&wxSpinTreeCtrl::OnSpinTreeDragEnd);

/*
    Connect(id, wxEVT_COMMAND_TREE_SEL_CHANGED, (wxObjectEventFunction)&wxSpinTreeCtrl::OnSpinSelectionChange);
    Connect(id, wxEVT_COMMAND_TREE_BEGIN_DRAG,  (wxObjectEventFunction)&wxSpinEditor::OnSpinTreeDragBegin);
    Connect(id, wxEVT_COMMAND_TREE_END_DRAG,    (wxObjectEventFunction)&wxSpinEditor::OnSpinTreeDragEnd);
*/

    // create and store and instance of wxSpinTreeVisitor:
    m_pSceneTreeVisitor = new wxSpinTreeVisitor(this);

    // add our liblo callback to all listener sockets:
    spinBaseContext *listener = spinApp::Instance().getContext();
    std::vector<lo_server>::iterator servIter;
    for (servIter = listener->lo_rxServs_.begin(); servIter != listener->lo_rxServs_.end(); ++servIter)
    {
        lo_server_add_method((*servIter),
            std::string("/SPIN/" + spinApp::Instance().getSceneID()).c_str(),
            NULL, wxSpinTreeCtrl_liblo_callback, this);
    }

}

wxSpinTreeCtrl::~wxSpinTreeCtrl()
{
}

void wxSpinTreeCtrl::BuildTree(osg::Node* pRoot)
{
    Freeze();
    DeleteAllItems();

    if (pRoot)
    {
        wxTreeItemId rootID = AddRoot(wxT("scene"));

        m_pSceneTreeVisitor->SetParentTreeItem(&rootID);
        //m_pSceneTreeVisitor->SetParentTreeItem(NULL);
        pRoot->accept(*m_pSceneTreeVisitor.get());
    }

    Thaw();

    ExpandAll();

}

void wxSpinTreeCtrl::Refresh()
{
    // TODO: more efficient method that compares existing tree to SPIN's scene
	// graph, and adds or removes nodes accordingly while keeping the existing
	// ones

    BuildTree(spinApp::Instance().sceneManager->worldNode.get());
}

void wxSpinTreeCtrl::addToTree(ReferencedNode *n, wxTreeItemId parentID)
{
    Freeze();
    std::string strLabel = n->nodeType + " : " + n->id->s_name;

    wxSpinTreeItemData *treeData = new wxSpinTreeItemData;
    treeData->m_pNode = n;

    wxTreeItemId id = AppendItem(parentID, wxString(strLabel.c_str(),wxConvUTF8), -1, -1, treeData);
    UpdateTreeItemIcon(id);
    Thaw();
    ExpandAll();
}

void wxSpinTreeCtrl::addNode(const char *id, const char *type)
{
    // note that a createNode message was broadcast from SPIN AFTER the node was
    // instantiated, so we should now be able to find it in the sceneManager,
    // and so we'll create a tree item (if it doesn't already exist).
    ReferencedNode *n = spinApp::Instance().sceneManager->getNode(id, type);
    if (!n) return;

    wxTreeItemId nodeInTree = GetTreeItem(n, GetRootItem());
    if (!nodeInTree)
    {
        ReferencedNode *parentNode = spinApp::Instance().sceneManager->getNode(n->getParent(), type);
        wxTreeItemId parentInTree = GetTreeItem(n, GetRootItem());
        if (parentInTree) addToTree(n,parentInTree);
        else addToTree(n,GetRootItem());
    }

}


void wxSpinTreeCtrl::removeNode(const char *id)
{
	// if the node to be removed is currently selected, then select NULL (root)
	if (strcmp(GetSelectedNode()->id->s_name,id)==0)
	{
		SelectNode(NULL);
	}

	// We need to find the node based on the string id provided:
	wxTreeItemId nodeInTree = GetTreeItem(id, GetRootItem());
	if (nodeInTree)
	{
		Freeze();
		Delete(nodeInTree);
		Thaw();
	}
}

bool wxSpinTreeCtrl::SelectNode(ReferencedNode* pNode)
{
    // there should always be at least one node (the scene root). If not, return
    // because this is a problem.
    if (GetCount() == 0) return false;

    // if pNode is NULL, we select the scene root
    if (!pNode)
    {
    	SelectItem(GetRootItem());
        UpdatePropGrid();
    }

    // if the node is already selected, don't do anything
    if (pNode == GetSelectedNode()) return true;

    wxTreeItemId id = GetTreeItem(pNode, GetRootItem());
    if (id)
    {
        SelectItem(id);
        UpdatePropGrid();
        return true;
    }

    // couldn't find the node, so return false
    return false;
}


wxTreeItemId wxSpinTreeCtrl::GetTreeItem(ReferencedNode* pNode, wxTreeItemId idParent, wxTreeItemIdValue cookie)
{
    return GetTreeItem(pNode->id->s_name, idParent, cookie);

    /*
    if (!idParent.IsOk())
        return NULL;

    std::cout << "lookging for node '" << pNode->id->s_name "' in tree ... " << std::endl;

    wxSpinTreeItemData *treeData = (wxSpinTreeItemData*)GetItemData(idParent);
    if (treeData)
    {
        if (treeData->m_pNode.get() == pNode)
            return idParent;
    }

    wxTreeItemId child;

    if (!cookie)
        child = GetFirstChild(idParent, cookie);
    else
        child = GetNextChild(idParent, cookie);

    if (!child.IsOk())
        return NULL;

    if (ItemHasChildren(child))
    {
        wxTreeItemId nextChild = GetTreeItem(pNode, child);
        if (nextChild) return nextChild;
    }
    return GetTreeItem(pNode, child, cookie);
*/
}


wxTreeItemId wxSpinTreeCtrl::GetTreeItem(const char *nodeId, wxTreeItemId idParent, wxTreeItemIdValue cookie)
{
    if (!idParent.IsOk()) return NULL;

    wxSpinTreeItemData *treeData = (wxSpinTreeItemData*)GetItemData(idParent);
    if (treeData)
    {
        if (strcmp(treeData->m_pNode->id->s_name,nodeId) == 0)
        {
            return idParent;
        }
    }

    if (ItemHasChildren(idParent))
    {
        wxTreeItemId child;
        for (child = GetFirstChild(idParent, cookie); child.IsOk(); child = GetNextChild(idParent, cookie))
        {
            wxTreeItemId targetItem = GetTreeItem(nodeId, child, cookie);
            if (targetItem.IsOk()) return targetItem;
        }
    }

    return GetTreeItem(nodeId, GetNextSibling(idParent), cookie);

}


ReferencedNode* wxSpinTreeCtrl::GetSelectedNode() const
{
   if (!GetSelection())
        return NULL;

    wxSpinTreeItemData *treeData = (wxSpinTreeItemData*)GetItemData(GetSelection());
    if (!treeData)
        return NULL;

    return treeData->m_pNode.get();
}


ReferencedNode* wxSpinTreeCtrl::GetNode(const wxTreeItemId& item) const
{
    wxSpinTreeItemData *treeData = (wxSpinTreeItemData*)GetItemData(item);
    if (!treeData)
        return NULL;

    return treeData->m_pNode.get();
}


void wxSpinTreeCtrl::UpdateTreeItemIcon(wxTreeItemId id)
{
    wxSpinTreeItemData *treeData = (wxSpinTreeItemData*)GetItemData(id);
    if (!treeData)
        return;

    if (treeData->m_pNode->nodeType=="GroupNode")
        SetItemImage(id, 1, wxTreeItemIcon_Normal);
    else if (treeData->m_pNode->nodeType=="ShapeNode")
        SetItemImage(id, wxTreeItemIcon_Normal);
    else
        SetItemImage(id, 0, wxTreeItemIcon_Normal);
}

// TODO:
/*
void wxSpinTreeCtrl::SetPropGrid(wxSpinPropGrid *PG)
{
    if (!PG) return;
    SpinPropGrid = PG;
}
*/
void wxSpinTreeCtrl::UpdatePropGrid()
{
    // TODO
    std::cout << "TODO: UpdatePropGrid()" << std::endl;

    /*
    if (!SpinPropGrid)
    {
        std::cout << "wxSpinTreeCtrl: Oops. SpinPropGrid does not exist. Cannot populate the property editor." << std::endl;
        return;
    }

    ReferencedNode *n = GetSelectedNode();
    if (n) SpinPropGrid->SetNode(n);
    else SpinPropGrid->SetNode(NULL); // This will empty the propgrid editor
     */
}



void wxSpinTreeCtrl::OnSpinSelectionChange(wxTreeEvent &event)
{
    ReferencedNode *n = GetSelectedNode();
    if (n)
    {
        std::cout << "got tree selection: " << n->getID() << std::endl;
    }
    else std::cout << "got tree selection, but couldn't find spin node" << std::endl;
    //UpdatePropGrid();
}

void wxSpinTreeCtrl::OnSpinTreeDragBegin(wxTreeEvent &event)
{
    draggedItem = event.GetItem();
	event.Allow();
}

void wxSpinTreeCtrl::OnSpinTreeDragEnd(wxTreeEvent &event)
{

    if (ReferencedNode *child = this->GetNode(draggedItem))
    {
        std::string parentString;

        // if dragged onto the root, we setParent to "world"
        if (event.GetItem() == GetRootItem())
        {
            parentString = "world";
        }

        // otherwise, we get the node that this was dropped on, and set the
        // parent to that symbol:
        else if (ReferencedNode *parent = this->GetNode(event.GetItem()))
        {
            parentString = parent->id->s_name;
        }

        if (!parentString.empty())
        {
        	/*
            lo_message msg = lo_message_new();
            lo_message_add(msg, "ss", "setParent", parentString.c_str());
            spin->nodeMessage(child->id, msg);
            */
            spinApp::Instance().NodeMessage(child->id->s_name, "ss", "setParent", parentString.c_str(), LO_ARGS_END);

        }
    }
}


int wxSpinTreeCtrl_liblo_callback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data)
{
    /*
    printf("wx got spin message: %s", path);
    for (int i=0; i<argc; i++) {
        printf(" ");
        lo_arg_pp((lo_type) types[i], argv[i]);
    }
    printf("\n");
    */

    // make sure there is at least one argument (ie, a method to call):
	if (!argc) return 0;

    wxSpinTreeCtrl *treeCtrl = (wxSpinTreeCtrl*) user_data;

    if (!treeCtrl) return 0;

	// get the method (argv[0]):
	std::string theMethod;
	if (lo_is_string_type((lo_type)types[0]))
	{
		theMethod = std::string((char *)argv[0]);
	}
	else return 0;


	// now look for messages that will affect the tree:
    if ((theMethod=="nodeList") && (argc>2))
	{
		for (int i=2; i<argc; i++)
		{
			if (strcmp((char*)argv[i],"NULL")!=0) treeCtrl->addNode((char*)argv[i], (char*)argv[1]);
		}
	}
	else if ((theMethod=="createNode") && (argc==3))
	{
		treeCtrl->addNode((char*)argv[1], (char*)argv[2]);
	}
	else if ((theMethod=="deleteNode") && (argc==2))
	{
		treeCtrl->removeNode((char*)argv[1]);
	}
    else if (theMethod=="refresh")
	{
	    treeCtrl->Refresh();
	}
	else if (theMethod=="clear")
	{
		treeCtrl->Refresh();
		treeCtrl->SelectNode(NULL);
	}

	return 1;
}
