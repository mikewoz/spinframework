


/*
 * ORIGINAL: wxOsgTreeCtrl.cpp :: part of Orihalcon Framework Library.
 *
 *   Copyright (C) 2005 by Toshiyuki Takahei <takahei@orihalcon.jp>
 *
 *   All rights reserved.
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License (LGPL) as
 * published by the Free Software Foundation; either version 2.1 of the
 * License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 * License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA or go to
 * http://www.gnu.org/copyleft/lesser.txt
 *
 */


#include "wxVessTreeCtrl.h"
#include "wxVessTreeVisitor.h"


#include "wxVessEditor.h"

#include "images/tree_object.xpm"
#include "images/tree_geode.xpm"
#include "images/tree_group.xpm"
#include "images/tree_node.xpm"
#include "images/tree_drawable.xpm"
#include "images/tree_stateset.xpm"
#include "images/tree_stateatt.xpm"

#include "vessThreads.h"
extern vessMaster *vess;
extern pthread_mutex_t pthreadLock;


BEGIN_EVENT_TABLE(wxVessTreeCtrl, wxTreeCtrl)
    //EVT_TREE_SEL_CHANGED(wxVessEditor::publicVessTree, wxVessEditor::OnVessSelectionChange)
	//EVT_TREE_BEGIN_DRAG(wxVessEditor::publicVessTree, wxVessEditor::OnDragBegin)
	//EVT_TREE_END_DRAG(wxVessEditor::publicVessTree, wxVessEditor::OnDragEnd)
	//EVT_TREE_ITEM_EXPANDING(O_TREE, wxVessTreeCtrl::OnItemExpanding)
	//EVT_TREE_ITEM_COLLAPSED(O_TREE, wxVessTreeCtrl::OnItemCollapsed)
	//EVT_TREE_ITEM_ACTIVATED(O_TREE, wxVessTreeCtrl::OnItemActivated)
END_EVENT_TABLE()



wxVessTreeCtrl::wxVessTreeCtrl(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size,
    long style, const wxValidator& validator, const wxString& name) : wxTreeCtrl(parent, id, pos, size, style, validator, name)
{
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


    //Connect(id,wxEVT_COMMAND_TREE_SEL_CHANGED,(wxObjectEventFunction)&wxVessEditor::OnVessSelectionChange);
    Connect(id,wxEVT_COMMAND_TREE_SEL_CHANGED,(wxObjectEventFunction)&wxVessTreeCtrl::OnVessSelectionChange);

    // Add an OSC callback that listens to scene messages (add/delete) so that
    // we can dynamically update the tree based on OSC messages:
    if (vess->sceneManager->rxServ)
    {
        std::string oscPattern = "/vess/" + vess->id;
        lo_server_thread_add_method(vess->sceneManager->rxServ, oscPattern.c_str(), NULL, wxVessTreeCtrl_liblo_callback, (void*)this);
    }

    // create and store and instance of wxVessTreeVisitor:
    m_pSceneTreeVisitor = new wxVessTreeVisitor(this);
}

wxVessTreeCtrl::~wxVessTreeCtrl()
{
}

void wxVessTreeCtrl::BuildTree(osg::Node* pRoot)
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

void wxVessTreeCtrl::Refresh()
{
    // TODO: more efficient method that compares existing tree to VESS, and adds
    // or removes nodes accordingly while keeping the existing ones

    BuildTree(vess->sceneManager->worldNode.get());
}

void wxVessTreeCtrl::addToTree(asReferenced *n, wxTreeItemId parentID)
{
    Freeze();
    std::string strLabel = n->nodeType + " : " + n->id->s_name;

    wxVessTreeItemData *treeData = new wxVessTreeItemData;
    treeData->m_pNode = n;

    wxTreeItemId id = AppendItem(parentID, wxString(strLabel.c_str(),wxConvUTF8), -1, -1, treeData);
    UpdateTreeItemIcon(id);
    Thaw();
    ExpandAll();
}

void wxVessTreeCtrl::addNode(const char *id, const char *type)
{
    // note that a createNode message was broadcast from vess AFTER the node was
    // instantiated, so we should now be able to find it in the sceneManager,
    // and so we'll create a tree item (if it doesn't already exist).
    asReferenced *n = vess->sceneManager->getNode(id, type);
    if (!n) return;

    wxTreeItemId nodeInTree = GetTreeItem(n, GetRootItem());
    if (!nodeInTree)
    {
        asReferenced *parentNode = vess->sceneManager->getNode(n->getParent(), type);
        wxTreeItemId parentInTree = GetTreeItem(n, GetRootItem());
        if (parentInTree) addToTree(n,parentInTree);
        else addToTree(n,GetRootItem());
    }

}



void wxVessTreeCtrl::removeNode(const char *id)
{

}

/*
void wxVessTreeCtrl::SetVisitor(wxVessTreeVisitor* pVisitor)
{
    if (!pVisitor) return;
    m_pSceneTreeVisitor = pVisitor;
}

wxVessTreeVisitor* wxVessTreeCtrl::GetVisitor()
{
    return m_pSceneTreeVisitor.get();
}

const wxVessTreeVisitor* wxVessTreeCtrl::GetVisitor() const
{
    return m_pSceneTreeVisitor.get();
}
*/

void wxVessTreeCtrl::SetPropGrid(wxVessPropGrid *PG)
{
    if (!PG) return;
    VessPropGrid = PG;
}

bool wxVessTreeCtrl::SelectNode(asReferenced* pNode)
{
    if (pNode == GetSelectedNode()) return true;

    if (GetCount() == 0) return false;

    wxTreeItemId id = GetTreeItem(pNode, GetRootItem());
    if (id)
    {
        SelectItem(id, true);
        return true;
    }

    return false;

    /*

    // if the root is selected
    wxVessTreeItemData *itemData = (wxVessTreeItemData*)GetItemData(GetRootItem());
    if (itemData)
    {
        if (itemData->m_pNode == pNode) {
            SelectItem(GetRootItem(), true);
            //** if the object found, jump out from this recursive loop!
            return true;
        }
    }

    if (pNode)
        return SelectNode(pNode, GetRootItem());

    return false;

    */
}

/*
bool wxVessTreeCtrl::SelectNode(asReferenced* pNode, wxTreeItemId idParent, wxTreeItemIdValue cookie)
{

    wxTreeItemId id;

    if (!cookie)
        id = GetFirstChild(idParent, cookie);
    else
        id = GetNextChild(idParent, cookie);

    if (!id.IsOk())
        return false;

    wxVessTreeItemData *pNode = (wxVessTreeItemData*)GetItemData(id);
    if (pNode)
    {
        if (pNode->m_pNode == pNode) {
            SelectItem(id, true);
            return true;
        }
    }

    if (ItemHasChildren(id))
        if (SelectNode(pNode, id))
            return true;

    return SelectNode(pNode, idParent, cookie);
}
*/

wxTreeItemId wxVessTreeCtrl::GetTreeItem(asReferenced* pNode, wxTreeItemId idParent, wxTreeItemIdValue cookie)
{
    if (!idParent.IsOk())
        return NULL;

    wxVessTreeItemData *treeData = (wxVessTreeItemData*)GetItemData(idParent);
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

}

asReferenced* wxVessTreeCtrl::GetSelectedNode() const
{
   if (!GetSelection())
        return NULL;

    wxVessTreeItemData *treeData = (wxVessTreeItemData*)GetItemData(GetSelection());
    if (!treeData)
        return NULL;

    return treeData->m_pNode.get();
}

asReferenced* wxVessTreeCtrl::GetNode(const wxTreeItemId& item) const
{
    wxVessTreeItemData *treeData = (wxVessTreeItemData*)GetItemData(item);
    if (!treeData)
        return NULL;

    return treeData->m_pNode.get();
}

/*
wxTreeItemId wxVessTreeCtrl::AppendOSGItem(wxTreeItemId parentId, const wxString& name, osg::Referenced* pNode)
{
    wxVessTreeItemData* pNode = new wxVessTreeItemData;
    pNode->m_pNode = pNode;
    wxTreeItemId id = AppendItem(parentId, name, -1, -1, pNode);
    return id;
}
*/

void wxVessTreeCtrl::UpdateTreeItemIcon(wxTreeItemId id)
{
    wxVessTreeItemData *treeData = (wxVessTreeItemData*)GetItemData(id);
    if (!treeData)
        return;

    if (treeData->m_pNode->nodeType=="asBasicNode")
        SetItemImage(id, 1, wxTreeItemIcon_Normal);
    else if (treeData->m_pNode->nodeType=="asShape")
        SetItemImage(id, wxTreeItemIcon_Normal);
    else
        SetItemImage(id, 0, wxTreeItemIcon_Normal);
}



void wxVessTreeCtrl::OnVessSelectionChange(wxTreeEvent &event)
{
    asReferenced *n = GetSelectedNode();
    if (n)
    {
        if (VessPropGrid) VessPropGrid->SetNode(n);
        else std::cout << "wxVessTreeCtrl: Oops. VessPropGrid does not exist. Cannot populate the property editor." << std::endl;
    } else {
        std::cout << "wxVessTreeCtrl: Selected node could not be cast as asReferenced ?!" << std::endl;
    }

    //event.Skip();
}


int wxVessTreeCtrl_liblo_callback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data)
{
    // make sure there is at least one argument (ie, a method to call):
	if (!argc) return 0;

    wxVessTreeCtrl *treeCtrl = (wxVessTreeCtrl*) user_data;

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
		treeCtrl->addNode((char*)argv[1], (char*)argv[2]);
	else if ((theMethod=="deleteNode") && (argc==2))
		treeCtrl->removeNode((char*)argv[1]);

	return 1;
}
