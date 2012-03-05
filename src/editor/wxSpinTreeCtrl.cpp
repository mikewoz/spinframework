/*
 * This file is part of the SPIN Framework.
 *
 * Copyright (c) 2009 Mike Wozniewski
 * Copyright (c) 2009 Zack Settel
 * Copyright (c) 2011 Alexandre Quessy
 *
 * SPIN Framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * SPIN Framework is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with SPIN Framework. If not, see <http://www.gnu.org/licenses/>.
 */

#include "wxSpinTreeCtrl.h"
#include "wxSpinTreeVisitor.h"
#include "spinApp.h"
#include "SceneManager.h"
#include "spinBaseContext.h"
#include "spinUtil.h"

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

    this->connectToSpin();
}

wxSpinTreeCtrl::~wxSpinTreeCtrl()
{

    m_pSceneTreeVisitor = 0; // OSG should handle deletion
    //delete m_pSceneTreeVisitor;
}

void wxSpinTreeCtrl::connectToSpin()
{
    // add our liblo callback to all listener sockets:
    spin::spinBaseContext *listener = spin::spinApp::Instance().getContext();
    std::vector<lo_server>::iterator servIter;
    for (servIter = listener->lo_rxServs_.begin(); servIter != listener->lo_rxServs_.end(); ++servIter)
    {
        std::string oscPattern = std::string("/SPIN/" + spin::spinApp::Instance().getSceneID());

        // remove existing callback (if exists):
        lo_server_del_method((*servIter), oscPattern.c_str(), NULL);

        // add new callback:
        lo_server_add_method((*servIter), oscPattern.c_str(), NULL, wxSpinTreeCtrl_liblo_callback, this);

        // TODO: we should really have callbacks for node messages and scene
        // messages rather than parsing every single message received
        //lo_server_add_method((*servIter), NULL, NULL, wxSpinTreeCtrl_liblo_callback, this);
    }
}

void wxSpinTreeCtrl::BuildTree(osg::Node* pRoot)
{
    Freeze();
    DeleteAllItems();
    if (pRoot)
    {
        wxTreeItemId rootID = AddRoot(wxT("world"));

        m_pSceneTreeVisitor->SetParentTreeItem(&rootID);
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

    BuildTree(spin::spinApp::Instance().sceneManager->worldNode.get());
}

void wxSpinTreeCtrl::addToTree(spin::ReferencedNode *n)
{
    wxTreeItemId nodeInTree = GetTreeItem(n);
    if (nodeInTree)
    {
        // If node is already in the tree, we check to see if the parent has
        // changed. If it has, we remove it first
        wxTreeItemId parentTreeItem = GetTreeItem(n->getParent());
        if (parentTreeItem == GetItemParent(nodeInTree))
        {
            // the parent in the tree is already correct, so we don't need to do
            // anything
            std::cout << "Warning (wxSpinTreeCtrl::addToTree). Node " << n->getID() << " already exists in tree." << std::endl;
        }
        else
        {
            // The node in the tree has the wrong parent, so we need to first
            // remove the node from the tree, before we can add it to the proper
            // parent.
            Freeze();
            Delete(nodeInTree);
            Thaw();
        }
    }

    wxTreeItemId parentTreeItem = GetTreeItem(n->getParent());
    if (parentTreeItem)
        addToTree(n,parentTreeItem);
    else
        addToTree(n,GetRootItem());
}

void wxSpinTreeCtrl::addToTree(spin::ReferencedNode *n, wxTreeItemId parentID)
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

void wxSpinTreeCtrl::addNode(const char *id)
{
    spin::ReferencedNode *n = spin::spinApp::Instance().sceneManager->getNode(id);
    if (n) addToTree(n);
}

/*
void wxSpinTreeCtrl::addNode(const char *id, const char *type)
{
    // note that a createNode message was broadcast from SPIN AFTER the node was
    // instantiated, so we should now be able to find it in the sceneManager,
    // and so we'll create a tree item (if it doesn't already exist).
    spin::ReferencedNode *n = spin::spinApp::Instance().sceneManager->getNode(id, type);
    if (n) addToTree(n);
}
*/

void wxSpinTreeCtrl::removeNode(const char *id)
{
    // if the node to be removed is currently selected, then select NULL (root)
    spin::ReferencedNode* n = GetSelectedNode();
    if (n && strcmp(n->id->s_name,id)==0)
    {
        SelectNode(NULL);
    }
    // We need to find the node based on the string id provided:
    wxTreeItemId nodeInTree = GetTreeItem(id);
    if (nodeInTree)
    {
        Freeze();
        Delete(nodeInTree);
        Thaw();
    }
}

bool wxSpinTreeCtrl::SelectNode(spin::ReferencedNode* pNode)
{
    // there should always be at least one node (the scene root). If not, return
    // because this is a problem.
    if (GetCount() == 0)
        return false;

    // if pNode is NULL, we select the scene root
    if (! pNode)
    {
        SelectItem(GetRootItem());
        UpdatePropGrid();
    }

    // if the node is already selected, don't do anything
    if (pNode == GetSelectedNode())
        return true;

    wxTreeItemId id = GetTreeItem(pNode);
    if (id)
    {
        SelectItem(id);
        UpdatePropGrid();
        return true;
    }
    // couldn't find the node, so return false
    return false;
}

/*
wxTreeItemId wxSpinTreeCtrl::GetTreeItem(spin::ReferencedNode* pNode, wxTreeItemId idParent, wxTreeItemIdValue cookie)
{
    return GetTreeItem(pNode->id->s_name, idParent, cookie);
}
*/
wxTreeItemId wxSpinTreeCtrl::GetTreeItem(spin::ReferencedNode* pNode)
{
    return GetTreeItem(pNode->id->s_name);
}

wxTreeItemId wxSpinTreeCtrl::GetTreeItem(const char *nodeId)
{
    return GetTreeItem(nodeId, GetRootItem());
}

wxTreeItemId wxSpinTreeCtrl::GetTreeItem(const char *nodeId, wxTreeItemId idParent, wxTreeItemIdValue cookie)
{
    if (! idParent.IsOk())
        return NULL;

    wxSpinTreeItemData *treeData = (wxSpinTreeItemData*)GetItemData(idParent);
    if (treeData && treeData->m_pNode.valid())
    {
        if (strcmp(treeData->m_pNode->id->s_name, nodeId) == 0)
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
            if (targetItem.IsOk())
                return targetItem;
        }
    }
    return GetTreeItem(nodeId, GetNextSibling(idParent), cookie);
}

spin::ReferencedNode* wxSpinTreeCtrl::GetSelectedNode() const
{
   if (!GetSelection())
        return NULL;
    wxSpinTreeItemData *treeData = (wxSpinTreeItemData*) GetItemData(GetSelection());
    if (!treeData)
        return NULL;
    if (!treeData->m_pNode.valid())
        return NULL;

    return treeData->m_pNode.get();
}

spin::ReferencedNode* wxSpinTreeCtrl::GetNode(const wxTreeItemId& item) const
{
    wxSpinTreeItemData *treeData = (wxSpinTreeItemData*) GetItemData(item);
    if (!treeData)
        return NULL;

    if (!treeData->m_pNode.valid())
        return NULL;

    return treeData->m_pNode.get();
}

void wxSpinTreeCtrl::UpdateTreeItemIcon(wxTreeItemId id)
{
    wxSpinTreeItemData *treeData = (wxSpinTreeItemData*)GetItemData(id);
    if ((!treeData) || (!treeData->m_pNode.valid()))
        return;
    if (treeData->m_pNode->nodeType == "GroupNode")
        SetItemImage(id, 1, wxTreeItemIcon_Normal);
    else if (treeData->m_pNode->nodeType == "ShapeNode")
        SetItemImage(id, wxTreeItemIcon_Normal);
    else
        SetItemImage(id, 0, wxTreeItemIcon_Normal);
}

// TODO:
void wxSpinTreeCtrl::SetPropGrid(wxSpinPropGrid *pg)
{
    if (!pg) return;
    spinPropGrid = pg;
}

void wxSpinTreeCtrl::UpdatePropGrid()
{
    if (!spinPropGrid)
    {
        std::cout << "wxSpinTreeCtrl: Oops. SpinPropGrid does not exist. Cannot populate the property editor." << std::endl;
        return;
    }

    spin::ReferencedNode *n = GetSelectedNode();
    if (n) spinPropGrid->SetNode(n);
    else spinPropGrid->SetNode(NULL); // This will empty the propgrid editor
}

void wxSpinTreeCtrl::OnSpinSelectionChange(wxTreeEvent & WXUNUSED(event))
{
    spin::ReferencedNode *n = GetSelectedNode();
    if (n)
    {
        //std::cout << "got tree selection: " << n->getID() << std::endl;
    }
    else std::cout << "got tree selection, but couldn't find spin node" << std::endl;
    UpdatePropGrid();
}

void wxSpinTreeCtrl::OnSpinTreeDragBegin(wxTreeEvent &event)
{
    spin::ReferencedNode *spinNode = this->GetNode(event.GetItem());
    if (spinNode)
    {
        draggedItem = event.GetItem();
        event.Allow();
    }
}

void wxSpinTreeCtrl::OnSpinTreeDragEnd(wxTreeEvent &event)
{

    if (spin::ReferencedNode *child = this->GetNode(draggedItem))
    {
        std::string parentString;

        // if dragged onto the root, we setParent to "world"
        if (event.GetItem() == GetRootItem())
        {
            parentString = "world";
        }

        // otherwise, we get the node that this was dropped on, and set the
        // parent to that symbol:
        else if (spin::ReferencedNode *parent = this->GetNode(event.GetItem()))
        {
            parentString = parent->id->s_name;
        }

        if (! parentString.empty())
        {
            /*
            lo_message msg = lo_message_new();
            lo_message_add(msg, "ss", "setParent", parentString.c_str());
            spin->nodeMessage(child->id, msg);
            */
            spin::spinApp::Instance().NodeMessage(child->id->s_name, "ss", "setParent", parentString.c_str(), SPIN_ARGS_END);
        }
    }
}


int wxSpinTreeCtrl_liblo_callback(const char *path, const char *types, lo_arg **argv, int argc, void * WXUNUSED(data), void *user_data)
{
    // DEBUG PRINT:

    printf("wxSpinTreeCtrl got spin message: %s", path);
    for (int i=0; i<argc; i++) {
        printf(" ");
        lo_arg_pp((lo_type) types[i], argv[i]);
    }
    printf("\n");


    if (!argc)
    {
        std::cout << "ERROR: got message for " << path << " without any method or arguments" << std::endl;
        return 1;
    }

    // check that the treeCtrl was provided as user_data
    wxSpinTreeCtrl *treeCtrl = (wxSpinTreeCtrl*) user_data;
    if (! treeCtrl)
        return 1;


    // WARNING: this callback will match ANY path, so we must manually check
    // if it is within the SPIN namespace, and if it matches the sceneID:
    // TODO: replace this with node/scene callbacks!!!
    /*
    std::string spinToken, sceneString, nodeString;
    std::istringstream pathstream(path);
    pathstream.get(); // ignore leading slash
    getline(pathstream, spinToken, '/');
    getline(pathstream, sceneString, '/');
    getline(pathstream, nodeString, '/');

    if ((spinToken!="SPIN") || !spin::wildcardMatch(sceneString.c_str(), spin::spinApp::Instance().getSceneID().c_str()) )
    {
        std::cout << "Warning: wxSpinTreeCtrl is ignoring message: " << path << std::endl;
        return 1;
    }
     */
    // get the method (argv[0]):
    std::string theMethod;
    if (lo_is_string_type((lo_type) types[0]))
    {
        theMethod = std::string((char *) argv[0]);
    }
    else
        return 0;

    // SCENE MESSAGES:
    if (1)//(nodeString.empty())
    {
        if ((theMethod=="nodeList") && (argc>2))
        {
            for (int i = 2; i < argc; ++i)
            {
                if (strcmp((char*) argv[i], "NULL") != 0) treeCtrl->addNode((char*) argv[i]);
            }
        }
        else if ((theMethod=="createNode") && (argc==3))
        {
            treeCtrl->addNode((char*) argv[1]);
        }
        else if ((theMethod=="deleteNode") && (argc==2))
        {
            treeCtrl->removeNode((char*) argv[1]);
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
        else if ((theMethod=="parentChange") && (argc==3))
        {
            //std::cout << "got parentChange for node " << (char*)argv[1] << ". New parent: " << (char*)argv[2] << std::endl;
            //treeCtrl->removeNode((char*) argv[1]);
            //treeCtrl->addNode((char*) argv[1]);
            treeCtrl->addNode((char*)argv[1]);

        }
    }
/*
    // NODE MESSAGES:
    else
    {
        if ((theMethod=="setParent") && (argc==2))
        {
            treeCtrl->removeNode(nodeString.c_str());
            treeCtrl->addNode(nodeString.c_str());
        }
    }
*/
    return 1;
}

