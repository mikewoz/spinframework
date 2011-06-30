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

/** \file
 * The wxSpinTreeCtrl class.
 */

#ifndef _wxSpinTreeCtrl_H_
#define _wxSpinTreeCtrl_H_

#include "ReferencedNode.h"

//#include "spinWX.h"
//#include "wxSpinPropGrid.h"
//#include "wxSpinEditor.h"

#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>
#include <osg/Drawable>
#include <osg/Texture>

//#include "wxOsg/wxOsg.h"
#include <wx/image.h>
#include <wx/imaglist.h>
#include <wx/treectrl.h>

class wxSpinTreeVisitor;

/**
 * Item in a tree full of SPIN nodes.
 */
class wxSpinTreeItemData : public wxTreeItemData
{
  public:
    wxSpinTreeItemData()
    {
        m_pNode = NULL;
    }
    // FIXME: data should be private
    osg::observer_ptr<spin::ReferencedNode> m_pNode;
};

/**
 * \brief A wxWidgets TreeCtrl that provides an overview of the contents of a
 *       SPIN scene.
 *
 * This class extends the regular wxTreeCtrl, adding methods for building the
 * tree from queries to the SceneManager.
 */
class wxSpinTreeCtrl : public wxTreeCtrl
{
public:

    wxSpinTreeCtrl(wxWindow* parent, wxWindowID id,
                  const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxDefaultSize,
                  long style = wxTR_HAS_BUTTONS, const wxValidator& validator = wxDefaultValidator,
                  const wxString& name = wxT("treeCtrl"));
    virtual ~wxSpinTreeCtrl();

    //void setListeningServer(lo_server_thread t);

    /**
     * This registers the callback function (wxSpinTreeCtrl_liblo_callback) with
     * SPIN so that the TreeCtrl is updates when the scene changes. Note that
     * this function must be called again if the server ever changes.
     */
    void connectToSpin();

    /**
     * Build the tree based on the contents of the sceneManager
     */
    void BuildTree(osg::Node* pRoot);

    /**
     * Refresh the tree based on the contents of the sceneManager
     */
    void Refresh();

    /**
     * This adds a node to the tree. Note that this function will typically be
     * called as a result of a createNode message broadcasted from SPIN AFTER
     * the node is actually instantiated in memory, so we don't need to actually
     * create it. We should be able to find it in the sceneManager, and just
     * create a tree item (using the addToTree() method).
     */
    void addNode(const char *id);

    /**
     * Same as addNode but must match a particular node type
     */
    //void addNode(const char *id, const char *type);

    /**
     * Remove a node from the tree. Note that the node may not exist in the
     * scene any more, so we need to perform all searches and comparisons only
     * with the string id provided.
     */
    void removeNode(const char *id);

    /**
     * This method allows us to select a node in the tree programatically. It is
     * useful, for example, when the selected node is deleted.
     */
    bool SelectNode(spin::ReferencedNode* pNode);

    /**
     * Get the current ReferencedNode node that the user has selected in the tree
     */
    spin::ReferencedNode* GetSelectedNode() const;

    /**
     * Updates the tree item's icon
     */
    void UpdateTreeItemIcon(wxTreeItemId id);

private:

    /**
    * Adds the ReferencedNode node to the tree
    */
    void addToTree(spin::ReferencedNode *n);

    /**
     * Adds the ReferencedNode node to the tree, once you know the parent
     */
    void addToTree(spin::ReferencedNode *n, wxTreeItemId parentID);

    /**
     * GetTreeItem returns the wxTreeItemId given an ReferencedNode node pointer
     */
    //wxTreeItemId GetTreeItem(spin::ReferencedNode* pNode, wxTreeItemId idParent, wxTreeItemIdValue cookie=0);
    wxTreeItemId GetTreeItem(spin::ReferencedNode* pNode);

    /**
     * Get the wxTreeItemId given a node id string
     */
    wxTreeItemId GetTreeItem(const char *nodeId);

    /**
     * GetTreeItem returns the wxTreeItemId given an id. This is useful in the case
     * when the node might not exist any more in the scene (eg, the node was deleted
     * and the deleteNode message was broadcasted, and now it must be removed from
     * the TreeCtrl.
     */
    wxTreeItemId GetTreeItem(const char *nodeId, wxTreeItemId idParent, wxTreeItemIdValue cookie=0);

    /**
     * Get the ReferencedNode node stored in a TreeCtrl leaf
     */
    spin::ReferencedNode* GetNode(const wxTreeItemId& item) const;


    /**
     * this will cause propgrid to populate with the selected node's properties.
     */
    void UpdatePropGrid();

    /**
     * Sets the internal SpinPropGrid pointer. This is needed because when a
     * node is selected in the tree, it will populate the prop grid below.
     */
    //void SetPropGrid(wxSpinPropGrid *PG);

     /**
     * When a user selects a node in the TreeCtrl, this event is produced.
     */
    void OnSpinSelectionChange(wxTreeEvent &event);

    void OnSpinTreeDragBegin(wxTreeEvent &event);
    void OnSpinTreeDragEnd(wxTreeEvent &event);


    wxImageList* m_pImages;
    osg::ref_ptr<wxSpinTreeVisitor> m_pSceneTreeVisitor;

    //wxSpinPropGrid* SpinPropGrid;


    wxTreeItemId draggedItem;

    DECLARE_EVENT_TABLE()
};

/**
 * For SpinTreeCtrl, we need to listen to OSC messages for high level events
 * such as the creation/deletion of nodes so that we may update the tree
 * accordingly.
 */
int wxSpinTreeCtrl_liblo_callback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);

#endif // _wxSpinTreeCtrl_H_

