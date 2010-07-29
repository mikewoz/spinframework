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

#ifndef _wxSpinTreeCtrl_H_
#define _wxSpinTreeCtrl_H_

#include "ReferencedNode.h"

#include "spinWX.h"
#include "wxSpinPropGrid.h"
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


class wxSpinTreeItemData : public wxTreeItemData
{
  public:
    wxSpinTreeItemData() {
        m_pNode = NULL;
    }

    osg::ref_ptr<ReferencedNode> m_pNode;
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


    void setListeningServer(lo_server_thread t);

    /**
     * Build the tree based on the contents of the sceneManager
     */
    void BuildTree(osg::Node* pRoot);

    /**
     * Refresh the tree based on the contents of the sceneManager
     */
    void Refresh();

    /**
     * Adds the ReferencedNode node to the tree
     */
    void addToTree(ReferencedNode *n, wxTreeItemId parentID);

    /**
     * This adds a node to the tree. Note that this function will typicall be
     * called as a result of a createNode message broadcasted from SPIN AFTER
     * the node is actually instantiated in memory, so we don't need to actually
     * create it. We should be able to find it in the sceneManager, and just
     * create a tree item (using the addToTree() method).
     */
    void addNode(const char *id, const char *type);

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
    bool SelectNode(ReferencedNode* pNode);

    /**
     * GetTreeItem returns the wxTreeItemId given an ReferencedNode node pointer
     */
    wxTreeItemId GetTreeItem(ReferencedNode* pNode, wxTreeItemId idParent, wxTreeItemIdValue cookie=0);

    /**
     * GetTreeItem returns the wxTreeItemId given an id. This is useful in the case
     * when the node might not exist any more in the scene (eg, the node was deleted
     * and the deleteNode message was broadcasted, and now it must be removed from
     * the TreeCtrl.
     */
    wxTreeItemId GetTreeItem(const char *nodeId, wxTreeItemId idParent, wxTreeItemIdValue cookie=0);

    /**
     * Get the current ReferencedNode node that the user has selected in the tree
     */
    ReferencedNode* GetSelectedNode() const;

    /**
     * Get the ReferencedNode node stored in a TreeCtrl leaf
     */
    ReferencedNode* GetNode(const wxTreeItemId& item) const;

    /**
     * Updates the tree item's icon
     */
    void UpdateTreeItemIcon(wxTreeItemId id);

    /**
     * this will cause propgrid to populate with the selected node's properties.
     */
    void UpdatePropGrid();

    /**
     * Sets the internal SpinPropGrid pointer. This is needed because when a
     * node is selected in the tree, it will populate the prop grid below.
     */
    void SetPropGrid(wxSpinPropGrid *PG);


     /**
     * When a user selects a node in the TreeCtrl, this event is produced.
     */
    void OnSpinSelectionChange(wxTreeEvent &event);

    void OnSpinTreeDragBegin(wxTreeEvent &event);
    void OnSpinTreeDragEnd(wxTreeEvent &event);





protected:

    wxImageList* m_pImages;
    osg::ref_ptr<wxSpinTreeVisitor> m_pSceneTreeVisitor;

    wxSpinPropGrid* SpinPropGrid;

    lo_server_thread listeningServer;

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
