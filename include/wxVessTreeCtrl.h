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
//  You should have received a copy of the Lesser GNU General Public License
//  along with SPIN Framework. If not, see <http://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------
//
//  NOTE: This file is based on source code from the Orihalcon Framework Library
//  Copyright (C) 2005 by Toshiyuki Takahei <takahei@orihalcon.jp>
//  (Released under the GNU Lesser General Public License)
//
// -----------------------------------------------------------------------------

#ifndef _wxVessTreeCtrl_H_
#define _wxVessTreeCtrl_H_

#include "asReferenced.h"

#include "vessWX.h"
#include "wxVessPropGrid.h"
//#include "wxVessEditor.h"

#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>
#include <osg/Drawable>
#include <osg/Texture>

//#include "wxOsg/wxOsg.h"
#include <wx/image.h>
#include <wx/imaglist.h>
#include <wx/treectrl.h>

class wxVessTreeVisitor;


class wxVessTreeItemData : public wxTreeItemData
{
  public:
    wxVessTreeItemData() {
        m_pNode = NULL;
    }

    osg::ref_ptr<asReferenced> m_pNode;
};

/**
 * \brief A wxWidgets TreeCtrl that provides an overview of the contents of a
 *       VESS scene.
 *
 * This class extends the regular wxTreeCtrl, adding methods for building the
 * tree from queries to the VESS sceneManager.
 */
class wxVessTreeCtrl : public wxTreeCtrl
{
public:

    wxVessTreeCtrl(wxWindow* parent, wxWindowID id,
                  const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxDefaultSize,
                  long style = wxTR_HAS_BUTTONS, const wxValidator& validator = wxDefaultValidator,
                  const wxString& name = wxT("treeCtrl"));
    virtual ~wxVessTreeCtrl();

    
    /**
     * Build the tree based on the contents of the sceneManager
     */
    void BuildTree(osg::Node* pRoot);
    
    /**
     * Refresh the tree based on the contents of the sceneManager
     */
    void Refresh();

    /**
     * Adds the asReferenced node to the tree
     */
    void addToTree(asReferenced *n, wxTreeItemId parentID);

    /**
     * This adds a node to the tree. Note that this function will typicall be 
     * called as a result of a createNode message broadcasted from vess AFTER
     * the node is actually instantiated in memory, so we don't need to actually
     * create it. We should be able to find it in the sceneManager, and just
     * create a tree item (using the addToTree() method).
     */
    void addNode(const char *id, const char *type);
    
    /**
     * Remove a node from the tree. Note that the node may not exist in the VESS
     * scene any more, so we need to perform all searches and comparisons only with
     * the string id provided.
     */
    void removeNode(const char *id);

    /**
     * This method allows us to select a node in the tree programatically. It is
     * useful, for example, when the selected node is deleted.
     */
    bool SelectNode(asReferenced* pNode);

    /**
     * GetTreeItem returns the wxTreeItemId given an asReferenced node pointer
     */
    wxTreeItemId GetTreeItem(asReferenced* pNode, wxTreeItemId idParent, wxTreeItemIdValue cookie=0);
    
    /**
     * GetTreeItem returns the wxTreeItemId given an id. This is useful in the case
     * when the node might not exist any more in the scene (eg, the node was deleted
     * and the deleteNode message was broadcasted, and now it must be removed from 
     * the TreeCtrl.
     */
    wxTreeItemId GetTreeItem(const char *nodeId, wxTreeItemId idParent, wxTreeItemIdValue cookie=0);
    
    /**
     * Get the current asReferenced node that the user has selected in the tree
     */
    asReferenced* GetSelectedNode() const;
    
    /**
     * Get the asReferenced node stored in a TreeCtrl leaf
     */
    asReferenced* GetNode(const wxTreeItemId& item) const;

    /**
     * Updates the tree item's icon
     */
    void UpdateTreeItemIcon(wxTreeItemId id);

    /**
     * When a user selects a node in the TreeCtrl, this event will populate the 
     * propgrid with that node's properties.
     */
    void OnVessSelectionChange(wxTreeEvent &event);

    /**
     * Sets the internal VessPropGrid pointer. This is needed because when a
     * node is selected in the tree, it will populate the prop grid below.
     */
    void SetPropGrid(wxVessPropGrid *PG);


protected:

    wxImageList* m_pImages;
    osg::ref_ptr<wxVessTreeVisitor> m_pSceneTreeVisitor;

    wxVessPropGrid* VessPropGrid;

    DECLARE_EVENT_TABLE()
};

/**
 * For VessTreeCtrl, we need to listen to OSC messages for high level events 
 * such as the creation/deletion of nodes so that we may update the tree 
 * accordingly.
 */
int wxVessTreeCtrl_liblo_callback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);


#endif // _wxVessTreeCtrl_H_
