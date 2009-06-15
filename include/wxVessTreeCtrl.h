


/*
 * ORIGINAL: wxOsgTreeCtrl.h :: part of Orihalcon Framework Library.
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


class wxVessTreeCtrl : public wxTreeCtrl
{
public:

    wxVessTreeCtrl(wxWindow* parent, wxWindowID id,
                  const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxDefaultSize,
                  long style = wxTR_HAS_BUTTONS, const wxValidator& validator = wxDefaultValidator,
                  const wxString& name = wxT("treeCtrl"));
    virtual ~wxVessTreeCtrl();

    void BuildTree(osg::Node* pRoot);
    void Refresh();

    void addToTree(asReferenced *n, wxTreeItemId parentID);

    void addNode(const char *id, const char *type);
    void removeNode(const char *id);


    bool SelectNode(asReferenced* pNode);

    wxTreeItemId GetTreeItem(asReferenced* pNode, wxTreeItemId idParent, wxTreeItemIdValue cookie=0);
    asReferenced* GetSelectedNode() const;
    asReferenced* GetNode(const wxTreeItemId& item) const;

    void UpdateTreeItemIcon(wxTreeItemId id);

/*
    void SetVisitor(wxVessTreeVisitor* pVisitor);
    wxVessTreeVisitor* GetVisitor();
    const wxVessTreeVisitor* GetVisitor() const;
*/

    void OnVessSelectionChange(wxTreeEvent &event);

    void SetPropGrid(wxVessPropGrid *PG);


protected:


    //bool SelectNode(asReferenced* pNode, wxTreeItemId idParent, wxTreeItemIdValue cookie=0);

    wxImageList* m_pImages;
    osg::ref_ptr<wxVessTreeVisitor> m_pSceneTreeVisitor;

    wxVessPropGrid* VessPropGrid;

    DECLARE_EVENT_TABLE()
};

int wxVessTreeCtrl_liblo_callback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);


#endif // _wxVessTreeCtrl_H_
