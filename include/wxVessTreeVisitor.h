


/*
 * ORIGINAL: wxOsgSceneTreeVisitor.h :: part of Orihalcon Framework Library.
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

#ifndef _WXVESSTREEVISITOR_H_
#define _WXVESSTREEVISITOR_H_

#include <osg/NodeVisitor>
#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>
#include <osg/Drawable>
#include <osg/StateSet>
#include <osg/StateAttribute>
#include <osg/Texture>
#include <osg/Image>

//#include "wxOsg/wxOsg.h"
#include <wx/treectrl.h>
#include <wx/string.h>

class wxVessTreeCtrl;

class wxVessTreeVisitor : public osg::NodeVisitor
{
public:

    wxVessTreeVisitor(wxVessTreeCtrl* pTreeCtrl);

    void SetParentTreeItem(wxTreeItemId* pParentId);

    virtual void apply(osg::Node& node);
    virtual void apply(osg::Group& node);
    /*
    virtual void apply(osg::Geode& node);
    virtual void apply(osg::Drawable& drawable);
    virtual void apply(osg::StateSet& stateSet);
    virtual void apply(osg::StateAttribute& stateAttrib);
    */

protected:

    //wxTreeItemId AddToTree(osg::Referenced* pObject);
    wxTreeItemId AddToTree(asReferenced* pObject);

protected:
    wxTreeItemId m_currentParentId;
    wxVessTreeCtrl* m_pTreeCtrl;
};

#endif
