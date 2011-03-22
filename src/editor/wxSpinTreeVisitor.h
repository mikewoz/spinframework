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
 * The wxSpinTreeVisitor class.
 */
#ifndef _WXSPINTREEVISITOR_H_
#define _WXSPINTREEVISITOR_H_

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

class wxSpinTreeCtrl;

/**
 * Visits a wxTreeCtrl widget to populate it with SPIN nodes.
 */
class wxSpinTreeVisitor : public osg::NodeVisitor
{
public:

    wxSpinTreeVisitor(wxSpinTreeCtrl* pTreeCtrl);

    void SetParentTreeItem(wxTreeItemId* pParentId);

    virtual void apply(osg::Node& node);
    virtual void apply(osg::Group& node);
    /*
    virtual void apply(osg::Geode& node);
    virtual void apply(osg::Drawable& drawable);
    virtual void apply(osg::StateSet& stateSet);
    virtual void apply(osg::StateAttribute& stateAttrib);
    */
// FIXME: 2011-03-22:aalex:Get rid of protected methods
protected:
    //wxTreeItemId AddToTree(osg::Referenced* pObject);
    wxTreeItemId AddToTree(ReferencedNode* pObject);
    wxTreeItemId m_currentParentId;
    wxSpinTreeCtrl* m_pTreeCtrl;
};

#endif
