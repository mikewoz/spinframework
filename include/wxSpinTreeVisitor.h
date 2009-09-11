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

protected:

    //wxTreeItemId AddToTree(osg::Referenced* pObject);
    wxTreeItemId AddToTree(ReferencedNode* pObject);

protected:
    wxTreeItemId m_currentParentId;
    wxSpinTreeCtrl* m_pTreeCtrl;
};

#endif
