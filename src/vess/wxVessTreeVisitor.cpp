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


#include "wxVessTreeCtrl.h"
#include "wxVessTreeVisitor.h"
#include "asReferenced.h"

wxVessTreeVisitor::wxVessTreeVisitor(wxVessTreeCtrl* pTreeCtrl) :
    osg::NodeVisitor(osg::NodeVisitor::NODE_VISITOR, osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
{
    m_pTreeCtrl = pTreeCtrl;
}

void wxVessTreeVisitor::SetParentTreeItem(wxTreeItemId* pParentId)
{
    if (pParentId)
        m_currentParentId = *pParentId;
    else
        m_currentParentId = wxTreeItemId();
}

void wxVessTreeVisitor::apply(osg::Node& node)
{
    // for any node other than an osg::Group, just keep going:
    traverse(node);
}

void wxVessTreeVisitor::apply(osg::Group& node)
{

    // for osg::Group, we check if it can be cast as an asReferenced, and add
    // it to the treeCtrl if so:

	asReferenced *n;

	if (n=dynamic_cast<asReferenced*>(&node)) {

	    wxTreeItemId parentId = m_currentParentId;
        //m_currentParentId = AddToTree(&node);
        m_currentParentId = AddToTree(n);

        traverse(node);

        m_pTreeCtrl->Expand(m_currentParentId);
        m_currentParentId = parentId;

	} else traverse(node);

}

/*

void wxVessTreeVisitor::apply(osg::Node& node)
{
    AddToTree(&node);

    if (node.getStateSet())
    {
        apply(*(node.getStateSet()));
    }

    traverse(node);
}

void wxVessTreeVisitor::apply(osg::Group& node)
{
    //Commented out by mikewoz
    //if (dynamic_cast<orh::SelectionDecorator*>(&node))
    //    return;
    // end comment

    wxTreeItemId parentId = m_currentParentId;
    m_currentParentId = AddToTree(&node);

    if (node.getStateSet())
    {
        apply(*(node.getStateSet()));
    }

    traverse(node);
    m_pTreeCtrl->Expand(m_currentParentId);
    m_currentParentId = parentId;
}

void wxVessTreeVisitor::apply(osg::Geode& node)
{
    wxTreeItemId parentId = m_currentParentId;
    m_currentParentId = AddToTree(&node);

    for(unsigned int i=0;i<node.getNumDrawables();++i)
    {
        osg::Drawable* pDrawable = node.getDrawable(i);
        if (pDrawable)
        {
            apply(*pDrawable);
        }
    }

    m_pTreeCtrl->Expand(m_currentParentId);
    m_currentParentId = parentId;
}

void wxVessTreeVisitor::apply(osg::Drawable& drawable)
{
    wxTreeItemId parentId = m_currentParentId;
    m_currentParentId = AddToTree(&drawable);

    if (drawable.getStateSet())
    {
        apply(*(drawable.getStateSet()));
    }

    m_pTreeCtrl->Expand(m_currentParentId);
    m_currentParentId = parentId;
}

void wxVessTreeVisitor::apply(osg::StateSet& stateSet)
{
    wxTreeItemId parentId = m_currentParentId;
    m_currentParentId = AddToTree(&stateSet);

    osg::StateSet::AttributeList al = stateSet.getAttributeList();
    for (osg::StateSet::AttributeList::iterator aitr=al.begin(); aitr!=al.end(); ++aitr)
        apply(*aitr->second.first.get());

    osg::StateSet::TextureAttributeList tal = stateSet.getTextureAttributeList();
    unsigned int lenat = tal.size();
    for (unsigned int j=0; j<lenat; ++j)
    {
        // multi-texture?
        al = tal[j];
        for (osg::StateSet::AttributeList::iterator aitr=al.begin(); aitr!=al.end(); ++aitr)
            apply(*aitr->second.first.get());
    }

    m_pTreeCtrl->Expand(m_currentParentId);
    m_currentParentId = parentId;
}

void wxVessTreeVisitor::apply(osg::StateAttribute& stateAttrib)
{
    wxTreeItemId parentId = m_currentParentId;
    m_currentParentId = AddToTree(&stateAttrib);

    if (dynamic_cast<osg::Texture*>(&stateAttrib))
    {
        osg::Texture* pTexture = dynamic_cast<osg::Texture*>(&stateAttrib);
        unsigned int count = pTexture->getNumImages();
        for (unsigned int i=0; i<count; ++i)
            AddToTree(pTexture->getImage(i));
    }

    m_pTreeCtrl->Expand(m_currentParentId);
    m_currentParentId = parentId;
}

*/


wxTreeItemId wxVessTreeVisitor::AddToTree(asReferenced* n)
{

    wxTreeItemId currentId;
    if (!n) return currentId;

    std::string strLabel = n->nodeType + " : " + n->id->s_name;
    wxVessTreeItemData *treeData = new wxVessTreeItemData;
    treeData->m_pNode = n;


/*
    std::string strLabel = "*";
    asReferenced *n;
    osg::Node* pNamedNode;

	if (n=dynamic_cast<asReferenced*>(pObject))
	{
        strLabel = n->nodeType + " : " + n->id->s_name;
    }
    else if (pNamedNode = dynamic_cast<osg::Node*>(pObject))
    {
        strLabel = pNamedNode->className();
        if (!pNamedNode->getName().empty())
            strLabel = strLabel + " : " + pNamedNode->getName().c_str();
    }

    SceneNodeData *pNode = new SceneNodeData;
    pNode->m_pObject = pObject;
    */


    if (!m_currentParentId) {
        currentId = m_pTreeCtrl->AddRoot(wxString(strLabel.c_str(),wxConvUTF8), -1, -1, treeData);
        m_currentParentId = currentId;
    } else
        currentId = m_pTreeCtrl->AppendItem(m_currentParentId, wxString(strLabel.c_str(),wxConvUTF8), -1, -1, treeData);

    m_pTreeCtrl->UpdateTreeItemIcon(currentId);

    return currentId;
}
