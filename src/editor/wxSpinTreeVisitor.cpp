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
#include "ReferencedNode.h"

wxSpinTreeVisitor::wxSpinTreeVisitor(wxSpinTreeCtrl* pTreeCtrl) :
    osg::NodeVisitor(osg::NodeVisitor::NODE_VISITOR, osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
{
    m_pTreeCtrl = pTreeCtrl;
}

void wxSpinTreeVisitor::SetParentTreeItem(wxTreeItemId* pParentId)
{
    if (pParentId)
        m_currentParentId = *pParentId;
    else
        m_currentParentId = wxTreeItemId();
}

void wxSpinTreeVisitor::apply(osg::Node& node)
{
    // for any node other than an osg::Group, just keep going:
    traverse(node);
}

void wxSpinTreeVisitor::apply(osg::Group& node)
{
    //std::cout << "wxSpinTreeVisitor parsing group " << node.getName() << "  (" << node.getNumChildren () << " children)" << std::endl;

    // for osg::Group, we check if it can be cast as an ReferencedNode, and add
    // it to the treeCtrl if so:

	ReferencedNode *n;

	if (n=dynamic_cast<ReferencedNode*>(&node)) {

	    wxTreeItemId parentId = m_currentParentId;
        //m_currentParentId = AddToTree(&node);
        m_currentParentId = AddToTree(n);

        traverse(node);

        m_pTreeCtrl->Expand(m_currentParentId);
        m_currentParentId = parentId;

	} else traverse(node);

}

/*

void wxSpinTreeVisitor::apply(osg::Node& node)
{
    AddToTree(&node);

    if (node.getStateSet())
    {
        apply(*(node.getStateSet()));
    }

    traverse(node);
}

void wxSpinTreeVisitor::apply(osg::Group& node)
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

void wxSpinTreeVisitor::apply(osg::Geode& node)
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

void wxSpinTreeVisitor::apply(osg::Drawable& drawable)
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

void wxSpinTreeVisitor::apply(osg::StateSet& stateSet)
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

void wxSpinTreeVisitor::apply(osg::StateAttribute& stateAttrib)
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


wxTreeItemId wxSpinTreeVisitor::AddToTree(ReferencedNode* n)
{

    wxTreeItemId currentId;
    if (!n) return currentId;

    std::string strLabel = n->nodeType + " : " + n->id->s_name;
    wxSpinTreeItemData *treeData = new wxSpinTreeItemData;
    treeData->m_pNode = n;


/*
    std::string strLabel = "*";
    ReferencedNode *n;
    osg::Node* pNamedNode;

	if (n=dynamic_cast<ReferencedNode*>(pObject))
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

