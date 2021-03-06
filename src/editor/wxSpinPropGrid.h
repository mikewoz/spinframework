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

#ifndef _WXSPINPROPGRID_H_
#define _WXSPINPROPGRID_H_

#include <cppintrospection/PropertyInfo>

#include "ReferencedNode.h"
#include "libloUtil.h"

#define wxUSE_PROPGRID 1

#include <wx/propgrid/propgrid.h>
//#include <wx/propgrid/propdev.h> // bug
#include <wx/propgrid/advprops.h>
#include <wx/propgrid/manager.h>

#include <vector>

//DECLARE_EVENT_TYPE(EVT_WXPG_OBJECT_SELECTED, -1);
//DECLARE_EVENT_TYPE(EVT_WXPG_TOOLBAR_CLICKED, -1);


/**
 * Property grid widget for editing a node.
 *
 * The widget is automatically generated using cppIntrospection
 */

class wxSpinPropGrid : public wxPropertyGridManager
{

public:

    //! Constructor
    wxSpinPropGrid(wxWindow *parent, wxWindowID id = -1,
                  const wxPoint& pos = wxDefaultPosition,
                  const wxSize& size = wxDefaultSize,
                  long style = wxPG_SPLITTER_AUTO_CENTER | wxTAB_TRAVERSAL | wxPG_TOOLBAR | wxPG_DESCRIPTION,
                  const wxChar* name = wxT("propGrid"));

    //! Set a new node to show.
    void SetNode(spin::ReferencedNode* pObject, bool forceUpdate=false);

    spin::ReferencedNode* GetCurrentNode() const {
        return currentNode.get();
    }

    //! Updates all the properties based on getState() of currentNode
    void UpdateFromSpin();

protected:

    //! Create a category and properties of the given object.
    void GenerateProperties(const cppintrospection::Type& type, spin::ReferencedNode* pObject);

    void OnPropertyChanging(wxPropertyGridEvent& event);

    //! Property value changed event. Set the change to the osg object.
    void OnPropertyChanged(wxPropertyGridEvent& event);

    void OnToolbarClicked(wxCommandEvent& event);

    osg::ref_ptr<spin::ReferencedNode> currentNode;    //!< Current osg object

    //lo_server_thread listeningServer;

    DECLARE_EVENT_TABLE()
};

void lo_message_add_wxProp( lo_message msg, wxPGProperty *propId );
void lo_message_add_wxProp( lo_message msg, wxPGProperty *propId, wxVariant val );

void wxProp_from_lo_message(wxPGProperty *parentId, const char *argTypes, int argc, lo_arg **args);

int wxSpinPropGrid_liblo_callback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);

#endif // _wxSpinPropGrid_H_
