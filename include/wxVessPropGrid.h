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

#ifndef _WXVESSPROPGRID_H_
#define _WXVESSPROPGRID_H_

#include <osg/Node>
#include <osg/Texture>
#include <osg/Vec4f>
#include <osg/BoundingSphere>
#include <osg/Plane>
#include <osgIntrospection/PropertyInfo>

#include "asReferenced.h"
#include "lo/lo.h"
//#include "wxOsg/wxOsg.h"

#include <wx/propgrid/propgrid.h>
#include <wx/propgrid/propdev.h>
#include <wx/propgrid/advprops.h>
#include <wx/propgrid/manager.h>

#include <vector>

DECLARE_EVENT_TYPE(EVT_WXPG_OBJECT_SELECTED, -1);
DECLARE_EVENT_TYPE(EVT_WXPG_TOOLBAR_CLICKED, -1);

/*!
    @brief Property grid for VESS scene
*/
class wxVessPropGrid : public wxPropertyGridManager
{
public:

    //! Constructor
    wxVessPropGrid(wxWindow *parent, wxWindowID id = -1,
                  const wxPoint& pos = wxDefaultPosition,
                  const wxSize& size = wxDefaultSize,
                  long style = wxPG_SPLITTER_AUTO_CENTER | wxTAB_TRAVERSAL | wxPG_TOOLBAR | wxPG_DESCRIPTION,
                  const wxChar* name = wxT("propGrid"));

    //! Set a new node to show.
    void SetNode(asReferenced* pObject, bool forceUpdate=false);

    asReferenced* GetCurrentNode() const {
        return currentNode.get();
    }

    //! Updates all the properties based on getState() of currentNode
    void UpdateFromVess();



protected:

    //! Create a category and properties of the given object.
    void GenerateProperties(const osgIntrospection::Type& type, asReferenced* pObject);

    //! Property value changed event. Set the change to the osg object.
    void OnPropertyChanged(wxPropertyGridEvent& event);

    void OnToolbarClicked(wxCommandEvent& event);

    osg::ref_ptr<asReferenced> currentNode;    //!< Current osg object

    lo_server_thread listeningServer;

    DECLARE_EVENT_TABLE()
};

void lo_message_add_wxProp( lo_message msg, wxPGId propId );
void wxProp_from_lo_message(wxPGId parentId, const char *argTypes, int argc, lo_arg **args);

int wxVessPropGrid_liblo_callback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);

#endif // _wxVessPropGrid_H_
