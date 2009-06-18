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

#include <osgIntrospection/Value>
#include <osgIntrospection/Type>
#include <osgIntrospection/Reflection>
#include <osgIntrospection/MethodInfo>
#include <osgIntrospection/PropertyInfo>
#include <osgIntrospection/variant_cast>

#include <osgIntrospection/Exceptions>
#include <osgIntrospection/ReflectionMacros>
#include <osgIntrospection/TypedMethodInfo>
#include <osgIntrospection/StaticMethodInfo>
#include <osgIntrospection/Attributes>
#include <osgIntrospection/ExtendedTypeInfo>



//#include "wxOsg/wxOsgPGValueType.h"
#include "wxVessPropGrid.h"
#include "DebugVisitor.h"
#include "libloUtil.h"

#include "../images/icon_D.xpm"
//#include "../images/nav_prev.xpm"
//#include "../images/nav_next.xpm"
//#include "../images/nav_reload.xpm"

//#include <osgField/Manager>

#include "vessThreads.h"
extern vessMaster *vess;

using namespace osgIntrospection;



enum
{
    ID_TOOLBAR_DEBUG=1,
    ID_TOOLBAR_REFRESH,
};

DEFINE_EVENT_TYPE(EVT_WXPG_OBJECT_SELECTED);
DEFINE_EVENT_TYPE(EVT_WXPG_TOOLBAR_CLICKED);

BEGIN_EVENT_TABLE(wxVessPropGrid, wxPropertyGridManager)
    EVT_PG_CHANGED(wxID_ANY, wxVessPropGrid::OnPropertyChanged)
    EVT_MENU(wxID_ANY, wxVessPropGrid::OnToolbarClicked)
END_EVENT_TABLE()

/*!
    Constructor
    @param[in] pParent
    @param[in] id
    @param[in] pos
    @param[in] size
    @param[in] style
    @param[in] name
*/
wxVessPropGrid::wxVessPropGrid(wxWindow *parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style, const wxChar* name) : wxPropertyGridManager(parent, id, pos, size, style, name)
{
    SetExtraStyle(wxPG_EX_MODE_BUTTONS | wxPG_EX_HIDE_PAGE_BUTTONS);

    // Register *all* present property classes
    // (required for text-save/load support)
    RegisterAdvancedPropertyClasses();


    GetGrid()->SetVerticalSpacing(2);

    wxFont pFont = GetFont();
    pFont.SetPointSize(8.0f);


    //wxToolBar* pToolBar = GetToolBar();
    //pToolBar->AddTool(ID_TOOLBAR_DEBUG, wxBitmap(icon_D), wxT("DebugPrint"));

    //pToolBar->Realize();


    // We'll set up an OSC receiver to listen to messages for the currently
    // selected node. Note that we MUST listen to the broadcast channel and not
    // to vessChannel directly. This is because VESS might broadcast different
    // messages than it receives (eg, it receives a 'move' message, but will
    // transmit a 'setTranslation' message).
    if (vess->sceneManager->isSlave())
    {
        // if this sceneManager is already a listener, then we can just hijack
        // the rxServ and add our own method callbacks.
        listeningServer = vess->sceneManager->rxServ;
    }
    else
    {
        // if this sceneManager is a server, then we have to create a new server
        // that listens to the meassages it is broadcasting
        if (isMulticastAddress(vess->txAddr))
            listeningServer = lo_server_thread_new_multicast(vess->txAddr.c_str(), vess->txPort.c_str(), oscParser_error);
        else
            listeningServer = lo_server_thread_new(vess->txPort.c_str(), oscParser_error);
        lo_server_thread_start(listeningServer);
    }
}

/*!
    Set a new node to show.
    @param[in] newNode New object to show.
    @param[in] forcaUpdate If 'true', force update when the new object is same as the current displayed object.
*/
void wxVessPropGrid::SetNode(asReferenced* newNode, bool forceUpdate)
{

    if (currentNode == newNode && !forceUpdate) return;

    GetGrid()->Freeze();
    GetGrid()->Clear();


    const osgIntrospection::Type &asReferencedType = osgIntrospection::Reflection::getType("asReferenced");

    if (newNode)
    {
        osgIntrospection::Value pObjectValue = osgIntrospection::Value(newNode);
        const Type& classType = pObjectValue.getInstanceType();

        if (!classType.isDefined())
        {
            std::cout << "ERROR: osgIntrospection has no data for selected node." << std::endl;
            return;
        }

        //std::cout << "Populating property editor for: " << newNode->id->s_name << std::endl;
		//introspect_print_type(classType);



        // Go through the base classes and add properties for any base class
        // that is castable as asReferened. Start at the the furthest base level
        std::cout << classType.getQualifiedName() << " has " << classType.getNumBaseTypes() << " base classes:";
        for (int i=0; i<classType.getNumBaseTypes(); i++)
        {
            const Type& BaseClassType = classType.getBaseType(i);
            std::cout<<" "<< BaseClassType.getQualifiedName();
            if ((BaseClassType==asReferencedType) || (BaseClassType.isSubclassOf(asReferencedType)))
            {
                GenerateProperties(BaseClassType, newNode);
            }
        }
        std::cout << std::endl;

        // Parse this class:
        GenerateProperties(classType, newNode);

        // Add a callback method to the listeningServer that will listen for
        // messages related to the new node, and update props:
        if (listeningServer)
        {
            std::string oscPattern;

            // first remove the previously registered method:
            if (currentNode.valid())
            {
                oscPattern = "/vess/" + vess->id + "/" + std::string(currentNode->id->s_name);
                lo_server_del_method_with_userdata(lo_server_thread_get_server(listeningServer), oscPattern.c_str(), NULL, (void*)this);
            }

            // add the new method:
            oscPattern = "/vess/" + vess->id + "/" + std::string(newNode->id->s_name);
            lo_server_thread_add_method(listeningServer, oscPattern.c_str(), NULL, wxVessPropGrid_liblo_callback, (void*)this);
        }

        // finally, update our internal pointer:
        currentNode = newNode;

    }

    GetGrid()->Thaw();


    if (newNode) UpdateFromVess();





/*
    if (!bNoEvent)
    {
        // Send an event to change scene graph tree control, etc.
        wxCommandEvent event(EVT_WXPG_OBJECT_SELECTED, GetId());
        event.SetEventObject(this);
        event.SetClientData(pObject);
        GetEventHandler()->ProcessEvent(event);
    }
    */
}

void wxVessPropGrid::UpdateFromVess()
{

    GetGrid()->Freeze();

    if (currentNode.valid())
    {
        std::vector<lo_message> nodeState = currentNode->getState();

        // iterate through state messages and set the corresponding wxProperty:
        std::vector<lo_message>::iterator nodeStateIterator = nodeState.begin();
        while (nodeStateIterator != nodeState.end())
        {
            char *argTypes = lo_message_get_types(*nodeStateIterator);
            int argc = lo_message_get_argc(*nodeStateIterator);
            lo_arg **args = lo_message_get_argv(*nodeStateIterator);

            // get the parent property (should be in the form "nodeType.methodName")
            std::string parentString = currentNode->nodeType + "." + std::string((char*)args[0]);
            wxPGId parentId = this->GetGrid()->GetRoot()->GetPropertyByName( wxString(parentString.c_str(), wxConvUTF8) );

            wxProp_from_lo_message(parentId, argTypes, argc, args);

            lo_message_free(*nodeStateIterator);
            nodeState.erase(nodeStateIterator); //note: iterator automatically advances after erase()
        }
    }

    GetGrid()->Thaw();
}


void wxVessPropGrid::GenerateProperties(const osgIntrospection::Type& classType, asReferenced* pObject)
{

	//std::cout << "wxVessPropGrid::GenerateProperties for class=" << classType.getQualifiedName() << ", id=" << pObject->id->s_name << std::endl;

    wxString className = wxString(classType.getQualifiedName().c_str(), wxConvUTF8);


    // Create a category for the object
    wxPGId categoryId = Append(new wxPropertyCategory(className));
    wxPGId parentId = categoryId;

    // Look through all methods
    const MethodInfoList& methods = classType.getMethods();
    for (MethodInfoList::const_iterator methodIter=methods.begin(); methodIter!=methods.end(); methodIter++)
    {
        const MethodInfo* method = *methodIter;
        const ParameterInfoList& params = method->getParameters();

        //std::cout << "  Found method: " << method->getName() << ", numParams=" << params.size() << ", virtual?=" << method->isVirtual() << ", void?=" << method->getReturnType().isVoid() << std::endl;
        wxString methodName = wxString(method->getName().c_str(), wxConvUTF8);

        // We look for all void methods which take at least 1 argument. However,
        // we need to ignore virtual methods, since they are catergorized in the
        // base class property set.

        if ( !method->isVirtual() && method->getReturnType().isVoid() && params.size() )
        {

            // if there are multiple parameters then make a string property
            // and add all params as children; otherwise, just add them to
            // the category
            if (params.size() > 1)
            {
                //parentId = AppendIn(categoryId, new wxStringProperty(methodName, wxPG_LABEL, wxT("")));
                parentId = AppendIn(categoryId, new wxPGPropertyWithChildren(methodName, wxPG_LABEL));
                EnableProperty(parentId, false);
            } else {
                parentId = categoryId;
            }

            // add all params to propery editor:
            for (ParameterInfoList::const_iterator paramIter=params.begin(); paramIter!=params.end(); paramIter++)
            {
                const ParameterInfo* param = *paramIter;


                if (param->getParameterType().isDefined())
                {
                    //std::cout << "    param: " << param->getName() << ", type: " << param->getParameterType().getQualifiedName() << std::endl;

                    // If it's just one param, display the method as the label,
                    // otherwise the method is already displayed as a parent so
                    // we can show the actual parameter name:
                    wxString propName;
                    if (params.size() > 1)
                        propName = wxString(param->getName().c_str(), wxConvUTF8);
                    else
                        propName = methodName;

                    wxPGId propId;
                    std::string t = param->getParameterType().getQualifiedName();

                    if (param->getParameterType().isEnum())
                    {
                        const EnumLabelMap enumLabels = param->getParameterType().getEnumLabels();
                        wxPGChoices chs;
                        wxArrayString arrLabels;
                        wxArrayInt arrIDs;

                        for (EnumLabelMap::const_iterator enumIter=enumLabels.begin(); enumIter!=enumLabels.end(); enumIter++)
                        {
                            arrLabels.Add(wxString((*enumIter).second.c_str(), wxConvUTF8));
                            arrIDs.Add((*enumIter).first);
                            chs.Add(wxString((*enumIter).second.c_str(), wxConvUTF8),(*enumIter).first);
                        }
                        //propId = AppendIn(parentId, new wxEnumProperty(propName, wxPG_LABEL, chs) );
                        propId = AppendIn(parentId, new wxEnumProperty(propName, wxPG_LABEL, arrLabels, arrIDs, 0));
                    }

                    else
                    {
                        // go through all of our possible types in order to create
                        // appropriate property:

                        if (t=="bool") {
                            //propId = AppendIn(parentId, new wxBoolProperty(propName, wxPG_LABEL, variant_cast<bool>(propValue)));
                            propId = AppendIn(parentId, new wxBoolProperty(propName, wxPG_LABEL, false));
                        }
                        else if (t=="int") {
                            //propId = AppendIn(parentId, new wxIntProperty(propName, wxPG_LABEL, variant_cast<int>(propValue)));
                            propId = AppendIn(parentId, new wxIntProperty(propName, wxPG_LABEL, 0));
                        }
                        else if (t=="float") {
                            //propId = AppendIn(parentId, new wxFloatProperty(propName, wxPG_LABEL, variant_cast<float>(propValue)));
                            propId = AppendIn(parentId, new wxFloatProperty(propName, wxPG_LABEL, 0.0));
                        }
                        else if (t=="double") {
                            //propId = AppendIn(parentId, wxFloatProperty(propName, wxPG_LABEL, variant_cast<double>(propValue)));
                            propId = AppendIn(parentId, new wxFloatProperty(propName, wxPG_LABEL, 0.0));
                        }
                        else if (t=="std::string")
                        {
                            //propId = AppendIn(parentId, wxStringProperty(propName, wxPG_LABEL, (variant_cast<std::string>(propValue)).c_str()));
                            propId = AppendIn(parentId, new wxStringProperty(propName, wxPG_LABEL, wxT("default") ));
                        }
                        else if (t=="char *" || t=="const char *")
                        {
                            //propId = AppendIn(parentId, wxStringProperty(propName, wxPG_LABEL, (variant_cast<std::string>(propValue)).c_str()));
                            propId = AppendIn(parentId, new wxStringProperty(propName, wxPG_LABEL, wxT("default") ));
                        }
                        else
                        {
                            /*
                            propId = AppendIn(parentId, new wxStringProperty(propName, wxPG_LABEL, wxT("[unkown type]") ));
                            EnableProperty(propId, false);
                            */
                        }
                    }

                    // Make a description.
                    if (propId)
                    {
                        wxString helpText;
                        helpText += wxT("Type: ") + wxString(t.c_str(), wxConvUTF8) + wxT("\n");
                        if (!IsPropertyEnabled(propId))
                        {
                            helpText += wxT("Read-only");
                        }
                        else
                        {
                            helpText += wxT("Full OSC message: /vess/") + wxString(vess->id.c_str(), wxConvUTF8) + wxT("/") + wxString(pObject->id->s_name, wxConvUTF8);
                            helpText += wxT(" ") + methodName + wxT(" <");
                            for (ParameterInfoList::const_iterator paramIter2=params.begin(); paramIter2!=params.end(); paramIter2++)
                            {
                                helpText += wxT(" ") + wxString((*paramIter2)->getParameterType().getQualifiedName().c_str(), wxConvUTF8);
                            }
                            helpText += wxT(" >\n");
                        }
                        helpText += wxString(method->getBriefHelp().c_str(), wxConvUTF8) + wxT("\n");
                        helpText += wxString(method->getDetailedHelp().c_str(), wxConvUTF8) + wxT("\n");
                        SetPropertyHelpString(propId, helpText);
                    }

                } // if param type isDefined
                else
                {
                    std::cout << "    param: " << param->getName() << ", type: [undefined]" << std::endl;
                }

            } // param iterator


            if (params.size() > 1)
            {
                wxString composedValue;
                parentId->GenerateComposedValue(composedValue);
                parentId->SetValueFromString(composedValue);
                //std::cout << "composed value for " << methodName.mb_str() << "= " << composedValue.mb_str() << std::endl;
            }

        } // if desired method

    } // method iterator
}

/*!
    Property value changed event. Set the change to the osg object.
    @param[in] event Event data.
*/
void wxVessPropGrid::OnPropertyChanged(wxPropertyGridEvent& event)
{
    wxPGId id = event.GetProperty();
    if (!id) return;

    wxPGId parent = event.GetMainParent();

    //std::cout << "OnPropertyChanged: parent=" << parent->GetBaseName().mb_str() << ", numChildren=" << parent->GetChildCount() << ", id=" << id->GetBaseName().mb_str() << ", valueAsString=" << id->GetValueAsString().mb_str() << std::endl;

    lo_message msg = lo_message_new();
    lo_message_add_string(msg, parent->GetBaseName().mb_str());

    if (parent->GetChildCount())
    {
        for (int i=0; i<parent->GetChildCount(); i++)
        {
            lo_message_add_wxProp( msg, parent->Item(i) );
        }
    }

    else {
        lo_message_add_wxProp( msg, id );
    }

	std::string OSCpath = "/vess/" + vess->id + "/" + std::string(currentNode->id->s_name);

	// If this sceneManager is just a listener, then we nees to send to the
	// broadcastChannel. Otherwise, we send to the (unicast) rxAddr of this
	// server.
    if (vess->sceneManager->isSlave())
    {
        lo_send_message(vess->lo_infoServ, OSCpath.c_str(), msg);
    }
    else
        lo_send_message(vess->sceneManager->rxAddr, OSCpath.c_str(), msg);


	lo_message_free(msg);

	//event.Veto();
	//event.Skip();
}


void wxVessPropGrid::OnToolbarClicked(wxCommandEvent& event)
{
    switch (event.GetId())
    {
        case ID_TOOLBAR_DEBUG:
        {
            if (currentNode.valid()) currentNode->debug();
            break;
        }
        default:
        {
            wxCommandEvent myEvent(EVT_WXPG_TOOLBAR_CLICKED, GetId());
            myEvent.SetEventObject(this);
            myEvent.SetInt(event.GetId());
            GetEventHandler()->ProcessEvent(myEvent);
        }
    }
}


void lo_message_add_wxProp( lo_message msg, wxPGId propId )
{
    std::string t = std::string(propId->GetValueType().mb_str());

    //std::cout << "  propId: " << propId->GetBaseName().mb_str() << ", type=" << t << ", value=" << propId->GetValueAsString().mb_str() << std::endl;

    if ( t=="string" || t=="std::string" || t=="char *" || t=="const char *" )
    {
        //lo_message_add_string( msg, (const char*) propId->GetValue().GetString().c_str() );
        lo_message_add_string( msg, (const char*) propId->GetValueAsString().mb_str(wxConvUTF8) );
    }
    else if ( t=="float" || t=="double" )
    {
        lo_message_add_float( msg, (float) propId->GetValue().GetDouble() );
    }
    /*
    else if ( t=="double" )
    {
        lo_message_add_double( msg, (double) propId->GetValue().GetDouble() );
    }
    */
    else if ( t=="int" || t=="long" )
    {
        lo_message_add_int32( msg, (int) propId->GetValue().GetInteger() );
    }
    else if ( t=="bool" )
    {
        lo_message_add_int32( msg, (int) propId->GetValue().GetBool() );
    }

}

void wxProp_from_lo_message(wxPGId parentId, const char *argTypes, int argc, lo_arg **argv)
{
    int i;

    // Go through the args and update properties or subproperties of parentId
    // (but remember that the 1st arg is a method name, so ignore that one)

    if (parentId)
    {


        for (i=1; i<argc; i++)
        {
            wxPGId propId;
            if (parentId->GetChildCount()) propId = parentId->Item(i-1);
            else propId = parentId;

            /*
            std::cout << " setting wxProp: " << propId->GetBaseName().mb_str() << " (type=" << propId->GetValueType().mb_str() << ") from lo_arg:";
            if (lo_is_numerical_type((lo_type)argTypes[i]))
            {
                std::cout << " " << (float) lo_hires_val((lo_type)argTypes[i], argv[i]);
            } else {
                std::cout << " " << (char*) argv[i];
            }
            std::cout << " (type=" << argTypes[i] << ")" << std::endl;
            */


            if (argTypes[i]=='s')
            {
                propId->SetValueFromString( wxString( (char*)argv[i], wxConvUTF8 ) );
            }
            else if (argTypes[i]=='i')
            {
                propId->SetValueFromInt( (long) lo_hires_val( (lo_type)argTypes[i], argv[i] ) );
                //propId->SetValue( (int) lo_hires_val( (lo_type)argTypes[i], argv[i] ) );
            }
            else if (argTypes[i]=='f')
            {
                propId->SetValue( (float) lo_hires_val( (lo_type)argTypes[i], argv[i] ) );
            }
            else if (argTypes[i]=='d')
            {
                propId->SetValue( (double) lo_hires_val( (lo_type)argTypes[i], argv[i] ) );
            }
        }

        if (parentId->GetChildCount())
        {
            wxString composedValue;
            parentId->GenerateComposedValue(composedValue);
            parentId->SetValueFromString(composedValue);
        }

    }
}


int wxVessPropGrid_liblo_callback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data)
{
    // make sure there is at least one argument (ie, a method to call):
	if (!argc) return 0;

    wxVessPropGrid *propGrid = (wxVessPropGrid*) user_data;

    if (!propGrid) return 0;

    std::string parentString = propGrid->GetCurrentNode()->nodeType + "." + std::string((char*)argv[0]);
    wxPGId parentId = propGrid->GetGrid()->GetRoot()->GetPropertyByName( wxString(parentString.c_str(), wxConvUTF8) );

    std::cout << "Got update for current propgrid node: " << parentString << std::endl;

    if (parentId)
    {
        propGrid->GetGrid()->Freeze();
        wxProp_from_lo_message(parentId, types, argc, argv);
        //propGrid->GetGrid()->RefreshProperty(parentId);
        propGrid->GetGrid()->Thaw();
    }

	return 1;
}
