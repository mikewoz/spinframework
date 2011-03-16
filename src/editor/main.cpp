/*
* This file is part of the SPIN Framework.
*
* Copyright (c) 2009 Mike Wozniewski
* Copyright (c) 2009 Zack Settel
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

#ifdef WX_PRECOMP
#include "wx_pch.h"
#endif

#ifdef __BORLANDC__
#pragma hdrstop
#endif //__BORLANDC__

#include "main.h"
#include "unused.h"

#include <wx/msgdlg.h>

#include <cppintrospection/Attributes>
#include <cppintrospection/Exceptions>
#include <cppintrospection/ExtendedTypeInfo>
#include <cppintrospection/MethodInfo>
#include <cppintrospection/PropertyInfo>
#include <cppintrospection/Reflection>
#include <cppintrospection/ReflectionMacros>
#include <cppintrospection/StaticMethodInfo>
#include <cppintrospection/Type>
#include <cppintrospection/TypedMethodInfo>
#include <cppintrospection/Value>
#include <cppintrospection/variant_cast>
#include <vector>
#include <string>
#include <iostream>

namespace spin
{
namespace editor
{
std::vector<std::string> listSpinNodeTypes()
{
    namespace intro = cppintrospection;
    std::vector<std::string> ret;
    try
    {
        const intro::Type &ReferencedNodeType = cppintrospection::Reflection::getType("ReferencedNode");
        const intro::TypeMap &allTypes = cppintrospection::Reflection::getTypes();
        cppintrospection::TypeMap::const_iterator it;
        for (it = allTypes.begin(); it != allTypes.end(); ++it)
        {
            if (((*it).second)->isDefined())
            {
                if ( ((*it).second)->isSubclassOf(ReferencedNodeType) )
                {
                    std::string theType = ((*it).second)->getName();
                    ret.push_back(theType);
                }
            }
        }
    }
    catch (const intro::Exception &ex)
    {
        std::cerr << __FUNCTION__ << ": Could not list node types:\n" << ex.what() << std::endl;
    }
    return ret;
}
} // end of namespace editor
} // end of namespace spin

//helper functions
enum wxbuildinfoformat
{
    short_f, long_f
};

wxString wxbuildinfo(wxbuildinfoformat format)
{
    wxString wxbuild(wxVERSION_STRING);

    if (format == long_f)
    {
#if defined(__WXMSW__)
        wxbuild << _T("-Windows");
#elif defined(__WXMAC__)
        wxbuild << _T("-Mac");
#elif defined(__UNIX__)
        wxbuild << _T("-Linux");
#endif

#if wxUSE_UNICODE
        wxbuild << _T("-Unicode build");
#else
        wxbuild << _T("-ANSI build");
#endif // wxUSE_UNICODE
    }

    return wxbuild;
}

MainFrame::MainFrame(wxFrame *frame) :
    GuiFrame(frame)
{
#if wxUSE_STATUSBAR
    statusBar->SetStatusText(_("Welcome to the SPIN Editor!"), 0);
    statusBar->SetStatusText(wxbuildinfo(short_f), 1);
#endif
    std::vector<std::string> nodeTypes = spin::editor::listSpinNodeTypes();
    std::cout << "SPIN Node types:" << std::endl;
    for (std::vector<std::string>::iterator iter = nodeTypes.begin(); iter != nodeTypes.end(); ++iter)
        std::cout << " * " << *iter << std::endl;
}

MainFrame::~MainFrame()
{
}

void MainFrame::OnClose(wxCloseEvent &event)
{
    UNUSED(event);
    Destroy();
}

void MainFrame::OnQuit(wxCommandEvent &event)
{
    UNUSED(event);
    Destroy();
}

void MainFrame::OnAbout(wxCommandEvent &event)
{
    UNUSED(event);
    wxString msg = wxbuildinfo(long_f);
    wxMessageBox(msg, _("Welcome to the SPIN Editor"));
}

