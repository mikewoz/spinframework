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

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>

#include <cppintrospection/Value>
#include <cppintrospection/Type>
#include <cppintrospection/Reflection>
#include <cppintrospection/MethodInfo>
#include <cppintrospection/PropertyInfo>
#include <cppintrospection/variant_cast>

#include <cppintrospection/Exceptions>
#include <cppintrospection/ReflectionMacros>
#include <cppintrospection/TypedMethodInfo>
#include <cppintrospection/StaticMethodInfo>
#include <cppintrospection/Attributes>

#include "spinapp.h"
#include "referencednode.h"

using namespace cppintrospection;
using namespace std;


#define OUTPUT_FILE "oscprotocol.html"
#define DEBUG 0

namespace docgenerator
{


static bool IsGoodParam(const ParameterInfo& param)
{
	/*
	vector<string> validParams;
	validParams.push_back("const char *");
	validParams.push_back("char *");
	validParams.push_back("int");
	validParams.push_back("float");
	validParams.push_back("double");
	validParams.push_back("bool");
	*/
	
	if ( !param.getParameterType().isDefined() )
	{
		if (DEBUG) cout << "  bad param: " << param.getName() << endl;
		return false;
	}
	
	return true;
}

static bool IsGoodMethod(const MethodInfo& method)
{

    // must return void (ie, be a set method)
    if ( !method.getReturnType().isVoid() ) return false;
	
    // virtual methods will eventually be parsed in base class:
    if ( method.isVirtual() ) return false;
    
    // must take at least param??
    //if ( !method.getNumParams() ) return false
    
	// look for some particular method names to ignore:
	if (method.getName()=="registerNode") return false;
	if (method.getName()=="attach") return false;
	if (method.getName()=="detach") return false;
	if (method.getName()=="updateChildNodePaths") return false;
	//if (method.getName()=="updateNodePath") return false;
	//if (method.getName()=="callbackUpdate") return false;
    
	
	return true;
}
	

static void GenerateHTML(const cppintrospection::Type &classType, ofstream& output)
{
	/*
	output << "<code>\n";
	output << "  /SPIN/{sceneID}/{nodeID} <strong>setParent</strong>  <<font color='gray'>(char *)</font> <strong>newParent</strong>>\n";
	output << "</code><br>\n";
	output << "&nbsp;&nbsp;&nbsp;&rarr; <small>This method sets the location of the node in the scene graph, adopting the local coordinate system of the newParent.</small><br><br>\n";
	*/

	if (DEBUG) cout << "TYPE: " << classType.getName() << endl;

	
	// Look through all methods
	const MethodInfoList& methods = classType.getMethods();
	//MethodInfoList methods;
	//classType.getAllMethods(methods);
    
    for (MethodInfoList::const_iterator methodIter=methods.begin(); methodIter!=methods.end(); methodIter++)
    {
        const MethodInfo* method = *methodIter;
        const ParameterInfoList& params = method->getParameters();

        // We look for all void methods which take at least 1 argument. However,
        // we need to ignore virtual methods, since they are catergorized in the
        // base class property set.
        
        
        if (DEBUG) cout << "  METHOD: " << method->getName() << "  (virtual=" << method->isVirtual() << ", void=" << method->getReturnType().isVoid() << ", numParams=" << params.size() << ")\t\t";


        if (IsGoodMethod(*method))
        {
       
        	output << "<div class='memitem'>\n";
        	
        	//output << "<div class='memproto'>\n";
        	output << "<div>\n";
        	//output << "<code>\n";
        	output << "  /SPIN/{sceneID}/{nodeID} <strong>" << method->getName() << "</strong>\n";
        	
        	
        	if (DEBUG) cout << "  PARAMS: ";
        	
            // add all param names
        	for (ParameterInfoList::const_iterator paramIter=params.begin(); paramIter!=params.end(); paramIter++)
        	{
        		const ParameterInfo* param = *paramIter;
        		
        		if (!IsGoodParam(*param)) continue;
        		
        		output << "    &lt;";

        		if (param->getParameterType().isEnum())
        		{
        			// paramter is a enum, so just provide all choices
        			const EnumLabelMap enumLabels = param->getParameterType().getEnumLabels();
        			for (EnumLabelMap::const_iterator enumIter=enumLabels.begin(); enumIter!=enumLabels.end(); enumIter++)
        			{
        				if (enumIter!=enumLabels.begin()) output << ",";
        				output << "'" << (*enumIter).second.c_str() << "'";
        			}
        		}
        		else
        		{
        			//vector<string>::const_iterator it = find(validParams.begin(), validParams.end(), param->getParameterType().getQualifiedName());
        			//if (it != validParams.end())
        			//{
            			// parameter type is standard:
            			output << "<font color='gray'>(" << param->getParameterType().getQualifiedName() << ")</font> <strong>" << param->getName() << "</strong>";
        			//}
            			
            		if (DEBUG) cout << "(" << param->getParameterType().getQualifiedName() << ") " << param->getName() << ", ";
        		}
        		
        		output << "&gt;\n";
        	}
        	
        	//output << "</code>\n";
        	output << "</div>\n";
        	
        	
        	// output description:
        	
        	output << "<div class='memdoc'>\n";
        	output << "<blockquote>\n";
            if (!method->getDetailedHelp().empty())
            {
            	output << "<small>" << method->getDetailedHelp() << "</small>\n"; 
            }
            else if (!method->getBriefHelp().empty())
            {
            	output << "<small>" << method->getBriefHelp() << "</small>\n";
            } else {
            	output << "<font color='gray'><small>&lt;no description available&gt;</small></font>\n";
            }
            /*
            if (!method->getDetailedHelp().empty())
            {
            	output << "Details: " << method->getDetailedHelp() << "<br>\n";
            }
            */
            output << "</blockquote>\n";
            output << "</div>\n";
            


        	output << "</div>\n";
        	output << "<br>\n";
            output << "\n";
            
            if (DEBUG) cout << endl;
            
        } // if desired method
        
        else if (DEBUG) cout << "  (rejected)" << endl;

    } // method iterator
    

}


} // end of namespace docgenerator


int main()
{
	using namespace docgenerator;
	//std::ostringstream output("");
	
	// loading an instance of spinApp will force the spin library to load:
	spin::spinApp &spin = spin::spinApp::Instance();

	ofstream output(OUTPUT_FILE);
	if (!output.is_open())
	{
		cout << "Error: Could not open file '" << OUTPUT_FILE << "'" << endl;
		return 1;
	}
	
	const cppintrospection::Type &ReferencedNodeType = cppintrospection::Reflection::getType("spin::ReferencedNode");
	const cppintrospection::Type &ReferencedStateSetType = cppintrospection::Reflection::getType("spin::ReferencedStateSet");
	/*
	output << "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.01 Transitional//EN\">\n";
	output << "<html><head><meta http-equiv=\"Content-Type\" content=\"text/html;charset=UTF-8\">\n";
	output << "<title>SPIN (Spatial Interaction) Framework</title>\n";
	output << "<link href=\"doxygen.css\" rel=\"stylesheet\" type=\"text/css\">\n";
	output << "<link href=\"tabs.css\" rel=\"stylesheet\" type=\"text/css\">\n";
	output << "</head><body>\n";
	output << "<div class=\"navigation\" id=\"top\">\n";
	output << "  <div class=\"tabs\">\n";
	output << "    <ul>\n";
	output << "      <li><a href=\"index.html\"><span>Main&nbsp;Page</span></a></li>\n";
	output << "      <li class=\"current\"><a href=\"pages.html\"><span>Related&nbsp;Pages</span></a></li>\n";
	output << "      <li><a href=\"annotated.html\"><span>Classes</span></a></li>\n";
	output << "      <li><a href=\"files.html\"><span>Files</span></a></li>\n";
	output << "    </ul>\n";
	output << "  </div>\n";
	output << "</div>\n";
	output << "<div class=\"contents\">\n";
	output << "<h1>OSC Protocol for SPIN Framework</h1>\n";
	output << "<p>\n";
	*/
    
    output << "<!DOCTYPE html PUBLIC \"-//W3C//DTD XHTML 1.0 Transitional//EN\" \"http://www.w3.org/TR/xhtml1/DTD/xhtml1-transitional.dtd\">\n";
    output << "<html xmlns=\"http://www.w3.org/1999/xhtml\">\n";
    output << "<head>\n";
    output << "<meta http-equiv=\"Content-Type\" content=\"text/xhtml;charset=UTF-8\"/>\n";
    output << "<title>SPIN Framework: SPIN (Spatial Interaction) Framework</title>\n";
    output << "<link href=\"tabs.css\" rel=\"stylesheet\" type=\"text/css\"/>\n";
    output << "<link href=\"doxygen.css\" rel=\"stylesheet\" type=\"text/css\"/>\n";
    output << "</head>\n";
    output << "<body>\n";
    output << "<!-- Generated by Doxygen 1.7.3 -->\n";
    output << "<div id=\"top\">\n";
    output << "<div id=\"titlearea\">\n";
    output << "<table cellspacing=\"0\" cellpadding=\"0\">\n";
    output << " <tbody>\n";
    output << " <tr style=\"height: 56px;\">\n";
    output << "  <td style=\"padding-left: 0.5em;\">\n";
    output << "   <div id=\"projectname\">SPIN Framework</div>\n";
    output << "  </td>\n";
    output << " </tr>\n";
    output << " </tbody>\n";
    output << "</table>\n";
    output << "</div>\n";
    output << "  <div id=\"navrow1\" class=\"tabs\">\n";
    output << "    <ul class=\"tablist\">\n";
    output << "      <li><a href=\"index.html\"><span>Main&#160;Page</span></a></li>\n";
    output << "      <li class=\"current\"><a href=\"pages.html\"><span>Related&#160;Pages</span></a></li>\n";
    output << "      <li><a href=\"modules.html\"><span>Modules</span></a></li>\n";
    output << "      <li><a href=\"namespaces.html\"><span>Namespaces</span></a></li>\n";
    output << "      <li><a href=\"annotated.html\"><span>Classes</span></a></li>\n";
    output << "      <li><a href=\"files.html\"><span>Files</span></a></li>\n";
    output << "    </ul>\n";
    output << "  </div>\n";
    output << "</div>\n";
    output << "<div class=\"header\">\n";
    output << "  <div class=\"headertitle\">\n";
    output << "<h1>SPIN OSC Protocol</h1>  </div>\n";
    output << "</div>\n";
    output << "<div class=\"contents\">\n";
    
	output << "<p>The SPIN Framework is designed so multiple processes can "
		   << "share state over a network via OpenSoundControl (OSC) messages. "
		   << "Below is the complete list of accepted OSC messages for the "
		   << "following nodes:</p>\n";
		   
	const cppintrospection::TypeMap &allTypes = cppintrospection::Reflection::getTypes();
	cppintrospection::TypeMap::const_iterator it;
	
	output << "<ul>\n";
	for (it=allTypes.begin(); it!=allTypes.end(); it++)
	{
		const cppintrospection::Type *classType = ((*it).second);
		
		if (classType->isDefined())
		{
		
			if ( classType->isSubclassOf(ReferencedNodeType) )
			{
				output << "  <li><a href='#" << classType->getName() << "'>" << classType->getName() << "</a></li>\n";
				
			}
		}
	}
	output << "</ul>\n";
	
	output << "<p>State Types:</p>\n";
	
	output << "<ul>\n";
	for (it=allTypes.begin(); it!=allTypes.end(); it++)
	{
		const cppintrospection::Type *classType = ((*it).second);
		
		if (classType->isDefined())
		{
		
			if ( classType->isSubclassOf(ReferencedStateSetType) )
			{
				output << "  <li><a href='#" << classType->getName() << "'>" << classType->getName() << "</a></li>\n";
				
			}
		}
	}
	output << "</ul>\n";	
	
	
	for (it=allTypes.begin(); it!=allTypes.end(); it++)
	{
		const cppintrospection::Type *classType = ((*it).second);
		
		if (classType->isDefined())
		{
		
			if ( classType->isSubclassOf(ReferencedNodeType) || classType->isSubclassOf(ReferencedStateSetType) )
			{

				output << "<hr\n";
				output << "<h2><a class='anchor' name='" << classType->getName() << "'>" << classType->getName() << ":</a></h2>\n";
				output << "<blockquote>\n\n";

				
		        // First go through all base classes and generateHTML:
				if (DEBUG) cout << endl << classType->getQualifiedName() << " has " << classType->getNumBaseTypes() << " base classes:" << endl;
		        for (int i=0; i<classType->getNumBaseTypes(); i++)
		        {
		            const Type &BaseClassType = classType->getBaseType(i);
		            
		            if ((BaseClassType==ReferencedNodeType) || (BaseClassType.isSubclassOf(ReferencedNodeType)) ||
		            	(BaseClassType==ReferencedStateSetType) || (BaseClassType.isSubclassOf(ReferencedStateSetType)))
		            {
		            	GenerateHTML(BaseClassType, output);
		            }
		        }
		        
		        // Then do this class:
		        GenerateHTML(*classType, output);
		        
		    	output << "</blockquote>\n";	


			} // if ReferencedNode
		} // if class is defined
	} // type iter
	
	
	//cout << output.str() << endl;
	
    output << "</div>\n";
    output << "</body>\n";
    output << "</html>\n";
    
	output.close();
	
	cout << "Created file: '" << OUTPUT_FILE << "'" << endl;
	
	return 0;
	
} // main()
