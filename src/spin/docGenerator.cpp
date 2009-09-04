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

using namespace osgIntrospection;
using namespace std;


#define OUTPUT_FILE "oscprotocol.html"
#define DEBUG 0


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
	

static void GenerateHTML(const osgIntrospection::Type &classType, ofstream& output)
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
            if (!method->getBriefHelp().empty())
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





int main()
{
	//std::ostringstream output("");
	
	ofstream output(OUTPUT_FILE);
	if (!output.is_open())
	{
		cout << "Error: Could not open file '" << OUTPUT_FILE << "'" << endl;
		return 1;
	}
	
	const osgIntrospection::Type &ReferencedNodeType = osgIntrospection::Reflection::getType("ReferencedNode");
	
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
	
	output << "<p>The SPIN Framework is designed so multiple processes can "
		   << "share state over a network via OpenSoundControl (OSC) messages. "
		   << "By default, these messages are sent using multicast UDP to the "
		   << "address of 224.0.0.1, which is known as the the all-hosts group. "
		   << "Most network interfaces are already members of this multicast "
		   << "group, so a typical client application need only to start a UDP "
		   << "listener on the appropriate port to discover messages related to "
		   << "SPIN.<p>\n";
		   
	output << "<p>The spinServer multicasts all "
		   << "state information in a specific OSC format. Furthermore, if a "
		   << "client wishes to update or modify state on the server, it must "
		   << "send a properly formatted message. In most cases, the message "
		   << "will be multicasted back out once it has been processed. This "
		   << "allows several clients to maintain a synchronous state with the "
		   << "server, while operating in a distributed fashion.</p>\n";
	
	output << "<p>Below is the complete list of accepted OSC messages for the "
		   << "following nodes:</p>\n";
		   
	const osgIntrospection::TypeMap &allTypes = osgIntrospection::Reflection::getTypes();
	osgIntrospection::TypeMap::const_iterator it;
	
	output << "<ul>\n";
	for (it=allTypes.begin(); it!=allTypes.end(); it++)
	{
		const osgIntrospection::Type *classType = ((*it).second);
		
		if (classType->isDefined())
		{
		
			if ( classType->isSubclassOf(ReferencedNodeType) )
			{
				output << "  <li><a href='#" << classType->getName() << "'>" << classType->getName() << "</a></li>\n";
				
			}
		}
	}
	output << "</ul>\n";
	
	
	for (it=allTypes.begin(); it!=allTypes.end(); it++)
	{
		const osgIntrospection::Type *classType = ((*it).second);
		
		if (classType->isDefined())
		{
		
			if ( classType->isSubclassOf(ReferencedNodeType) )
			{

				output << "<hr\n";
				output << "<h2><a class='anchor' name='" << classType->getName() << "'>" << classType->getName() << ":</a></h2>\n";
				output << "<blockquote>\n\n";

				
		        // First go through all base classes and generateHTML:
				if (DEBUG) cout << endl << classType->getQualifiedName() << " has " << classType->getNumBaseTypes() << " base classes:" << endl;
		        for (int i=0; i<classType->getNumBaseTypes(); i++)
		        {
		            const Type &BaseClassType = classType->getBaseType(i);
		            
		            if ((BaseClassType==ReferencedNodeType) || (BaseClassType.isSubclassOf(ReferencedNodeType)))
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
	
	output.close();
	
	cout << "Created file: '" << OUTPUT_FILE << "'" << endl;
	
	return 0;
	
} // main()