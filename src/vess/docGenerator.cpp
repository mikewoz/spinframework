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


#define OUTPUT_FILE "OSCprotocol.tmp"


int main()
{
	//std::ostringstream output("");
	
	ofstream output(OUTPUT_FILE);
	if (!output.is_open())
	{
		cout << "Error: Could not open file '" << OUTPUT_FILE << "'" << endl;
		return 1;
	}
	
	const osgIntrospection::Type &asReferencedType = osgIntrospection::Reflection::getType("asReferenced");
	
	/*
	vector<string> validParams;
	validParams.push_back("const char *");
	validParams.push_back("char *");
	validParams.push_back("int");
	validParams.push_back("float");
	validParams.push_back("double");
	validParams.push_back("bool");
	*/
	
	/*
	output << "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.01 Transitional//EN\">\n";
	output << "<html><head><meta http-equiv=\"Content-Type\" content=\"text/html;charset=UTF-8\">\n";
	output << "<title>SpIF: SPIN (Spatial Interaction) Framework</title>\n";
	output << "<link href=\"doxygen.css\" rel=\"stylesheet\" type=\"text/css\">\n";
	output << "<link href=\"tabs.css\" rel=\"stylesheet\" type=\"text/css\">\n";
	output << "</head><body>\n";
	output << "<div class=\"navigation\" id=\"top\">\n";
	output << "  <div class=\"tabs\">\n";
	output << "    <ul>\n";
	output << "      <li class=\"current\"><a href=\"index.html\"><span>Main&nbsp;Page</span></a></li>\n";
	output << "      <li><a href=\"annotated.html\"><span>Classes</span></a></li>\n";
	output << "      <li><a href=\"files.html\"><span>Files</span></a></li>\n";
	output << "    </ul>\n";
	output << "  </div>\n";
	output << "</div>\n";
	output << "<div class=\"contents\">\n";
	output << "<h1>OSC Protocol for SPIN Framework</h1>\n";
	output << "<p>\n";
	*/
	
	const osgIntrospection::TypeMap &allTypes = osgIntrospection::Reflection::getTypes();
	for ( osgIntrospection::TypeMap::const_iterator it=allTypes.begin(); it!=allTypes.end(); it++)
	{
		const osgIntrospection::Type *classType = (*it).second;
		
		if (classType->isDefined())
		{
			if ( classType->isSubclassOf(asReferencedType) )
			{
				output << "<h3>" << classType->getName() << "</h3>\n";
				
				output << "<blockquote>\n";

				if (!classType->getBriefHelp().empty())
				{
					output << "<p><i>" << classType->getBriefHelp() << "</i></p>\n";
				}
				
				if (!classType->getDetailedHelp().empty())
				{
					output << "<p><i>" << classType->getDetailedHelp() << "</i></p>\n";
				}
				
				
				
			    // Look through all methods
				//const MethodInfoList& methods = classType->getMethods();
				
				MethodInfoList methods;
				classType->getAllMethods(methods);
							    
			    
			    for (MethodInfoList::const_iterator methodIter=methods.begin(); methodIter!=methods.end(); methodIter++)
			    {
			        const MethodInfo* method = *methodIter;
			        const ParameterInfoList& params = method->getParameters();
			
			        // We look for all void methods which take at least 1 argument. However,
			        // we need to ignore virtual methods, since they are catergorized in the
			        // base class property set.
			
			        if ( !method->isVirtual() && method->getReturnType().isVoid() && params.size() )
			        {
			        	ParameterInfoList::const_iterator paramIter;
			
			        	// make sure all parameters are defined:
			        	bool goodParams = true;
			        	for (paramIter=params.begin(); paramIter!=params.end(); paramIter++)
			        	{
			        		if (!(*paramIter)->getParameterType().isDefined())
			        		{
			        			goodParams = false;
			        			break;
			        		}
			        	}
			        	
			        	if (!goodParams) break;
			        	
			        	output << "<p>\n";
			        	output << "<code>\n";
			        	output << "  /vess/{vessID}/{nodeID} <strong>" << method->getName() << "</strong>";
			
			            // add all param names
			        	for (paramIter=params.begin(); paramIter!=params.end(); paramIter++)
			        	{
			        		const ParameterInfo* param = *paramIter;
			        		
			        		output << "  <";

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
		            		}
			        		
			        		output << ">";
			        	}
			        	
			        	output << "\n</code>\n";
			        	

			            if (!method->getBriefHelp().empty())
			            {
			            	output << "<blockquote>\n";
			            	output << "" << method->getBriefHelp() << "<br>\n";
			            	output << "</blockquote>\n";
			            }
			            
			            /*
			            if (!method->getDetailedHelp().empty())
			            {
			            	output << "<blockquote>\n";
			            	output << "Details: " << method->getDetailedHelp() << "<br>\n";
			            	output << "</blockquote>\n";
			            }
			            */
			            
			            output << "</p>\n";
			            
			            
			        } // if desired method
			
			    } // method iterator
			    
				output << "</blockquote>\n";

			} // if asReferenced
		} // if class is defined
	} // type iter
	
	
	//cout << output.str() << endl;
	
	output.close();
	
	cout << "Created file: '" << OUTPUT_FILE << "'" << endl;
	
	return 0;
	
} // main()