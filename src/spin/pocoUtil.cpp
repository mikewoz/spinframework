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

#include <string>
#include <iostream>

#include "pocoutil.h"

#ifdef WITH_POCO

#include "spinapp.h"
#include "scenemanager.h"


namespace spin
{

bool applyHTTPMessage(std::string path, const Poco::Net::HTMLForm &form)
{
    
    // WARNING: this method is meant to respond to ANY path, so we must manually
    // check if it is within the /SPIN namespace, and if it matches the sceneID:
    
    std::string spinToken, sceneString, nodeString;
    std::istringstream pathstream(path);
    pathstream.get(); // ignore leading slash
    getline(pathstream, spinToken, '/');
    getline(pathstream, sceneString, '/');
    getline(pathstream, nodeString, '?');
    
    /*
    std::cout << "introspect_invoke for path: " << path << std::endl;
    std::cout << "spinToken?:  " << spinToken << std::endl;
    std::cout << "sceneString: " << sceneString << std::endl;
    std::cout << "nodeString:  " << nodeString << std::endl;
    std::cout << "args:        " << form["args"] << std::endl;
    */
    
    if ((spinToken!="SPIN") || !wildcardMatch(sceneString.c_str(),spinApp::Instance().getSceneID().c_str()) )
    {
        std::cout << "Warning: server is ignoring HTTP message: " << path << std::endl;
        return 0;
    }
    
    
    // Now, we look for the args variable, which will contain the method 
    // and all arguments in one string. We need to convert that to an argv array
    // so that we can use the existing sceneCallback and nodeCallback methods
    
    std::vector<std::string> methodAndArgs = tokenize(form["args"]);
    std::string typeString = "";
    if (methodAndArgs.size())
    {
        lo_message msg = lo_message_new();
        
        for (int index=0; index<methodAndArgs.size(); index++)
        {
            float f;
            if (fromString<float>(f, methodAndArgs[index]))
            {
                lo_message_add(msg, "f", f);
            }
            else
            {
                lo_message_add(msg, "s", methodAndArgs[index].c_str());
            }
        }
        
        
        // if the nodeString is empty, then this is a scene message
        if (nodeString.empty())
        {
            spinApp::Instance().SceneMessage(msg);
        }
        else
        {
            // The nodeString might have a wildcard, so here we call the method on
            // any nodes (or statesets) that match:
            
            
            // first nodes:
            std::vector<ReferencedNode*> matchedNodes;
            std::vector<ReferencedNode*>::iterator iter;
            matchedNodes = spinApp::Instance().sceneManager_->findNodes(nodeString.c_str());
            for (iter = matchedNodes.begin(); iter != matchedNodes.end(); ++iter)
            {
                spinApp::Instance().NodeMessage((*iter)->getID().c_str(), msg);
            }
            
            // now statesets:
            std::vector<ReferencedStateSet*> matchedStateSets;
            std::vector<ReferencedStateSet*>::iterator sIter;
            
            matchedStateSets = spinApp::Instance().sceneManager_->findStateSets(nodeString.c_str());
            for (sIter = matchedStateSets.begin(); sIter != matchedStateSets.end(); ++sIter)
            {
                spinApp::Instance().NodeMessage((*sIter)->getID().c_str(), msg);
            }
        }
    }
    
    return true;
}

    
void FormRequestHandler::handleRequest(Poco::Net::HTTPServerRequest& request, Poco::Net::HTTPServerResponse& response)
{
    //std::cout << "Request from " << request.clientAddress().toString() << std::endl;
    
    PartHandler partHandler;
    Poco::Net::HTMLForm form(request, request.stream(), partHandler);
    
    std::string spinToken, sceneString, nodeString, args;
    std::istringstream pathstream(request.getURI());
    pathstream.get(); // ignore leading slash
    getline(pathstream, spinToken, '/');
    getline(pathstream, sceneString, '/');
    getline(pathstream, nodeString, '?');
    
    if (sceneString.empty()) sceneString = "default";
    //if (nodeString.empty()) nodeString = "shp";
    if (form.empty()) args = "createNode shp ShapeNode";
    else args = form["args"];
    
    response.setChunkedTransferEncoding(true);
    response.setContentType("text/html");
    
    std::ostream& ostr = response.send();
    
    
    ostr <<
    "<html>\n"
    "<head>\n"
    "<title>SPIN Web Service</title>\n"
    "</head>\n"
    "<body>\n"
    "<h1>SPIN Web Service</h1>\n"
    "<h3>Enter a SPIN command in the form below:</h3>\n"
    "<table><tr><td nowrap=\"nowrap\">\n"
    "<form name=\"urlForm\" method=\"GET\" action=\"null\">\n"
    "/SPIN/"
    "<input type=\"text\" name=\"sceneID\" value=\"" << sceneString << "\" size=\"10\">"
    "/"
    "<input type=\"text\" name=\"nodeID\" value=\"" << nodeString << "\" size=\"10\">"
    "</form></td>\n"
    "<td nowrap=\"nowrap\">\n"
    "<form name=\"spinform\" method=\"POST\" action=\"null\">\n"
    "<input type=\"text\" name=\"args\" value=\"" << args << "\" size=\"20\">\n"
    "<input type=\"submit\" value=\"GO\" onclick=\"this.form.action='/SPIN/'+document.forms['urlForm']['sceneID'].value+'/'+document.forms['urlForm']['nodeID'].value\">\n"
    "</form>\n"
    "</tr></table>\n"
    "<p>(NOTE: you can send scene messages by leaving the node name blank)</p>\n"
    "\n";
    
    ostr << "<h2>Result</h2><p>\n";
    ostr << "Method: " << request.getMethod() << "<br>\n";
    ostr << "URI: " << request.getURI() << "<br>\n";
    Poco::Net::NameValueCollection::ConstIterator it = request.begin();
    Poco::Net::NameValueCollection::ConstIterator end = request.end();
    for (; it != end; ++it)
    {
        ostr << it->first << ": " << it->second << "<br>\n";
    }
    ostr << "</p>";
    /*
     if (!form.empty())
     {
     ostr << "<h2>Result</h2><p>\n";
     it = form.begin();
     end = form.end();
     for (; it != end; ++it)
     {
     ostr << it->first << ": " << it->second << "<br>\n";
     }
     ostr << "</p>";
     }
     */
    
    // --------parse
    if (!form.empty())
    {
        applyHTTPMessage(request.getURI(), form);
    }
    
    // ---------------
    
    if (!partHandler.name().empty())
    {
        ostr << "<h2>Upload</h2><p>\n";
        ostr << "Name: " << partHandler.name() << "<br>\n";
        ostr << "File Name: " << partHandler.fileName() << "<br>\n";
        ostr << "Type: " << partHandler.contentType() << "<br>\n";
        ostr << "Size: " << partHandler.length() << "<br>\n";
        ostr << "</p>";
    }
    ostr << "</body>\n";

}

} // end of namespace spin    
    
#endif // ifdef WITH_POCO
