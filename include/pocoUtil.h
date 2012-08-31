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
#ifndef __pocoUtil_H
#define __pocoUtil_H

#include "config.h"

#ifdef WITH_POCO

#include "Poco/Net/HTTPServer.h"
#include "Poco/Net/HTTPRequestHandler.h"
#include "Poco/Net/HTTPRequestHandlerFactory.h"
#include "Poco/Net/HTTPServerParams.h"
#include "Poco/Net/HTTPServerRequest.h"
#include "Poco/Net/HTTPServerResponse.h"
#include "Poco/Net/HTMLForm.h"
#include "Poco/Net/PartHandler.h"
#include "Poco/Net/MessageHeader.h"
#include "Poco/Net/ServerSocket.h"
#include "Poco/CountingStream.h"
#include "Poco/NullStream.h"
#include "Poco/StreamCopier.h"
#include "Poco/Exception.h"
#include "Poco/Util/ServerApplication.h"
#include "Poco/Util/Option.h"
#include "Poco/Util/OptionSet.h"
#include "Poco/Util/HelpFormatter.h"

namespace spin
{
  
/**
 * This is the generic handler for all HTTP messages
 */
bool applyHTTPMessage(std::string path, const Poco::Net::HTMLForm &form);
    
class PartHandler: public Poco::Net::PartHandler
{
public:
    PartHandler():
    _length(0)
    {
    }
    
    void handlePart(const Poco::Net::MessageHeader& header, std::istream& stream)
    {
        _type = header.get("Content-Type", "(unspecified)");
        if (header.has("Content-Disposition"))
        {
            std::string disp;
            Poco::Net::NameValueCollection params;
            Poco::Net::MessageHeader::splitParameters(header["Content-Disposition"], disp, params);
            _name = params.get("name", "(unnamed)");
            _fileName = params.get("filename", "(unnamed)");
        }
        
        Poco::CountingInputStream istr(stream);
        Poco::NullOutputStream ostr;
        Poco::StreamCopier::copyStream(istr, ostr);
        _length = istr.chars();
    }
    
    int length() const
    {
        return _length;
    }
    
    const std::string& name() const
    {
        return _name;
    }
    
    const std::string& fileName() const
    {
        return _fileName;
    }
    
    const std::string& contentType() const
    {
        return _type;
    }
    
private:
    int _length;
    std::string _type;
    std::string _name;
    std::string _fileName;
};


class FormRequestHandler: public Poco::Net::HTTPRequestHandler
{
public:
    FormRequestHandler() 
    {
    }
    
    void handleRequest(Poco::Net::HTTPServerRequest& request, Poco::Net::HTTPServerResponse& response);
};
        
class FormRequestHandlerFactory: public Poco::Net::HTTPRequestHandlerFactory
{
public:
    FormRequestHandlerFactory()
    {
    }
    
    Poco::Net::HTTPRequestHandler* createRequestHandler(const Poco::Net::HTTPServerRequest& request)
    {
        return new FormRequestHandler();
    }
};
        
        
} // end of namespace spin

#endif // WITH_POCO

#endif