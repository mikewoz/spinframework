#include <string>
#include <iostream>
#include <boost/filesystem.hpp>

#include <osgViewer/CompositeViewer>
#include <osg/ArgumentParser>
#include <osgDB/ReadFile>
#include <osg/Timer>


#include <spin/SceneManager.h>
#include <spin/spinApp.h>
#include <spin/spinServerContext.h>
#include <spin/spinUtil.h>
#include <spin/spinLog.h>
#include <spin/ReferencedNode.h>
#include <spin/config.h>

#include <spin/introspect_helpers.h>
#include <cppintrospection/Type>
#include <cppintrospection/Value>

#include <lo/lo_lowlevel.h>
#include <lo/lo.h>


#include "Poco/Net/HTTPServer.h"
#include "Poco/Net/HTTPRequestHandler.h"
#include "Poco/Net/HTTPRequestHandlerFactory.h"
#include "Poco/Net/HTTPServerParams.h"
#include "Poco/Net/HTTPServerRequest.h"
#include "Poco/Net/HTTPServerResponse.h"
#include "Poco/Net/HTTPServerParams.h"
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

/*
cppintrospection::MethodInfo introspect_getMethodInfo(const cppintrospection::Type *t)
{
    using namespace cppintrospection;

    const MethodInfoList &mil = type.getMethods();
    if (!mil.empty())
    {
        for (MethodInfoList::const_iterator j=mil.begin(); j!=mil.end(); ++j)
        {
            // get the MethodInfo object that describes the current
            // method
            const MethodInfo &mi = **j;


            introspect_print_method(mi);
        }
    }

}
*/

bool introspect_invoke(std::string path, const Poco::Net::HTMLForm &form)
{
    using namespace spin;

    std::cout << "introspect_invoke for path: " << path << std::endl;
    

    // WARNING: this method is meant to respond to ANY path, so we must manually
    // check if it is within the /SPIN namespace, and if it matches the sceneID:

    std::string spinToken, sceneString, nodeString;
    std::istringstream pathstream(path);
    pathstream.get(); // ignore leading slash
    getline(pathstream, spinToken, '/');
    getline(pathstream, sceneString, '/');
    getline(pathstream, nodeString, '/');


    if ((spinToken!="SPIN") || !wildcardMatch(sceneString.c_str(),spinApp::Instance().getSceneID().c_str()) )
    {
    	std::cout << "Warning: server is ignoring HTTP message: " << path << std::endl;
    	return 0;
    }


    // Now, we look for the spin.call variable, which will contain the method 
    // and all arguments in one string. We need to convert that to an argv array
    // so that we can use the existing sceneCallback and nodeCallback methods

    std::vector<std::string> methodAndArgs = tokenize(form["spin.call"]);

    if (methodAndArgs.size())
    {
        lo_message msg = lo_message_new();
        
        for (int index=0; index<methodAndArgs.size(); index++)
        {
            int i;
            float f;
            if (fromString<int>(i, methodAndArgs[index]))
            {
                std::cout << "converted " << methodAndArgs[index] << " to int" << std::endl;
                lo_message_add(msg, "i", i);
                typeString += "i";
            }
            else if (fromString<float>(f, methodAndArgs[index]))
            {
                std::cout << "converted " << methodAndArgs[index] << " to float" << std::endl;
                lo_message_add(msg, "f", f);
            }
            else
            {
                std::cout << "converted " << methodAndArgs[index] << " to string" << std::endl;
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
            
            std::vector<t_symbol*> matched;
            std::vector<t_symbol*>::iterator iter;
            
            // first nodes:
            matched = spinApp::Instance().sceneManager_->findNodes(nodeString.c_str());
            for (iter = matched.begin(); iter != matched.end(); ++iter)
            {
                spinApp::Instance().NodeMessage((*iter)->s_name, msg);
            }

            // now statesets:
            matched = spinApp::Instance().sceneManager_->findStateSets(nodeString.c_str());
            for (iter = matched.begin(); iter != matched.end(); ++iter)
            {
                spinApp::Instance().NodeMessage((*iter)->s_name, msg);
            }
        }
    }

    return true;
}

bool introspect_invoke_old(std::string path, const Poco::Net::HTMLForm &form)
{
    using namespace spin;
    
    // vars: request.getMethod(), form
    
    
    t_symbol *s = gensym(form["nodeID"].c_str());
    //ReferencedNode *n = dynamic_cast<ReferencedNode*>(gensym(form["nodeID"].c_str())->s_thing);
    std::string method = form["method"];
    
    
    
    if (s->s_thing)
    {
    
        cppintrospection::ValueList theArgs;
        cppintrospection::Value classInstance;
        if (s->s_type == REFERENCED_STATESET)
        {
            classInstance = cppintrospection::Value(dynamic_cast<ReferencedStateSet*>(s->s_thing));
        }
        else
        {
            classInstance = cppintrospection::Value(dynamic_cast<ReferencedNode*>(s->s_thing));
        }

        // the getInstanceType() method however, gives us the real type being pointed at:
        const cppintrospection::Type &classType = classInstance.getInstanceType();

        if (classType.isDefined())
        {
            std::cout << "introspection for: " << path << std::endl;
            introspect_print_type(classType);
        
            // TODO: update server state:
            Poco::Net::NameValueCollection::ConstIterator it;
            for (it = form.begin(); it != form.end(); ++it)
            {
                if ((it->first!="nodeID")&&(it->first!="method"))
                {
                    theArgs.push_back(it->second);
                }
            }
            
            // call method on class:
            // (the true means that it will try base classes as well)
            classType.invokeMethod(method, classInstance, theArgs, true);            
            
            // debug:
            cppintrospection::ValueList debugArgs;
            //debugArgs.push_back("debug");
            cppintrospection::Value res = classType.invokeMethod("debug", classInstance, debugArgs, true);
            
            if (res.isEmpty() || res.isNullPointer())
            {
                std::cout << "error calling method" << std::endl;
            }
            else
            {
                std::cout << "success calling method" << std::endl;
                return true;
            }
        }
        else
        {
            std::cout << "ERROR: cound not process message '" << path << ". cppintrospection has no data for that node." << std::endl;
            return false;
        }
    }
    else
    {
        std::cout << "spinserver has no element at path: " << path << std::endl;
        return false;
    }


    return true;
}



class MyPartHandler: public Poco::Net::PartHandler
{
public:
	MyPartHandler():
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
	/// Return a HTML document with the current date and time.
{
public:
	FormRequestHandler() 
	{
	}
	
	void handleRequest(Poco::Net::HTTPServerRequest& request, Poco::Net::HTTPServerResponse& response)
	{
		//Poco::Util::Application& app = Poco::Util::Application::instance();
		//app.logger().information("Request from " + request.clientAddress().toString());
        std::cout << "Request from " << request.clientAddress().toString() << std::endl;

		MyPartHandler partHandler;
		Poco::Net::HTMLForm form(request, request.stream(), partHandler);

		response.setChunkedTransferEncoding(true);
		response.setContentType("text/html");

		std::ostream& ostr = response.send();
		
		ostr <<
			"<html>\n"
			"<head>\n"
			"<title>SPIN Web Server Sample</title>\n"
			"</head>\n"
			"<body>\n"
			"<h1>SPIN Web Server Sample</h1>\n"
			"<h2>Tests:</h2>\n"
			"<form name=\"spinform\" method=\"GET\" action=\"null\">\n"
			"/SPIN/default/"
            "<input type=\"text\" name=\"nodeID\" value=\"shp\" size=\"15\">"
            "&nbsp;&nbsp;&nbsp;"
            "<input type=\"text\" name=\"method\" value=\"rotate\" size=\"15\">"
            " move<input type=\"text\" name=\"x\" value=\"0\" size=\"3\">"
            " <input type=\"text\" name=\"y\" value=\"0\" size=\"3\">"
            " <input type=\"text\" name=\"z\" value=\"10\" size=\"3\">\n"
			" <input type=\"submit\" value=\"GO\" onclick=\"this.form.action='/SPIN/default/'+this.form.nodeID.value\">\n"
			"</form>\n"
            "\n";
        /*    
        ostr <<
			"<html>\n"
			"<head>\n"
			"<title>SPIN Web Server Sample</title>\n"
			"</head>\n"
			"<body>\n"
			"<h1>SPIN Web Server Sample</h1>\n"
			"<h2>GET Form</h2>\n"
			"<form method=\"GET\" action=\"/form\">\n"
			"<input type=\"text\" name=\"text\" size=\"31\">\n"
			"<input type=\"submit\" value=\"GET\">\n"
			"</form>\n"
			"<h2>POST Form</h2>\n"
			"<form method=\"POST\" action=\"/form\">\n"
			"<input type=\"text\" name=\"text\" size=\"31\">\n"
			"<input type=\"submit\" value=\"POST\">\n"
			"</form>\n"
			"<h2>File Upload</h2>\n"
			"<form method=\"POST\" action=\"/form\" enctype=\"multipart/form-data\">\n"
			"<input type=\"file\" name=\"file\" size=\"31\"> \n"
			"<input type=\"submit\" value=\"Upload\">\n"
			"</form>\n";
            */
			
		ostr << "<h2>Request</h2><p>\n";
		ostr << "Method: " << request.getMethod() << "<br>\n";
		ostr << "URI: " << request.getURI() << "<br>\n";
		Poco::Net::NameValueCollection::ConstIterator it = request.begin();
		Poco::Net::NameValueCollection::ConstIterator end = request.end();
		for (; it != end; ++it)
		{
			ostr << it->first << ": " << it->second << "<br>\n";
		}
		ostr << "</p>";

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
        
        // --------parse
        
        std::cout << "checking nodeID" << form["nodeID"] << std::endl;
        
        introspect_invoke(request.getURI(), form);
        
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
};


class FormRequestHandlerFactory: public Poco::Net::HTTPRequestHandlerFactory
{
public:
	FormRequestHandlerFactory()
	{
	}

	Poco::Net::HTTPRequestHandler* createRequestHandler(const Poco::Net::HTTPServerRequest& request)
	{
		return new FormRequestHandler;
	}
};









// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
int main(int argc, char **argv)
{
	using namespace spin;
	spinServerContext server;
    
	// *************************************************************************
    // If no command line arguments were passed, check if there is an args file
    // at ~/.spinFramework/args and override argc and argv with those:
    std::vector<char*> newArgs = spin::getUserArgs();
    if ((argc==1) && (newArgs.size() > 1))
    {
        newArgs.insert(newArgs.begin(), argv[0]);
        argc = (int)newArgs.size()-1;
        argv = &newArgs[0];
    }
    
	// *************************************************************************
	// PARSE ARGUMENTS::

	osg::ArgumentParser arguments(&argc,argv);
	
    // set up the usage document, which a user can acess with -h or --help
	arguments.getApplicationUsage()->setDescription(arguments.getApplicationName()+" is a server for the SPIN Framework.");
	arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options] <scene-to-load.xml>");
    
    // add generic spin arguments (for address/port setup, scene-id, etc) 
    server.addCommandLineOptions(&arguments);

    // arguments specific to spinserver:
    arguments.getApplicationUsage()->addCommandLineOption("--disable-discovery", "Disable multicast discovery services.");

	// PARSE ARGS:

    // parse generic args:
    if (!server.parseCommandLineOptions(&arguments))
        return 0;

	// any option left unread are converted into errors to write out later
	arguments.reportRemainingOptionsAsUnrecognized();

	
	// *************************************************************************
	// remaining arguments are assumed to be scene elements to be loaded (.xml)
	
	std::vector<std::string>::iterator arg;
	for (int i=1; i<argc; i++) //arg=arguments.begin(); arg!=arguments.end(); ++arg)
	{
		if (arguments[i][0]!='-')
		{
			// not an option so assume string is a filename
			spinApp::Instance().sceneManager->loadXML(arguments[i]);
		} else i++;
	}
	 	
	// any option left unread are converted into errors
	//arguments.reportRemainingOptionsAsUnrecognized();
	
	if (arguments.errors())
	{
		arguments.writeErrorMessages(std::cout);
		return 1;
	}

	// *************************************************************************
	// start spin:
	server.start();
    
    
    // start webserver:
    
    unsigned short port = 9980;
			
    // set-up a server socket
	Poco::Net::ServerSocket svs(port);
	// set-up a HTTPServer instance
	Poco::Net::HTTPServer pocoServer(new FormRequestHandlerFactory, svs, new Poco::Net::HTTPServerParams);
	// start the HTTPServer
	pocoServer.start();


	// *************************************************************************
	// send a userRefresh message:
    SCENE_MSG("s", "userRefresh");
	
	// *************************************************************************
	// loop:
    std::cout << "\nspinserver is READY" << std::endl;
    try {	
        while (server.isRunning())
        {
            sleep(1);
            // loop until a quit message is received (TODO)
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Got exception " << e.what() << std::endl;
        return 1;
    }

    // stop webserver:
	pocoServer.stop();

    usleep(100);
    std::cout << "spinserver exited normally." << std::endl;

    return 0;
}
