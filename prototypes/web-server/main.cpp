// HTTP service into a Boost/C++ program.
 
#include <iostream>
#include <pion/net/HTTPServer.hpp>
#include <pion/net/HTTPTypes.hpp>
#include <pion/net/HTTPRequest.hpp>
#include <pion/net/HTTPResponseWriter.hpp>
 
using namespace std;
using namespace pion;
using namespace pion::net;
 
void handleRootURI(HTTPRequestPtr& http_request, TCPConnectionPtr& tcp_conn)
{
    static const std::string kHTMLStart("<html><body>\n");
    static const std::string kHTMLEnd("</body></html>\n");
 
    HTTPResponseWriterPtr writer(
        HTTPResponseWriter::create(
            tcp_conn,
            *http_request,
            boost::bind(
                &TCPConnection::finish,
                tcp_conn)));
    HTTPResponse& r = writer->getResponse();
    r.setStatusCode(HTTPTypes::RESPONSE_CODE_OK);
    r.setStatusMessage(HTTPTypes::RESPONSE_MESSAGE_OK);
    HTTPTypes::QueryParams& params = http_request->getQueryParams();
 
    writer->writeNoCopy(kHTMLStart);
    writer->write("The query about resource ");
    writer->write(http_request->getResource());
 
    if (params.size() > 0)
    {
        writer->write(" has the following parameters: <br>");
        for (HTTPTypes::QueryParams::const_iterator i = params.begin();
             i != params.end(); ++i)
        {
            writer->write(i->first);
            writer->write("=");
            writer->write(i->second);
            writer->write("<br>");
        }
    }
    else
    {
        writer->write(" has no parameter.");
    }
    writer->writeNoCopy(kHTMLEnd);
    writer->send();
}
 
int main(int argc, char *argv[])
{
    static const unsigned int DEFAULT_PORT = 8080;
    try
    {
        HTTPServerPtr hello_server(new HTTPServer(DEFAULT_PORT));
        hello_server->addResource("/", &handleRootURI);
        hello_server->start(); // Spawn a thead to run the HTTP
                               // service, and the main thread can
                               // focus on the main work now.
        std::cout << "Hello, the server is running now ...\n";
        sleep(10);
    }
    catch (std::exception& e)
    {
        std::cerr << "Failed running the HTTP service due to " <<  e.what();
    }
    return 0;
}

