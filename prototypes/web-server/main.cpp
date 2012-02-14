// HTTP service into a Boost/C++ program.
 
#include <iostream>
#include <pion/net/HTTPServer.hpp>
#include <pion/net/HTTPTypes.hpp>
#include <pion/net/HTTPRequest.hpp>
#include <pion/net/HTTPResponseWriter.hpp>
 
void handleRootURI(pion::net::HTTPRequestPtr& http_request, pion::net::TCPConnectionPtr& tcp_conn)
{
    static const std::string html_start("<html><body>\n");
    static const std::string html_end("</body></html>\n");
 
    pion::net::HTTPResponseWriterPtr writer(
        pion::net::HTTPResponseWriter::create(tcp_conn, *http_request,
            boost::bind(&pion::net::TCPConnection::finish, tcp_conn)));
    pion::net::HTTPResponse& r = writer->getResponse();
    r.setStatusCode(pion::net::HTTPTypes::RESPONSE_CODE_OK);
    r.setStatusMessage(pion::net::HTTPTypes::RESPONSE_MESSAGE_OK);
    pion::net::HTTPTypes::QueryParams& params = http_request->getQueryParams();
 
    writer->writeNoCopy(html_start);
    writer->write("The query about resource ");
    writer->write(http_request->getResource());
 
    if (params.size() > 0)
    {
        writer->write(" has the following parameters: <br>");
        for (pion::net::HTTPTypes::QueryParams::const_iterator i = params.begin();
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
    writer->writeNoCopy(html_end);
    writer->send();
}
 
int main(int argc, char *argv[])
{
    static const unsigned int DEFAULT_PORT = 8080;
    try
    {
        pion::net::HTTPServerPtr hello_server(new pion::net::HTTPServer(DEFAULT_PORT));
        hello_server->addResource("/", &handleRootURI);
        hello_server->start(); // Spawn a thead to run the HTTP
                               // service, and the main thread can
                               // focus on the main work now.
        std::cout << "Hello, the server is running now ...\n";
        while (true)
            sleep(1);
    }
    catch (std::exception& e)
    {
        std::cerr << "Failed running the HTTP service due to " <<  e.what();
    }
    return 0;
}

