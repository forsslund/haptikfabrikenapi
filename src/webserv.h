#ifndef WEBSERV
#define WEBSERV

#include "server_http.h"
#include "client_http.h"

#include <thread>
#include <mutex>

using namespace std;

typedef SimpleWeb::Server<SimpleWeb::HTTP> HttpServer;
typedef SimpleWeb::Client<SimpleWeb::HTTP> HttpClient;



class Webserv
{
public:
    Webserv();
    virtual ~Webserv();
    void initialize(int port=8088);


    int fd;


    void jonas_reply (HttpServer::Response& response, shared_ptr<HttpServer::Request> request);


    void setMessage(string s);
    int getEnc5();
    bool activeEnc5();

private:
    thread webservThread;
    HttpServer* server;
    void start() { server->start(); }
    int enc5;

    string message;
    std::mutex message_mutex;

    std::chrono::high_resolution_clock::time_point lastEnc5message;

};



#endif // WEBSERV

