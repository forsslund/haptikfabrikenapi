#include "webserv.h"


//Added for the json-example
#define BOOST_SPIRIT_THREADSAFE
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

//Added for the default_resource example
#include<fstream>
#include <functional>
#include <iostream>

using namespace std;
//Added for the json-example:
using namespace boost::property_tree;





/// Add this node to the H3DNodeDatabase system.
Webserv::Webserv(){
    enc5=0;
    cout << "Webserv Constructing"<<endl;
}

Webserv::~Webserv()
{
    cout<<"Webserv Stopping Server"<<endl;
    server->stop();
    if(webservThread.joinable()){
        webservThread.join();
    }
    delete server; // Works but theoretically undefined behaviour. New ASIO lib may fix.
    cout<<"Webserv Node Destructed"<<endl;
}


void Webserv::jonas_reply(HttpServer::Response& response,
                          shared_ptr<HttpServer::Request> request)
{
  string path=request->path;
  /*
  if(path=="/head/off")
    showHead->setValue(1);
  if(path=="/head/on")
    showHead->setValue(0);
  if(path=="/system/exit"){
    std::cout << "**********************" << std::endl
              << "Exiting application..." << std::endl;
    throw Exception::QuitAPI();
  }
  if(path=="/playback/play")
    doPlayback->setValue(true);
  if(path=="/playback/pause")
    doPlayback->setValue(false);
  if(path=="/save")
    saveVolume->setValue(true);
  if(path.find("/teeth/rotation/") != string::npos){
    string deg = path.substr(16,string::npos);
    double rad = atoi(deg.c_str()) * 3.1415/180.0;
    teethRotation->setValue(Rotation(0,1,0,float(rad)));

  }*/

  // Get encoder
  //enc5 = 0;
  if(path.find("/setencoder/5/") != string::npos){
    string s = path.substr(14,string::npos);
    message_mutex.lock();
    enc5 = atoi(s.c_str());
    message_mutex.unlock();

    // Clock our last message
    lastEnc5message = chrono::high_resolution_clock::now();
  }

  message_mutex.lock();
  string latest_message = message;
  message_mutex.unlock();



    stringstream content;
    content << "{";


    content << "\"request_path\": " << "\"" << request->path << "\"" << ",\n";

    content << "\"device_status\": " << "{\n" << latest_message << "},\n";

    content << "\"fps\": " << 123.23 << ",\n";

    /*
    // Materials list
    // NOTE: id is confusingly set to i+1 here. That is because the GUI
    //       thought that was how we indexed them. "Air" is then id 1.
    //       should be reverted to indexed from 0 in the future.
    content << "\"materials\": [ ";
    const vector<string>& segmentNames = segmentNameField->getValue();
    const vector<int>& val = noOfVoxelsBoredByUser->getValue();
    for(int i=0;i<segmentNames.size();++i){
        content << "{" <<
                             "\"id\": " << i+1 << "," <<
                             "\"name\": \"" << segmentNames[i] << "\"," <<
                             "\"removed_voxels\": { \"total\": " <<  val[i] << "}"<<
                          "}";
        if(i!=segmentNames.size()-1)
            content << ",";
    }
    content << "],";

    // Forbidden list
    content << "\"forbidden\": [ ";
    const vector<string>& fNames = forbidden_segmentNameField->getValue();
    const vector<int>& fVal = forbidden_noOfVoxelsBoredByUser->getValue();
    for(int i=0;i<fNames.size();++i){
        content << "{" <<
                     "\"id\": " << i+1 << "," <<
                     "\"name\": \"" << fNames[i] << "\"," <<
                     "\"removed_voxels\": { \"forbidden\": " << fVal[i] << "}"<<
                   "}";
        if(i!=fNames.size()-1)
            content << ",";
    }
    content << "],";

    content << "\"current_state\": " << state->getValue() << ",";
    content << "\"fraction_user_expert_current_step\": " <<
               fractionUserExpertCurrentStep->getValue() << ",";
    content << "\"playback_is_play\": " << (playback_isPlay->getValue()? 1:0) << ",";
    content << "\"playback_time\": " << playback_time->getValue() << ",";
    content << "\"showHead\": " << showHead->getValue() << ",";
    content << textout->getValue(); // format:    "key: value,"
    */
    content << "\"build_date\": \"" << std::string(__DATE__) << " " <<
                                       std::string(__TIME__) << "\"";

    content << "}";


    //find length of content_stream (length received using content_stream.tellp())
    content.seekp(0, ios::end);

    response <<  "HTTP/1.1 200 OK\r\n"<<
                 "Content-Length: " << content.tellp() << "\r\n" <<
                 "Access-Control-Allow-Origin: *\r\n" <<
                 "\r\n" << content.rdbuf();


}

void Webserv::setMessage(string s)
{
    message_mutex.lock();
    message = s;
    message_mutex.unlock();
}

int Webserv::getEnc5()
{
    int r;
    message_mutex.lock();
    r = enc5;
    message_mutex.unlock();
    return r;
}

bool Webserv::activeEnc5()
{
    using namespace chrono;
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - lastEnc5message);

    // True if we got a message within the last second
    return time_span.count() < 1;
}

void Webserv::initialize(int port)
{
    cout<<"Webserv Initializing"<<endl;


    //HTTP-server at port 8088 using 2 threads
    server = new HttpServer(port, 2);

    using namespace std::placeholders;

    std::function<void(HttpServer::Response&,
                       std::shared_ptr<HttpServer::Request>)> f =
        std::bind(&Webserv::jonas_reply, this, placeholders::_1,
                                               placeholders::_2);

    //server->resource["^/$"]["GET"] = f;
    server->default_resource["GET"] = f;

    webservThread = std::thread(&Webserv::start,this);
    cout<<"Webserv Node Initialized"<<endl;

    return;
}

