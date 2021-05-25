#pragma once
#ifdef _WIN32 // Defined by MSVC compiler for both 32 and 64 bit platforms
// Prevents <Windows.h> from #including <Winsock.h>, as we use <Winsock2.h> instead.
#ifndef _WINSOCKAPI_
#define DID_DEFINE_WINSOCKAPI
#define _WINSOCKAPI_
#endif
#pragma comment(lib, "ws2_32.lib")

#include <winsock2.h>
#include <afunix.h>
#else

// Linux
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <errno.h>

#define SOCKET int
int closesocket(SOCKET s){
    return close(s);
}


void Sleep(int ms){
    usleep(ms * 1000);
}


/*
* Below stolen from WinSock2:
* This is used instead of -1, since the
* SOCKET type is unsigned.
*/
#define INVALID_SOCKET  (SOCKET)(~0)
#define SOCKET_ERROR            (-1)
#endif


#include <list>
#include <thread>
#include <mutex>

template <class T> 
class SocketClient {
public:
	void Start(std::string socketPath);

	SocketClient() : socketPath(""), listenThread() //Default threadummy
	{
		// Initialize Winsock
#ifdef _WIN32
		int iResult;
		iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
		if (iResult != 0) {
			printf("WSAStartup failed: %d\n", iResult);
		}
#endif
	}
	~SocketClient() {
		stopClient = true;
		if (listenSocket != INVALID_SOCKET) closesocket(listenSocket);
		if (listenThread.joinable()) listenThread.join();
	}
	
	void Shutdown() { 
		stopClient = true;
	}	
	T Get() { 
		const std::lock_guard<std::mutex> lock(dataMutex);
		return data; 
	}

private:
	void Listen();
	bool isRunning = false;
#ifdef _WIN32
    WSADATA wsaData{0};
#endif
	volatile bool stopClient = false;	
	bool connected = false;
	T data = { 0 };
	std::mutex dataMutex;	
	std::thread listenThread;
	std::string socketPath;
	SOCKET listenSocket = INVALID_SOCKET;
};


///////////////////// SocketClient.cpp ////////
#ifdef _WIN64
using ssize_t = __int64;
#endif

template <class T>
void SocketClient<T>::Start(std::string socketPath_) {
	// Fire up a thread to take care of incomming connections. But first check that thread is not running (i.e. not joinable)
	if (!listenThread.joinable()) {
		socketPath = socketPath_;
		listenThread = std::thread(&SocketClient<T>::Listen, this);
	}
		
}

template <class T>
void SocketClient<T>::Listen() {
        printf("\nIn SocketClient.h listening thread\n");

	struct sockaddr_un addr;
	int dataSize = sizeof(data);
	char* dataBuffer = new char[dataSize];
	
	while (!stopClient) {
		// Try setting up a connection
		printf( "Attempting to connect to %s\n", socketPath.c_str() );
		listenSocket = socket(AF_UNIX, SOCK_STREAM, 0);
		if (listenSocket != INVALID_SOCKET) {
			connected = true;

			memset(&addr, 0, sizeof(struct sockaddr_un));
			addr.sun_family = AF_UNIX;
            strncpy(addr.sun_path, socketPath.c_str(), sizeof(addr.sun_path) - 1);
			int res = connect(listenSocket, (struct sockaddr*)&addr, sizeof(struct sockaddr_un));
            if (res==-1){
                int errsv = errno;
                printf("%s\n", strerror(errsv));
            }
            if (res >= 0) {
				printf("Connected to server.\n");
				while (!stopClient && connected) {					
					// Block untill activity on socket
					fd_set set;
					FD_ZERO(&set);
					FD_SET(listenSocket, &set);
					struct timeval timeout;			
					timeout.tv_sec = 2;
					timeout.tv_usec = 0;
					int activity = select(listenSocket+1, &set, NULL, NULL, &timeout);		// Any activity on socket?
					if(activity>0){	 // Not timeout, not error
						// Recive until the whole datapackage is filled
						int byteCounter = 0;
						while (byteCounter < dataSize && !stopClient) {
							int res = recvfrom(listenSocket, &dataBuffer[byteCounter], dataSize - byteCounter, 0, NULL, NULL);
							if (res <= 0) { // Since there was activity, we should have data, if no data or error code, connection was lost
								connected = false;
								break;
							}
							else {
								byteCounter += res;
							}
						}

						if (byteCounter == dataSize) {
							// Atomically store data
							const std::lock_guard<std::mutex> lock(dataMutex);
							memcpy((void*)&data, dataBuffer, sizeof(data));
						}
					}
					else if(activity<0){
						connected = false;
					}
				}
			}
			Sleep(10);
			shutdown(listenSocket, 0);
			closesocket(listenSocket);
			listenSocket = INVALID_SOCKET;
			connected = false;
		}
		Sleep(250); // Wait before trying again
	}
#ifdef _WIN32
	WSACleanup();
#endif
	delete [] dataBuffer;
	return;
}
