#include <pthread.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <arpa/inet.h>
#include "tls_client_lib.h"
#include "tls_pthread.h"
#include "netconstants.h"
#include "constants.h"


// Socket
static int sockfd;

// SSL structures
SSL_CTX *ctx;
SSL *ssl;

// Close sockets, free SSL structures and exit program
void closeAndExit(int exitCode)
{
	close(sockfd);
	SSL_free(ssl);
	SSL_CTX_free(ctx);
	thread_cleanup();
	cleanup_openssl();
}
// Tells us that the network is running.
static volatile int networkActive=0;

// Cert verification callback. Implement this in the client
int verify_callback(int preverify, X509_STORE_CTX *x509_ctx)
{
	return preverify;
}

void handleError(const char *buffer)
{
	switch(buffer[1])
	{
		case RESP_OK:
			printf("Command / Status OK\n");
			break;

		case RESP_BAD_PACKET:
			printf("BAD MAGIC NUMBER FROM ARDUINO\n");
			break;

		case RESP_BAD_CHECKSUM:
			printf("BAD CHECKSUM FROM ARDUINO\n");
			break;

		case RESP_BAD_COMMAND:
			printf("PI SENT BAD COMMAND TO ARDUINO\n");
			break;

		case RESP_BAD_RESPONSE:
			printf("PI GOT BAD RESPONSE FROM ARDUINO\n");
			break;

		default:
			printf("PI IS CONFUSED!\n");
	}
}

void handleStatus(const char *buffer)
{
	int32_t data[16];
	memcpy(data, &buffer[1], sizeof(data));

	printf("\n ------- VINCENT STATUS REPORT ------- \n\n");
	printf("Left Ticks:\t\t%d\n", data[0]);
	printf("Right Ticks:\t\t%d\n", data[1]);
	printf("\n---------------------------------------\n\n");
}

void handleMessage(const char *buffer)
{
	printf("MESSAGE FROM VINCENT: %s\n", &buffer[1]);
}

void handleCommand(const char *buffer)
{
	// We don't do anything because we issue commands
	// but we don't get them. Put this here
	// for future expansion
}

void handleNetwork(const char *buffer, int len)
{
	// The first byte is the packet type
	int type = buffer[0];

	switch(type)
	{
		case NET_ERROR_PACKET:
		handleError(buffer);
		break;

		case NET_STATUS_PACKET:
		handleStatus(buffer);
		break;

		case NET_MESSAGE_PACKET:
		handleMessage(buffer);
		break;

		case NET_COMMAND_PACKET:
		handleCommand(buffer);
		break;
	}
}

void sendData(const char *buffer, int len)
{
	int c;
	printf("\nSENDING %d BYTES DATA\n\n", len);
	if(networkActive)
	{
		/* TODO: Insert SSL write here to write buffer to network */
		c = SSL_write(ssl, buffer, strlen(buffer)+1);

		/* END TODO */	
		networkActive = (c > 0);
	}
}

void *receiveThread(void *p)
{
	char buffer[128];
	int len;

	while(networkActive)
	{
		/* TODO: Insert SSL read here into buffer */
		len = SSL_read(ssl, buffer, sizeof(buffer));
		
		/* END TODO */

		networkActive = (len > 0);

		if(networkActive)
			handleNetwork(buffer, len);
	}

	printf("Exiting network listener thread\n");
	pthread_exit(NULL);
}

void connectToServer(const char *serverName, int portNum)
{
	struct sockaddr_in serv_addr;
	memset(&serv_addr, 0, sizeof(serv_addr));

	sockfd = socket(AF_INET, SOCK_STREAM, 0);

	if(sockfd<0)
	{
		perror("Cannot create socket: ");
		exit(-1);
	}

	char hostIP[32];
	struct hostent *he;

	he=gethostbyname(serverName);

	if(he==NULL)
	{
		herror("Unable to get IP address: ");
		exit(-1);
	}

	struct in_addr **addr_list = (struct in_addr **) he->h_addr_list;
	strncpy(hostIP, inet_ntoa(*addr_list[0]), sizeof(hostIP));
	printf("Host %s IP address %s\n", serverName, hostIP);

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(portNum);
	inet_pton(AF_INET, hostIP, &serv_addr.sin_addr);

	/* TODO: Call init_openssl to initialize SSL libraries 
	 		and create a context with create_context */
	// Initialize SSL
	init_openssl();
	// Create context
	ctx = create_context("signing.pem");
	/* END TODO */


	if(connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
	{
		perror("Error connecting: ");
		exit(-1);
	}

	/* TODO: Call connectSSL to create an SSL session, with host name
	   		verification */
	ssl = connectSSL(ctx, sockfd, serverName);

	/* END TODO */

	/* TODO: Call verifyCertificate. If certificate is OK print message to inform user.
	   			If certificate is bad, inform user and exit */
	if (!verifyCertificate(ssl))
	{
		printf("SSL Certificate validation failed.\n");
		closeAndExit(-1);
	}
	else 
		printf("SSL CERTIFICATE IS VALID\n");
	
	networkActive=1;
	// Spawn receive thread
	pthread_t _recv;
	pthread_create(&_recv, NULL, receiveThread, NULL);

	void *result;
	pthread_join(_recv, &result);
	
	printf("Connection closed. Exiting\n");
	closeAndExit(0);
}

void flushInput()
{
	char c;

	while((c = getchar()) != '\n' && c != EOF);
}

void getParams(int32_t *params)
{
	printf("Enter distance/angle in cm/degrees (e.g. 50).\n");
	printf("E.g. 50 means go at 50 cm for forward/backward, or 50 degrees for left/right.\n");
	scanf("%d", &params[0]);
	flushInput();
}

void *kbThread(void *p)
{
	int quit=0;
	char ch;
	
	while(!quit)
	{
		printf("Command (f=forward, b=reverse, l=turn left, r=turn right, s=stop, m=mark location, a=scan lidar, z=backtrack, q=exit)\n");
		scanf("%c", &ch);

		// Purge extraneous characters from input stream
		flushInput();

		char buffer[10];
		int32_t params[2];

		buffer[0] = NET_COMMAND_PACKET;
		switch(ch)
		{
			case 'f':
			case 'F':
					getParams(params);
					buffer[1] = ch;
					memcpy(&buffer[2], params, sizeof(params));
					sendData(buffer, sizeof(buffer));
					break;
			case 'b':
			case 'B':
					getParams(params);
					buffer[1] = ch;
					memcpy(&buffer[2], params, sizeof(params));
					sendData(buffer, sizeof(buffer));
					break;
			case 'l':
			case 'L':
					getParams(params);
					buffer[1] = ch;
					memcpy(&buffer[2], params, sizeof(params));
					sendData(buffer, sizeof(buffer));
					break;
			case 'r':
			case 'R':
					getParams(params);
					buffer[1] = ch;
					memcpy(&buffer[2], params, sizeof(params));
					sendData(buffer, sizeof(buffer));
					break;
			case 's':
			case 'S':
					buffer[1] = ch;
					sendData(buffer, sizeof(buffer));
					break;
			case 'g':
			case 'G':
					params[0]=0;
					params[1]=0;
					memcpy(&buffer[2], params, sizeof(params));
					buffer[1] = ch;
					sendData(buffer, sizeof(buffer));
					break;
			case 'm':
			case 'M':
					buffer[1] = ch;
					sendData(buffer, sizeof(buffer));
					break;
			case 'q':
			case 'Q':
					quit=1;
					break;
			case 'z':
			case 'Z':
					buffer[1] = ch;
					sendData(buffer, sizeof(buffer));
					break;
			case 'a':
			case 'A':
					buffer[1] = ch;
					sendData(buffer, sizeof(buffer));
					break;
			default:
					printf("BAD COMMAND\n");
		}
	}

	printf("Exiting keyboard thread\n");
	pthread_exit(NULL);
}

int main(int ac, char **av)
{
	if(ac != 3)
	{
		fprintf(stderr, "\n\n%s <IP address> <Port Number>\n\n", av[0]);
		exit(-1);
	}

	// Spawn the keyboard thread
	pthread_t _kb;

	// Start the client
	pthread_create(&_kb, NULL, kbThread, NULL);
	connectToServer(av[1], atoi(av[2]));

	void *result;
	pthread_join(_kb, &result);

	printf("\nMAIN exiting\n\n");
}
