#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <sys/types.h>
#include <errno.h>
#include <string.h>
#include <string>
#include <math.h>
#include "comm_protocol/packet.h"
#include "comm_protocol/constants.h"
#include "serial/serial.h"
#include "serial/serialize.h"
#include "tls/netconstants.h"
#include "tls/tls_server_lib.h"
#include "tls/tls_pthread.h"
#include "lidar/lidar.h"
#include "gnuplot/gnuplot_i.h"

using namespace std;


#define SERVER_PORT			5000
#define PORT_NAME			"/dev/ttyACM0"
#define BAUD_RATE			B9600

// Our network buffer consists of 1 byte of packet type, and 128 bytes of data
#define BUF_LEN				129

#define COUNTS_PER_REV 384
#define WHEEL_CIRC 20.42
#define VINCENT_BREADTH 8
#define PI 3.141592654

float vincentCirc = PI * VINCENT_BREADTH;

char last_command; //user inputted direction

//float x=0, y=0;
//float angle_from_origin = 0; //the current angle Vincent is facing, with ref to 0 where it is the initial North

// This variable shows whether a network connection is active
// We will also use this variable to prevent the server from serving
// more than one connection, to keep connection management simple.
static volatile int networkActive;

// The listener and connection file descriptors
int listenfd, connfd;

// NEW: SSL related structures
SSL_CTX *ctx; // SSL Context
SSL *ssl; // SSL Session

/*

	Vincent Serial Routines to the Arduino

	*/

void startGnuplot(){
	printf("Starting GNUPlot...\n");
	static gnuplot_ctrl * h;
	static int count = 0;
	//h = gnuplot_init();

	//gnuplot_setstyle(h, "dots") ;
    //gnuplot_set_xlabel(h, "X axis") ;
    //gnuplot_set_xlabel(h, "Y axis") ;
	char mainfile[] = "./lidar/lidar_reading.dat" ;
	char locationfile[] = "./lidar/arrow.dat";
	if(count == 0){
		h = gnuplot_init();
		gnuplot_cmd(h, "set xrange[ 0 : 1000]");
		gnuplot_cmd(h, "set yrange[ 0 : 1000]");

		gnuplot_cmd(h, "plot '%s' using 1:2 with points,\ '%s'  using 1:2 with points", mainfile, locationfile);
		printf("GNUPlot initialization complete.\n");
	}
	else{
		gnuplot_resetplot(h);
		gnuplot_cmd(h, "plot '%s' using 1:2 with points,\ '%s'  using 1:2 with points", mainfile, locationfile);
		printf("Updated GNUPlot\n");
	}
	count++;

}

void sendLidar(TPacket *packet) {
	/*float distMoved =0; //x and y are the coordinates from origin (0,0)
	float angleMoved = 0; //for angle, take clockwise as +ve

    
    if (last_command == 'F' || last_command == 'f') {
        distMoved = packet->params[0]/COUNTS_PER_REV*WHEEL_CIRC; // converts ticks to distance 
        float rad = angle_from_origin/180.0*PI;
            
        x = distMoved*sin(rad);
        y = distMoved*cos(rad);
        scan_lidar(&x, &y, &angle_from_origin);
    }
    else if (last_command == 'B' || last_command == 'b') {
        distMoved = packet->params[0]/COUNTS_PER_REV*WHEEL_CIRC; // converts ticks to distance
        float rad = angle_from_origin/180.0*PI;

        x = -distMoved*sin(rad);
        y = -distMoved*cos(rad);
        scan_lidar(&x, &y, &angle_from_origin);
    }
    else if (last_command == 'L' || last_command == 'l') {
        angleMoved = (float) (packet->params[0] / (vincentCirc * COUNTS_PER_REV)) * (360.0 * WHEEL_CIRC);// converts ticks to angle
        angle_from_origin -= angleMoved;
        scan_lidar(&x, &y, &angle_from_origin);
    }
    else if (last_command == 'R' || last_command == 'r') {
        angleMoved =  (float) (packet->params[0] / (vincentCirc * COUNTS_PER_REV)) * (360.0 * WHEEL_CIRC); //converts ticks to angle
        angle_from_origin += angleMoved;
        scan_lidar(&x, &y, &angle_from_origin);
    }
    */
    //shutdown_lidar();
	scan_lidar();
	startGnuplot();
}  

void startAudio(){
	FILE *fp = popen("mplayer audio.mp3", "w");
    sleep(5);
    fprintf(fp,"q");
	pclose(fp);
}

void takePicture(){
	static int picture_count = 0;

	char pic_cmd[50];
	printf("Taking picture...\n");
	sprintf(pic_cmd, "raspistill -o location%d.jpeg",picture_count);
	sleep(1);
	FILE *fp = popen(pic_cmd, "w");
	pclose(fp);
	picture_count++;
	printf("Picture taken\n");
}

// Prototype for sendNetworkData
void sendNetworkData(const char *, int);

void handleErrorResponse(TPacket *packet)
{
	printf("UART ERROR: %d\n", packet->command);
	char buffer[2];
	buffer[0] = NET_ERROR_PACKET;
	buffer[1] = packet->command;
	sendNetworkData(buffer, sizeof(buffer));
}

void handleMessage(TPacket *packet)
{
	char data[33];
	printf("UART MESSAGE PACKET: %s\n", packet->data);
	data[0] = NET_MESSAGE_PACKET;
	memcpy(&data[1], packet->data, sizeof(packet->data));
	sendNetworkData(data, sizeof(data));
}

void handleStatus(TPacket *packet)
{
	char data[65];
	printf("UART STATUS PACKET\n");
	data[0] = NET_STATUS_PACKET;
	memcpy(&data[1], packet->params, sizeof(packet->params));
	sendNetworkData(data, sizeof(data));
	sendLidar(packet);
}


void handleResponse(TPacket *packet)
{
	// The response code is stored in command
	switch(packet->command)
	{
		case RESP_OK:
			char resp[2];
			printf("Command OK\n");
			resp[0] = NET_ERROR_PACKET;
			resp[1] = RESP_OK;
			sendNetworkData(resp, sizeof(resp));
		break;

		case RESP_STATUS:
			handleStatus(packet);
		break;

		default:
		printf("Boo\n");
	}
}


void handleUARTPacket(TPacket *packet)
{
	switch(packet->packetType)
	{
		case PACKET_TYPE_COMMAND:
				// Only we send command packets, so ignore
			break;

		case PACKET_TYPE_RESPONSE:
				handleResponse(packet);
			break;

		case PACKET_TYPE_ERROR:
				handleErrorResponse(packet);
			break;

		case PACKET_TYPE_MESSAGE:
				handleMessage(packet);
			break;
		case PACKET_TYPE_PLAY_SOUND:
				printf("At marked location, playing audio... \n");
				startAudio();
			break;
	}
}


void uartSendPacket(TPacket *packet)
{
	char buffer[PACKET_SIZE];
	int len = serialize(buffer, packet, sizeof(TPacket));

	serialWrite(buffer, len);
}

void handleError(TResult error)
{
	switch(error)
	{
		case PACKET_BAD:
			printf("ERROR: Bad Magic Number\n");
			break;

		case PACKET_CHECKSUM_BAD:
			printf("ERROR: Bad checksum\n");
			break;

		default:
			printf("ERROR: UNKNOWN ERROR\n");
	}
}

void *uartReceiveThread(void *p)
{
	char buffer[PACKET_SIZE];
	int len;
	TPacket packet;
	TResult result;
	int counter=0;

	while(1)
	{
		len = serialRead(buffer);
		counter+=len;
		if(len > 0)
		{
			result = deserialize(buffer, len, &packet);

			if(result == PACKET_OK)
			{
				counter=0;
				handleUARTPacket(&packet);
			}
			else 
				if(result != PACKET_INCOMPLETE)
				{
					printf("PACKET ERROR\n");
					handleError(result);
				} // result
		} // len > 0
	} // while
}

/*

	Vincent Network Routines

	*/


void sendNetworkData(const char *data, int len)
{   
	// Send only if network is active
	if(networkActive)
	{
		printf("WRITING TO CLIENT\n");

		/* TODO: Implement SSL write here to write data to the network */
        
        int c = SSL_write(ssl, data, len);

		/* END TODO */

		// Network is still active if we can write more then 0 bytes.
		networkActive = (c > 0);
	}
}

void handleCommand(const char *buffer)
{
	// The first byte contains the command
	char cmd = buffer[1];
	if(cmd != 's' && cmd != 'S')
		last_command = cmd;
	uint32_t cmdParam[1];

	// Copy over the parameters.
	memcpy(cmdParam, &buffer[2], sizeof(cmdParam));

	TPacket commandPacket;
	
	int32_t temp = cmdParam[0];
	
	if(cmd == 'f' || cmd == 'F' || cmd == 'b' || cmd == 'B'){
		cmdParam[0] = temp/WHEEL_CIRC*COUNTS_PER_REV; //convert distance to ticks
		printf("COMMAND RECEIVED: %c %d, TO ARDUINO: %d\n", cmd, temp, cmdParam[0]);
	}
	else if(cmd == 'l' || cmd == 'L' || cmd == 'r' || cmd == 'R'){
		cmdParam[0] = (temp * vincentCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC); // converts angle to ticks
		printf("COMMAND RECEIVED: %c %d, TO ARDUINO: %d\n", cmd, temp, cmdParam[0]);
	}
	else{
		printf("COMMAND RECEIVED: %c\n", cmd);
	}
	
	commandPacket.params[0] = cmdParam[0];
	
	switch(cmd)
	{
		case 'f':
		case 'F':
			commandPacket.command = COMMAND_FORWARD;
			uartSendPacket(&commandPacket);
			break;

		case 'b':
		case 'B':
			commandPacket.command = COMMAND_REVERSE;
			uartSendPacket(&commandPacket);
			break;

		case 'l':
		case 'L':
			commandPacket.command = COMMAND_TURN_LEFT;
			uartSendPacket(&commandPacket);
			break;

		case 'r':
		case 'R':
			commandPacket.command = COMMAND_TURN_RIGHT;
			uartSendPacket(&commandPacket);
			break;

		case 's':
		case 'S':
			commandPacket.command = COMMAND_STOP;
			uartSendPacket(&commandPacket);
			break;

		case 'z':
		case 'Z':
			commandPacket.command = COMMAND_BACKTRACK;
			uartSendPacket(&commandPacket);
			break;
			
		case 'm':
		case 'M':
			commandPacket.command = COMMAND_MARK_LOCATION;
			uartSendPacket(&commandPacket);
			takePicture();
			break;

		case 'g':
		case 'G':
			commandPacket.command = COMMAND_GET_STATS;
			uartSendPacket(&commandPacket);
			break;
			
		case 'a':
		case 'A':
			scan_lidar();
			startGnuplot();
			break;
			
		default:
			printf("Bad command\n");

	}
}

void handleNetworkData(const char *buffer, int len)
{
	
	if(buffer[0] == NET_COMMAND_PACKET)
		handleCommand(buffer);
}

void *netRecvThread(void *p)
{
	int len;

	char buffer[BUF_LEN];
	
	while(networkActive)
	{
		/* TODO: Implement SSL read into buffer */
        
        len = SSL_read(ssl, buffer, sizeof(buffer));

		/* END TODO */
		// As long as we are getting data, network is active
		networkActive=(len > 0);

		if(len > 0)
			handleNetworkData(buffer, len);
		else
			if(len < 0)
				perror("ERROR READING NETWORK: ");
	}
	pthread_exit(NULL);
}


void startServer(int portNum)
{
	struct sockaddr_in serv_addr;
	memset(&serv_addr, 0, sizeof(serv_addr));
	listenfd = socket(AF_INET, SOCK_STREAM, 0);

	if(listenfd < 0)
	{
		perror("Cannot create socket:");
		exit(-1);
	}

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	serv_addr.sin_port = htons(portNum);

	if(bind(listenfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
	{
		perror("Unable to bind: ");
		exit(-1);
	}

	printf("Listening..\n");
	if(listen(listenfd, 10) < 0)
	{
		perror("Unable to listen to port: ");
		exit(-1);
	}

	int c = sizeof(struct sockaddr_in);

	/* TODO: Call init_openssl to initialize library, then create the context.
	   		After creating the context call configure_context to load Vincent's
			certificate and private key */

	// Initialize SSL
    init_openssl();

	// Create the context
    SSL_CTX *ctx = create_context();

	// Configure the context
    configure_context(ctx, "vincent.crt", "vincent.key");

	/* TODO END */

	// Configure for multithreading
	CRYPTO_thread_setup();

	while(1)
	{
		struct sockaddr_in client;
		connfd = accept(listenfd, (struct sockaddr *) &client, (socklen_t *)&c);

		if(connfd < 0)
		{
			perror("ACCEPT ERROR: ");
			exit(-1);
		}
		char clientAddress[32];

		// Use inet_ntop to extract client's IP address.
		inet_ntop(AF_INET, &client.sin_addr, clientAddress, 32);

		printf("Received connection from %s\n", clientAddress);

		// New connection. Set networkActive to true
		networkActive=1;

		/* TODO: Call connectSSL to initiate the SSL connection */
		// Create SSL session
        ssl = connectSSL(ctx, connfd);

		/* END TODO */

		// Now spawn handler thread
		pthread_t netHandler;
		pthread_create(&netHandler, NULL, netRecvThread, NULL);

		// We wait for the netHandler thread to end, stopping us from accepting new
		// connections. This is just to simplify connection management because we
		// only need to think of one connection at a time, instead of having to resolve
		// which connection sent what command and who should get what response.

		void *result;
		pthread_join(netHandler, &result);

		printf("Client closed connections.\n");
		// Close the connection
		close(connfd);
		SSL_free(ssl);
	}

	close(listenfd);
	SSL_CTX_free(ctx);
	thread_cleanup();
	cleanup_openssl();
}

// We have a separate thread for the server since it runs infinitely
void *serverThread(void *p)
{
	startServer(SERVER_PORT);
	pthread_exit(NULL);
}

void sendHello()
{
	// Send a hello packet
	TPacket helloPacket;

	helloPacket.packetType = PACKET_TYPE_HELLO;
	uartSendPacket(&helloPacket);
}

int main(int argc, const char * argv[])
{
	printf("\nVINCENT REMOTE SSL CLIENT\n\n");

	printf("=============LIDAR INITIALIZATION==============\n");
	initialize_lidar(argc, argv);
	printf("===============================================\n\n");

	// Start the uartReceiveThread and serverThread. The network listener thread
	// is started by serverThread
	pthread_t serThread, netThread;

	printf("=============COMMUNICATION INITIALIZATION======\n");
	printf("Opening Serial Port\n");
	// Open the serial port
	startSerial(PORT_NAME, BAUD_RATE, 8, 'N', 1, 5);
	printf("Done. Waiting 3 seconds for Arduino to reboot\n");
	sleep(3);

	printf("DONE. Starting Serial Listener\n");
	pthread_create(&serThread, NULL, uartReceiveThread, NULL);

	printf("Starting Network Listener\n");
	pthread_create(&netThread, NULL, serverThread, NULL);
	printf("DONE. Sending HELLO to Arduino\n");
	sendHello();
	printf("DONE.\n");
	printf("===============================================\n\n");


	void *r;
	//pthread_join(serThread, &r);
	pthread_join(netThread, &r);
}
