#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdint.h>
#include <math.h>
#include "packet.h"
#include "serial.h"
#include "serialize.h"
#include "constants.h"

#define PORT_NAME                       "/dev/ttyACM0"
#define BAUD_RATE                       B9600

// Number of ticks per revolution from the
// wheel encoder.
#define COUNTS_PER_REV      384

// Wheel circumference in cm.
// We will use this to calculate ticks required
#define WHEEL_CIRC          20.42

//PI, for calculating turn circumference
#define PI 3.141592654

// Vincent's length and breadth in cm
#define VINCENT_LENGTH 16.0
#define VINCENT_BREADTH 12.5

int exitFlag=0, distance, power;
sem_t _xmitSema;

// Vincent's diagonal. We compute and store this once
// since it is expensive to compute and really doesn't change.
float vincentDiagonal = sqrt((VINCENT_LENGTH * VINCENT_LENGTH) + (VINCENT_BREADTH * VINCENT_BREADTH));

// Vincent's turning circumference, calculated once
float vincentCirc = PI * vincentDiagonal;


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

void handleStatus(TPacket *packet)
{
        printf("\n ------- VINCENT STATUS REPORT ------- \n\n");
        printf("Left Ticks:\t\t%d\n", packet->params[0]);
        printf("Right Ticks:\t\t%d\n", packet->params[1]);
        printf("\n---------------------------------------\n\n");
}

void handleResponse(TPacket *packet)
{
        // The response code is stored in command
        switch(packet->command)
        {
                case RESP_OK:
                        printf("Command OK\n");
                break;

                case RESP_STATUS:
                        handleStatus(packet);
                break;

                default:
                        printf("Arduino is confused\n");
        }
}

void handleErrorResponse(TPacket *packet)
{
        // The error code is returned in command
        switch(packet->command)
        {
                case RESP_BAD_PACKET:
                        printf("Arduino received bad magic number\n");
                break;

                case RESP_BAD_CHECKSUM:
                        printf("Arduino received bad checksum\n");
                break;

                case RESP_BAD_COMMAND:
                        printf("Arduino received bad command\n");
                break;

                case RESP_BAD_RESPONSE:
                        printf("Arduino received unexpected response\n");
                break;

                default:
                        printf("Arduino reports a weird error\n");
        }
}

void handleMessage(TPacket *packet)
{
        printf("Message from Vincent: %s\n", packet->data);
}

void handlePacket(TPacket *packet)
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
        }
}

void sendPacket(TPacket *packet)
{
        char buffer[PACKET_SIZE];
        int len = serialize(buffer, packet, sizeof(TPacket));

        serialWrite(buffer, len);
}

void *receiveThread(void *p)
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
                                handlePacket(&packet);
                        }
                        else
                                if(result != PACKET_INCOMPLETE)
                                {
                                        printf("PACKET ERROR\n");
                                        handleError(result);
                                }
                }
        }
}

void flushInput()
{
        char c;

        while((c = getchar()) != '\n' && c != EOF);
}

void getInputs(){
		printf("Enter distance/angle in cm/degrees (e.g. 50) and power in %% (e.g. 75) separated by space.\n");
        printf("E.g. 50 75 means go at 50 cm at 75%% power for forward/backward, or 50 degrees left or right turn at 75%%  power\n");
        scanf("%d %d", &distance, &power);
        flushInput();
}

void getParamsDist(TPacket *commandPacket)
{
        commandPacket->params[0] = (unsigned long)((float) distance/ WHEEL_CIRC * COUNTS_PER_REV);
        commandPacket->params[1] = (float) power;
}

void getParamsAng(TPacket *commandPacket)
{
        commandPacket->params[0] = (unsigned long)((float) distance*(360.0 * WHEEL_CIRC)) / (vincentCirc * COUNTS_PER_REV);
        commandPacket->params[1] = (float) power;
}

void sendCommand(char command)
{
        TPacket commandPacket;

        commandPacket.packetType = PACKET_TYPE_COMMAND;

        switch(command)
        {
                case 'f':
                case 'F':
						getInputs();
                        getParamsDist(&commandPacket);
                        commandPacket.command = COMMAND_FORWARD;
                        sendPacket(&commandPacket);
                        break;

                case 'b':
                case 'B':
						getInputs();
                        getParamsDist(&commandPacket);
                        commandPacket.command = COMMAND_REVERSE;
                        sendPacket(&commandPacket);
                        break;

                case 'l':
                case 'L':
						getInputs();
                        getParamsAng(&commandPacket);
                        commandPacket.command = COMMAND_TURN_LEFT;
                        sendPacket(&commandPacket);
                        break;

                case 'r':
                case 'R':
						getInputs();
                        getParamsAng(&commandPacket);
                        commandPacket.command = COMMAND_TURN_RIGHT;
                        sendPacket(&commandPacket);
                        break;

                case 's':
                case 'S':
                        commandPacket.command = COMMAND_STOP;
                        sendPacket(&commandPacket);
                        break;

                case 'g':
                case 'G':
                        commandPacket.command = COMMAND_GET_STATS;
                        sendPacket(&commandPacket);
                        break;

                case 'q':
                case 'Q':
                        exitFlag=1;
                        break;

                default:
                        printf("Bad command\n");

        }
}

int main()
{
        // Connect to the Arduino
        startSerial(PORT_NAME, BAUD_RATE, 8, 'N', 1, 5);

        // Sleep for two seconds
        printf("WAITING TWO SECONDS FOR ARDUINO TO REBOOT\n");
        sleep(2);
        printf("DONE\n");

        // Spawn receiver thread
        pthread_t recv;

        pthread_create(&recv, NULL, receiveThread, NULL);

        // Send a hello packet
        TPacket helloPacket;

        helloPacket.packetType = PACKET_TYPE_HELLO;
        sendPacket(&helloPacket);

        while(!exitFlag)
        {
                char ch;
                printf("Command (f=forward, b=reverse, l=turn left, r=turn right, s=stop, g=get stats, q=exit)\n");
                scanf("%c", &ch);

                // Purge extraneous characters from input stream
                flushInput();

                sendCommand(ch);
        }

        printf("Closing connection to Arduino.\n");
        endSerial();
}
