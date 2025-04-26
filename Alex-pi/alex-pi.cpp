#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdint.h>
#include "packet.h"
#include "serial.h"
#include "serialize.h"
#include "constants.h"

#define PORT_NAME			"/dev/ttyACM0"
#define BAUD_RATE			B9600

int exitFlag=0;
sem_t _xmitSema;

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

void handleColour(TPacket *packet) {
	printf("R:\t\t%d\n", packet->params[0]);
	printf("G:\t\t%d\n", packet->params[1]);
	printf("B:\t\t%d\n", packet->params[2]);
}

void handleResponse(TPacket *packet)
{
	// The response code is stored in command
	switch(packet->command)
	{
		case RESP_OK:
			printf("Command OK\n");
		break;

		case RESP_COLOUR:
			handleColour(packet);

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
	printf("Message from Alex: %s\n", packet->data);
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

void sendCommand(char command)
{
	TPacket commandPacket;

	commandPacket.packetType = PACKET_TYPE_COMMAND;

	switch(command)
	{
		case 'w':
		case 'W':
			commandPacket.command = COMMAND_FORWARD;
			sendPacket(&commandPacket);
			break;

		case 's':
		case 'S':
			commandPacket.command = COMMAND_REVERSE;
			sendPacket(&commandPacket);
			break;

		case 'a':
		case 'A':
			commandPacket.command = COMMAND_TURN_LEFT;
			sendPacket(&commandPacket);
			break;

		case 'd':
		case 'D':
			commandPacket.command = COMMAND_TURN_RIGHT;
			sendPacket(&commandPacket);
			break;

		case 'f':
		case 'F':
			commandPacket.command = COMMAND_STOP;
			sendPacket(&commandPacket);
			break;

		case 'c':
		case 'C':
			commandPacket.command = COMMAND_COLOUR;
			sendPacket(&commandPacket);
			break;

		case 'g':
		case 'G':
			commandPacket.command = COMMAND_CLAW;
			sendPacket(&commandPacket);
			break;

		case '1':
			commandPacket.command = COMMAND_GEAR_1;
			sendPacket(&commandPacket);
			break;

		case '2':
			commandPacket.command = COMMAND_GEAR_2;
			sendPacket(&commandPacket);
			break;

		case '3':
			commandPacket.command = COMMAND_GEAR_3;
			sendPacket(&commandPacket);
			break;

		case '4':
			commandPacket.command = COMMAND_GEAR_4;
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
		printf("Command (1-4=gear, w=forward, s=reverse, a=turn left, d=turn right, f=stop, c=colour, g=claw, q=exit)\n");
		scanf("%c", &ch);

		// Purge extraneous characters from input stream
		flushInput();

		sendCommand(ch);
	}

	printf("Closing connection to Arduino.\n");
	endSerial();
}
