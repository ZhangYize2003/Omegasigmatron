#include <serialize.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdarg.h>
#include <math.h>
#include "packet.h"
#include "constants.h"

#define S0 0
#define S1 1
#define S2 2
#define S3 3
#define SENSOR_OUTPUT 4
#define SENSOR_OUTPUT_PIN 26
#define SENSOR_DELAY 20

/*
 * Alex's configuration constants
 */

// Speed of Alex at 100% power in cm/s
#define SPEED 55.0

// Angular speed of Alex at 100% power in degree/s
#define ANGULAR_SPEED 4.0

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC 19.48

// Alex's length and breadth in cm
#define ALEX_LENGTH 25
#define ALEX_BREADTH 16

// Alex's diagonal
float alexDiagonal = 0.0;

// Alex's turning circumference
float alexCirc = 0.0;

volatile unsigned long redFreq = 0;
volatile unsigned long greenFreq = 0;
volatile unsigned long blueFreq = 0;
volatile int is_open = 0;
/*
 *    Alex's State Variables
 */

volatile int gear = 1;
float gearTime = 0;
float moveStartTime = 0;
volatile TDirection dir;

void left() {
  ccw();
}

void right() {
  cw();
}

/*
 * 
 * Alex Communication Routines.
 * 
 */
 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprintf(const char *format, ...) { 
  va_list args; 
  char buffer[128]; 
  va_start(args, format); 
  vsprintf(buffer, format, args); 
  sendMessage(buffer);  
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}

/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */

// Functions to be called by INT2 and INT3 ISRs.
void clawISR()
{
  DDRL |= (1 << PL3) | (1 << PL4);
  TCCR5A = 0b10100010;
  TCCR5B = 0b00010010;
  ICR5 = 20000;

  if (is_open) {
    OCR5A = 400;
    OCR5B = 1500;
    is_open = !is_open;
    dbprintf("Claw is now *OPEN*\n");
  }
  else if (!is_open) {
    OCR5A = 1500;
    OCR5B = 400;
    is_open = !is_open;
    dbprintf("Claw is now *CLOSED*\n");
  }
  
  //open : a:1500, b:400
  //close : a:400, b:1500
}

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  PRR0 &= ~(1 << PRUSART0);
  UBRR0H = (103 >> 8);
  UBRR0L = 103; // 103 for 9600baud
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // Asynchronous USART Mode
  UCSR0A = 0; // Clear the bits of UCSR0A while setting up

  // Serial.begin(9600);
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using the other UARTs
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
  // Enable receiver and transmitter
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);

  // Enable RX complete interrupt
  UCSR0B |= (1 << RXCIE0);
  }

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count=0;

  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs

  while(Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  // See if need to change
  for (int i = 0; i < len; i++) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = buffer[i]; // write each byte of the buffer
  }
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs
}

/*
void sendColour() {
  //Obtain the distance the colour values were taken from
  
  TPacket colourPacket;
  colourPacket.packetType = PACKET_TYPE_RESPONSE;
  colourPacket.command = RESP_COLOUR;
  colourPacket.params[0] = redFreq;
  colourPacket.params[1] = greenFreq;
  colourPacket.params[2] = blueFreq;
  sendResponse(&colourPacket);
}
*/

void colourSetup() {
  DDRA |= ((1 << S0) | (1 << S1) | (1 << S2) | (1 << S3)); //Set S0, S1, S2, S3 to OUTPUT
  DDRA &= ~(1 << SENSOR_OUTPUT); //Set sensorOut to INPUT
  
  //Setting frequency scaling to 20%
  PORTA |= (1 << S0);
  PORTA &= ~(1 << S1);
}

int avgFreq() {
  int reading;
  int total = 0;

  //Obtain 5 readings
  for (int i = 0; i < 5; i++) {
    reading = pulseIn(SENSOR_OUTPUT_PIN, LOW);
    total += reading;
    delay(SENSOR_DELAY);
  }
  //Return the average of 5 readings
  return total / 5;
}

void senseRed() {
  // Setting RED filtered photodiodes to be read
  PORTA &= ~((1 << S2) | (1 << S3)); 
  delay(SENSOR_DELAY);

  // Reading the output frequency for RED
  redFreq = avgFreq();
  delay(SENSOR_DELAY);
  dbprintf("Red frequency: %ld", redFreq);
}

void senseGreen() {
  // Setting GREEN filtered photodiodes to be read
  PORTA |= ((1 << S2) | (1 << S3));
  delay(SENSOR_DELAY);

  // Reading the output frequency for GREEN
  greenFreq = avgFreq();
  delay(SENSOR_DELAY);
  dbprintf("Green frequency: %ld", greenFreq);
}

void senseBlue() {
  // Setting BLUE filtered photodiodes to be read
  PORTA &= ~(1 << S2);
  PORTA |= (1 << S3);
  delay(SENSOR_DELAY);

  // Reading the output frequency for BLUE
  blueFreq = avgFreq();
  delay(SENSOR_DELAY);
  dbprintf("Blue frequency: %ld", blueFreq);
}

void senseColour() { 
  senseRed();
  senseGreen();
  senseBlue();
}

void findColour() {
  if (is_open) {
    if (redFreq <= 150 && greenFreq >= 150) {
      dbprintf("Astronaut is *RED*\n");
    }
    else if (redFreq >= 160 && greenFreq <= 160) {
      dbprintf("Astronaut is *GREEN*\n");
    }
    else {
      dbprintf("I see nothing but chickens...\n");
    }
  }
  else {
    if (redFreq <= 190 && greenFreq >= 190) {
      dbprintf("Astronaut is *RED*\n");
    }
    else if (redFreq >= 195 && greenFreq <= 195) {
      dbprintf("Astronaut is *GREEN*\n");
    }
    else {
      dbprintf("I see nothing but chickens...\n");
    }
  }
}

double findGearTime(int gear) {
  switch (gear)
  {
    case 1:
      gearTime = 50;
      break;
    case 2:
      gearTime = 150;
      break;
    case 3:
      gearTime = 250;
      break;
    case 4:
      gearTime = 500;
      break;
    default:
      gearTime = 0;
  }
}
/*
 * Alex's setup and run codes
 * 
 */

void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        findGearTime(gear);
        forward();
      break;
    case COMMAND_REVERSE:
        sendOK();
        findGearTime(gear);
        backward();
      break;
    case COMMAND_TURN_LEFT:
        sendOK();
        findGearTime(gear);
        left();
      break;
    case COMMAND_TURN_RIGHT:
        sendOK();
        findGearTime(gear);
        right();
      break;
    case COMMAND_GEAR_1:
        sendOK();
        gear = 1;
        dbprintf("gear: %d", gear);
        break;
    case COMMAND_GEAR_2:
        sendOK();
        gear = 2;
        dbprintf("gear: %d", gear);
        break;
    case COMMAND_GEAR_3:
        sendOK();
        gear = 3;
        dbprintf("gear: %d", gear);
        break;
    case COMMAND_GEAR_4:
        sendOK();
        gear = 4;
        dbprintf("gear: %d", gear);
        break;
    case COMMAND_STOP:
        sendOK();
        stop();
      break;
    case COMMAND_COLOUR:
        sendOK();
        senseColour();
        findColour();
        //sendColour();
      break;
    case COMMAND_CLAW:
        sendOK();
        clawISR();
      break;

    /*
     * Implement code for other commands here.
     * 
     */
        
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setup() {
  // put your setup code here, to run once:
  alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  alexCirc = PI * alexDiagonal;

  cli();
  setupSerial();
  startSerial();
  colourSetup();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {
// Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

//forward(0, 100);

// Uncomment the code below for Week 9 Studio 2


 // put your main code here, to run repeatedly:
  
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
  {
    handlePacket(&recvPacket);
  }
  else if(result == PACKET_BAD)
  {
    sendBadPacket();
  }
  else if(result == PACKET_CHECKSUM_BAD)
  {
    sendBadChecksum();
  }
  
  if(gearTime > 0) 
  {
    if(dir == FORWARD)
    {
      if(millis() - moveStartTime >= gearTime)
      {
        moveStartTime = 0;
        gearTime = 0;
        stop();
      }
    }
    else if(dir == BACKWARD)
    {
      if(millis() - moveStartTime >= gearTime)
      {
        moveStartTime = 0;
        gearTime = 0;
        stop();
      }
    }
    else if (dir == LEFT) 
    {
      if (millis() - moveStartTime >= gearTime) 
      {
        moveStartTime = 0;
        gearTime = 0;
        stop();
      }
    }
    else if (dir == RIGHT) 
    {
      if (millis() - moveStartTime >= gearTime) 
      {
        moveStartTime = 0;
        gearTime = 0;
        stop();
      }
    }
    else if (dir == (TDirection) STOP) 
    {
      moveStartTime = 0;
      gearTime = 0;
      stop();
    }  
  }
}
