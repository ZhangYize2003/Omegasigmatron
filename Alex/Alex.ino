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
 * Alex's State Variables For Colour Sensor and Claw
 */
volatile unsigned long redFreq = 0;
volatile unsigned long greenFreq = 0;
volatile unsigned long blueFreq = 0;
volatile int is_open = 0;

/*
 * Alex's State Variables For Movement
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

/**
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

void sendColour() {
  TPacket colourPacket;

  colourPacket.packetType = PACKET_TYPE_RESPONSE;
  colourPacket.command = RESP_COLOUR;
  colourPacket.params[0] = redFreq;
  colourPacket.params[1] = greenFreq;
  colourPacket.params[2] = blueFreq;

  sendResponse(&colourPacket);
}

/**
 * Sets up Timer 5 ISR to open/close the claw using PWM.
 * open: a = 1500, b = 400.
 * close: a = 400, b = 1500.
 */
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
}

/*
 * Setup and start codes for serial communications using bare-metal code
 */
void setupSerial()
{
  PRR0 &= ~(1 << PRUSART0);
  UBRR0H = (103 >> 8);
  UBRR0L = 103; // 103 for 9600baud
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // Asynchronous USART Mode
  UCSR0A = 0; // Clear the bits of UCSR0A while setting up
}

/*
 * Initialises the USART communication by enabling the receiver,
 * transmitter, and RX complete interrupt on USART0.
 */
void startSerial()
{
  // Enable receiver and transmitter
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);

  // Enable RX complete interrupt
  UCSR0B |= (1 << RXCIE0);
}

/**
 * Read the serial port. Returns the number of characters read into buffer.
 *
 * @param [in] buffer Pointer to a character array where the incoming serial data will be stored.
 *
 * @return The number of characters successfully read from the serial port.
 */
int readSerial(char *buffer)
{
  int count=0;

  while(Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

/**
 * Write data to the serial port, sending each byte from the given buffer.
 * 
 * @param[in] buffer Pointer to the character array containing data to send.
 * @param[in] len Number of bytes to send from the buffer.
 */
void writeSerial(const char *buffer, int len)
{
  for (int i = 0; i < len; i++) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = buffer[i]; // write each byte of the buffer
  }
}

/**
 * Configure the color sensor pins by setting up S0, S1, S2, S3 to OUTPUT,
 * sensorOut to INPUT and initialising the frequency scaling to 20%.
 */
void colourSetup() {
  DDRA |= ((1 << S0) | (1 << S1) | (1 << S2) | (1 << S3)); //Set S0, S1, S2, S3 to OUTPUT
  DDRA &= ~(1 << SENSOR_OUTPUT); //Set sensorOut to INPUT
  PORTA |= (1 << S0);
  PORTA &= ~(1 << S1);
}

/**
 * Calculates the average pulse duration from a digital colour sensor by
 * reading the duration of pulses 5 times.
 *
 * @return The average pulse duration over 5 readings.
 */
int avgFreq() {
  int reading;
  int total = 0;

  for (int i = 0; i < 5; i++) {
    reading = pulseIn(SENSOR_OUTPUT_PIN, LOW);
    total += reading;
    delay(SENSOR_DELAY);
  }
  return total / 5;
}

/**
 * Measures and stores the output corresponding to the red component of 
 * light detected by the colour sensor. It changes the logic of S2 and S3
 * pins to read the red photodiode.
 */
void senseRed() {
  PORTA &= ~((1 << S2) | (1 << S3)); 
  delay(SENSOR_DELAY);

  redFreq = avgFreq();
  delay(SENSOR_DELAY);
  dbprintf("Red frequency: %ld", redFreq);
}

/**
 * Measures and stores the output corresponding to the green component of 
 * light detected by the colour sensor. It changes the logic of S2 and S3
 * pins to read the green photodiode.
 */
void senseGreen() {
  PORTA |= ((1 << S2) | (1 << S3));
  delay(SENSOR_DELAY);

  greenFreq = avgFreq();
  delay(SENSOR_DELAY);
  dbprintf("Green frequency: %ld", greenFreq);
}

/**
 * Measures and stores the output corresponding to the blue component of 
 * light detected by the colour sensor. It changes the logic of S2 and S3
 * pins to read the blue photodiode.
 */
void senseBlue() {
  PORTA &= ~(1 << S2);
  PORTA |= (1 << S3);
  delay(SENSOR_DELAY);

  blueFreq = avgFreq();
  delay(SENSOR_DELAY);
  dbprintf("Blue frequency: %ld", blueFreq);
}

/**
 * Measures the output corresponding to all components of 
 * light detected by the colour sensor
 */
void senseColour() { 
  senseRed();
  senseGreen();
  senseBlue();
}

/**
 * This function determines the color detected by the color sensor based on red 
 * and green frequency readings, and prints the corresponding result. The threshhold
 * frequencies are derived experimentally in the lab.
 */
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

/**
 * Sets the gearTime variable based on the selected gear level.
 * 
 * @param [in] gear An integer representing the current gear level.
 */
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
 * Alex's setup and movement codes
 */
void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    // For movement commands, the default speed is at 100% power.
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
        sendColour();
      break;
    case COMMAND_CLAW:
        sendOK();
        clawISR();
      break;      
    default:
      sendBadCommand();
  }
}

/**
 * Waits for a valid HELLO packet to be received from the communication channel.
 */
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
  }
}

void setup() {
  cli();
  setupSerial();
  startSerial();
  colourSetup();
  sei();
}

/**
 * Handles an incoming packet based on its type.
 */
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

/**
 * Main loop  that continuously stores incoming packets as results and
 * manages robot movement based on duration and direction.
 * Alex stops moving when the duration of movement exceeds gearTime.
 */
void loop() { 
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
