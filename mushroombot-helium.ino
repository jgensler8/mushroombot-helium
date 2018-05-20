/*
 * Copyright 2017, Helium Systems, Inc.
 * All Rights Reserved. See LICENCE.txt for license information
 */

#include "Arduino.h"
#include "Board.h"
#include <Helium.h>
#include <HeliumUtil.h>
#include <heliumlft.h>
#include <Adafruit_VC0706.h>
#include <base64.hpp>

// NOTE: Please ensure you've created a channel with the above
// CHANNEL_NAME as it's name.
#define CHANNEL_NAME "mushroombot"

Helium  helium(&atom_serial);
Channel channel(&helium);

SoftwareSerial cameraconnection = SoftwareSerial(2, 3);
Adafruit_VC0706 cam = Adafruit_VC0706(&cameraconnection);

Packet p;
//Packet additional_p;
//char additional_p_data[HELIUM_LFT_MAX_DATA_SIZE];
char b64_p_data[HELIUM_LFT_MAX_DATA_SIZE];

struct helium_info info;

uint32_t transaction_id = 0;

void
setup()
{
    Serial.begin(9600);
    Serial.println("Starting");

    // ~~~~~~~~~~~~~~~~~~ CAMERA ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    cam.begin(38400);

    delay(500);
    // for good measure. not sure if this will help
    cam.reset();
    char *reply = cam.getVersion();
    if (reply == 0) {
      Serial.print("Failed to get version");
    } else {
      Serial.println("-----------------");
      Serial.print(reply);
      Serial.println("-----------------");
    }
    
    // ~~~~~~~~~~~~~~~~~~ HELIUM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Begin communication with the Helium Atom
    // The baud rate differs per supported board
    // and is configured in Board.h1
    helium.begin(HELIUM_BAUD_RATE);
    atom_serial.listen();
    Serial.println("Atom Listening...");
    helium_connect(&helium);
    Serial.println("Helium Connected...");
    channel_create(&channel, CHANNEL_NAME);
    Serial.println("Channel Created...");
}

void
loop()
{
  
  helium.info(&info);
  transaction_id = info.time;
  
  cam.reset();    
    cam.setImageSize(VC0706_640x480);        // biggest
//    cam.setImageSize(VC0706_320x240);        // medium
//    cam.setImageSize(VC0706_160x120);          // small
  delay(500);
  cameraconnection.listen();
    cam.setImageSize(VC0706_640x480);        // biggest
//    cam.setImageSize(VC0706_320x240);        // medium
//    cam.setImageSize(VC0706_160x120);          // small
  if (! cam.takePicture()) 
    Serial.println("Failed to snap!");
  else 
    Serial.println("Picture taken!");
  
  // Get the size of the image (frame) taken  
  uint16_t jpeg_bytes_left = cam.frameLength();
  Serial.print("Storing ");
  Serial.print(jpeg_bytes_left, DEC);
  Serial.print(" byte image.");
  
  int packet_index = 0;
  int maxBytesToRead = 32;
  int number_of_packets = ceil((float)jpeg_bytes_left / maxBytesToRead );
  
  while (jpeg_bytes_left > 0) {
    uint8_t bytesToRead = min(maxBytesToRead, jpeg_bytes_left);
    cameraconnection.listen();
    p.data = cam.readPicture(bytesToRead);
    p.len = bytesToRead;
    jpeg_bytes_left -= bytesToRead;

// somehow, we lose part of p.data... not sure why...
//    if(jpeg_bytes_left > 0) {
//      uint8_t bytesToRead = min(maxBytesToRead, jpeg_bytes_left);
//      additional_p.data = cam.readPicture(bytesToRead);
//      additional_p.len = bytesToRead;
//      jpeg_bytes_left -= bytesToRead;
//
//      memcpy(additional_p_data, p.data, p.len);
//      memcpy(additional_p_data + p.len, additional_p.data, additional_p.len);
//
//      p.data = additional_p_data;
//      p.len += additional_p.len;
//    }

    // base64 encode data
    p.len = encode_base64(p.data, p.len, b64_p_data);
    p.data = b64_p_data;

    // create channelable packet
    Packet channelable_p = generate_channelable_packet(transaction_id, packet_index, number_of_packets, p);
    Serial.print("Uploading ");
    Serial.println((char*)channelable_p.data);

    // upload the packet
    atom_serial.listen();
    Serial.println("Atom Listening...");
    channel_send(&channel, CHANNEL_NAME, channelable_p.data, channelable_p.len);
    
    packet_index += 1;
  }

  transaction_id += 1;

  // Wait about 60 seconds
  delay(60000);
}
