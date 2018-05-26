/*
 * Copyright 2017, Helium Systems, Inc.
 * All Rights Reserved. See LICENCE.txt for license information
 */

#include "Arduino.h"
#include "Board.h"
#include <Helium.h>
#include <HeliumUtil.h>
#include <Adafruit_VC0706.h>
#include <base64.hpp>

// helium
#define CHANNEL_NAME "mushroombot"
#define CHANNEL_CHECKPOINT_KEY "channel.checkpoint"
#define CHANNEL_DELTA_KEY "channel.delta"
Helium  helium(&atom_serial);
Channel channel(&helium);
Config  config(&channel);

// camera
#define CAMERA_MAX_BYTES_TO_READ 32
SoftwareSerial cameraSerial = SoftwareSerial(2, 3);
Adafruit_VC0706 cameraDevice = Adafruit_VC0706(&cameraSerial);

int32_t last_pic_checkpoint = 0;

uint32_t
now(Helium* helium)
{
  struct helium_info info;
  int status = helium->info(&info);
  if(status != helium_status_OK){
    return 0;
  } else {
    return info.time;
  }
}

bool
shouldUploadData(Helium* helium, Config* config, SoftwareSerial* atomSerial)
{
  atomSerial->listen();
  
  float checkpoint;
  int status = config->get(CHANNEL_CHECKPOINT_KEY, &checkpoint, 1.0);
  if(status != helium_status_OK) {
    Serial.print(F("Failed to get new checkpoint: "));
    Serial.println(status);
    return false;
  }
  
  float delta = 0;
  status = config->get(CHANNEL_DELTA_KEY, &delta, 1.0);
  if(status != helium_status_OK) {
    Serial.print(F("Failed to get new delta: "));
    Serial.println(status);
    return false;
  }

  uint32_t checkpoint_ = (uint32_t) checkpoint;
  uint32_t delta_ = (uint32_t) delta;

  // if we haven't acknowledged this checkpoint and are in the time deleta, we should take a picture
  uint32_t now_ = now(helium);
  Serial.print(F("Now: "));
  Serial.print((unsigned long)now_);
  Serial.print(F(", Checkpoint: "));
  Serial.print(checkpoint_);
  Serial.print(F(", Delta: "));
  Serial.println(delta_);
  return checkpoint_ > last_pic_checkpoint && now_ >= checkpoint_ && now_ < checkpoint_ + delta_;
}

uint16_t
takePicture(SoftwareSerial* cameraSerial, Adafruit_VC0706* cameraDevice)
{
  cameraDevice->reset();    
  cameraDevice->setImageSize(VC0706_640x480);
  delay(500);
  cameraSerial->listen();
  cameraDevice->setImageSize(VC0706_640x480);
  if(cameraDevice->takePicture()) {
    return cameraDevice->frameLength();
  } else {
    return 0;
  }
}
  
void
takeAndUploadPicture(Helium* helium, Channel* channel, SoftwareSerial* atomSerial, SoftwareSerial* cameraSerial, Adafruit_VC0706* cameraDevice)
{
  uint32_t transaction_id = now(helium);

  // Take Picture
  uint16_t jpeg_bytes_left = takePicture(cameraSerial, cameraDevice);
  if(jpeg_bytes_left == 0) {
    return;
  }
  Serial.print(F("Image Size Bytes: "));
  Serial.println(jpeg_bytes_left, DEC);

  // Upload Picture
  char* data;
  char data_b64[CAMERA_MAX_BYTES_TO_READ * 4 / 3];
  char packet[HELIUM_MAX_DATA_SIZE];
  int packet_index = 0;
  int total_number_of_packets = ceil((float)jpeg_bytes_left / CAMERA_MAX_BYTES_TO_READ );

  while (jpeg_bytes_left > 0) {
    uint8_t bytesToRead = min(CAMERA_MAX_BYTES_TO_READ, jpeg_bytes_left);
    cameraSerial->listen();
    data = cameraDevice->readPicture(bytesToRead);
    jpeg_bytes_left -= bytesToRead;

    // base64 encode data
    int len_b64 = encode_base64(data, bytesToRead, data_b64);

    // create channelable packet
    int total_prefix_characters = sprintf(packet, "%lu,%d,%d,\0", transaction_id, packet_index, total_number_of_packets);
    memcpy(packet + total_prefix_characters, data_b64, len_b64);
    sprintf(packet + total_prefix_characters + len_b64, "\n");
    Serial.print((char*)packet);

    // upload the packet
    atomSerial->listen();
    channel_send(channel, CHANNEL_NAME, packet, total_prefix_characters + len_b64 + 1);
    
    packet_index += 1;
  }
}


void
setupSerial()
{
    Serial.println(F("Starting Serial"));
    Serial.begin(9600);
}

void
setupCamera(SoftwareSerial* cameraSerial, Adafruit_VC0706* cameraDevice)
{
  Serial.println(F("Starting Camera"));
  cameraDevice->begin(38400);
  // for good measure. not sure if this will help
  delay(500);
  cameraDevice->reset();
}

void
setupHelium(Helium* helium, Channel* channel)
{
  // Begin communication with the Helium Atom
  // The baud rate differs per supported board
  // and is configured in Board.h1
  helium->begin(HELIUM_BAUD_RATE);
  Serial.println(F("Atom Listening..."));
  helium_connect(helium);
  Serial.println(F("Helium Connected..."));
  channel_create(channel, CHANNEL_NAME);
  Serial.println(F("Channel Created...")); 
}

void
setup()
{
  setupSerial();
  setupCamera(&cameraSerial, &cameraDevice);
  setupHelium(&helium, &channel);
}

void
loop()
{
  if(shouldUploadData(&helium, &config, &atom_serial)) {
    Serial.println(F("Uploading Data..."));
    takeAndUploadPicture(&helium, &channel, &atom_serial, &cameraSerial, &cameraDevice);
  //    readAndUploadTemperature();
  //    readAndUploadHumidity();
  //    readAndUploadSoilMoisture();
    last_pic_checkpoint = now(&helium);
  }
  
//  Serial.println("updating checkpoint");
//  int config_set_result = config.set("channel.checkpoint", last_pic_checkpoint + 1);
//  if(config_set_result != helium_status_OK) {
//    Serial.print("failed to update checkpoint: ");
//    Serial.println(config_set_result);
//  }

  Serial.println(F("Loop Ended"));
  delay(5000);
}
