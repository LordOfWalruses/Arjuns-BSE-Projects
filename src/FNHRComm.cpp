/*
 * File       Communication class for Freenove Hexapod Robot
 * Author     Ethan Pan @ Freenove (support@freenove.com)
 * Date       2023/02/09
 * Version    V12.5
 * Copyright  Copyright © Freenove (http://www.freenove.com)
 * License    Creative Commons Attribution ShareAlike 3.0
 *            (http://creativecommons.org/licenses/by-sa/3.0/legalcode)
 * -----------------------------------------------------------------------------------------------*/

#if defined(ARDUINO_AVR_MEGA2560)

#include "FNHRComm.h"

Communication* communication = NULL;

Communication::Communication() {}

void Communication::Start(bool commFunction)
{
  this->commFunction = commFunction;

  StartStateLed();

  robotAction.Start();

  communication = this;

  FlexiTimer2::set(20, UpdateService);
  FlexiTimer2::start();

  if (commFunction)
  {
    SetRobotBootState(GetRobotBootState());

    StartPins();
    StartSerial();
    StartRF24();
    StartESP8266();
  }
}

void Communication::SetWiFi(String name, String password)
{
  esp8266SSID = name;
  esp8266PWD = password;
}

void Communication::SetWiFiChannel(byte channel)
{
  channel = constrain(channel, 1, 13);
  esp8266CHL = channel;
}

void Communication::SetRemote(byte byte0, byte byte1, byte byte2, byte byte3, byte byte4)
{
  rf24Address[0] = byte0;
  rf24Address[1] = byte1;
  rf24Address[2] = byte2;
  rf24Address[3] = byte3;
  rf24Address[4] = byte4;
}

void Communication::SetRemoteChannel(byte channel)
{
  channel = constrain(channel, 0, 125);
  rf24Channel = channel;
}

void Communication::StartStateLed()
{
  pinMode(stateLedPin, OUTPUT);
  digitalWrite(stateLedPin, LOW);
}

void Communication::SetStateLed(bool state)
{
  digitalWrite(stateLedPin, state);
}

void Communication::ReverseStateLed()
{
  stateLedState = !stateLedState;
  SetStateLed(stateLedState);
}

void Communication::StartPins()
{
  for (int i = 0; i < 8; i++)
  {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
  }
}

void Communication::StartSerial()
{
  Serial.begin(115200);
}

void Communication::UpdateSerial()
{
  while (Serial.available() > 0)
  {
    byte inByte = Serial.read();
    if (inByte == Orders::transStart)
      serialInDataCounter = 0;
    serialInData[serialInDataCounter++] = inByte;
    if (inByte == Orders::transEnd)
      if (serialInData[0] == Orders::transStart)
      {
        HandleOrder(serialInData, OrderSource::FromSerial);
        serialInDataCounter = 0;
      }
  }
}

void Communication::StartRF24()
{
  pinMode(50, INPUT_PULLUP);
  isRF24Available = rf24.begin();
  if (!isRF24Available)
    return;
  rf24.setRetries(3, 15);
  rf24.setPALevel(RF24_PA_LOW, false);
  rf24.setDataRate(RF24_1MBPS);
  rf24.enableDynamicPayloads();
  rf24.setAutoAck(true);
  rf24.setChannel(rf24Channel);
  rf24.openWritingPipe(rf24Address);
  rf24.openReadingPipe(1, rf24Address);
  rf24.startListening();
}

void Communication::UpdateRF24()
{
  if (!isRF24Available)
    return;

  if (!rf24.available())
    return;

  int inDataCount = rf24.getDynamicPayloadSize();

  if (inDataCount > 0)
  {
    rf24.read(rf24InData, inDataCount);

    for (int index = 0; index < inDataCount; index++)
    {
      if (rf24InData[index] == Orders::transStart)
      {
        if (rf24InData[index + 1] != Orders::requestEcho)
          HandleOrder(rf24InData + index, OrderSource::FromRF24);
      }
    }
  }
}

bool Communication::SendESP8266(byte muxId, byte *buffer, unsigned int length)
{
//  Serial.print("Send: "); // For debugging.

  esp8266Serial.println("AT+CIPSEND=" + String(muxId) + "," + String(length));  // Request send data.
  if (ReceiveESP8266(">", 0, 10).length() == 0) // Didn't get ready to send data.
  {
//    Serial.println("Failed: Didn't receive: >");  // For debugging.
    return false;
  }

  for (unsigned int i = 0; i < length; i++) // Send data.
  {
//   Serial.print(buffer[i]); Serial.print(", "); // For debugging.
    esp8266Serial.write(buffer[i]);
  }

  if (ReceiveESP8266("SEND OK").length() == 0)  // Send failed.
  {
//    Serial.println("Failed: Didn't receive: SEND OK");  // For debugging.
    return false;
  }

//  Serial.println("Success");  // For debugging.
  return true;
}

String Communication::ReceiveESP8266(String endWord, unsigned int endLength, unsigned long timeOut)
{
  String string;
  unsigned long lastMillis = millis();

  while (true)  // Receive from ESP8266 module until get end word or timeout.
  {
    string += esp8266Serial.readStringUntil('\n');  // Only receive one line of message at a time. To avoid receiving messages that do not belong to this command.
    if (string.endsWith("\r")) string += "\n";  // '\r' and '\n' always appear at the end of a line of message at the same time.
    if (string.indexOf(endWord, endLength == 0 ? 0 : string.length() - endLength) != -1)  // Received end word.
    {   // If request reset, part of received messages will be in 74800 baud. indexOf() can't find end word then, but use fromIndex will work.
//      Serial.print(string); // For debugging.
      return string;
    }
    if (millis() - lastMillis > timeOut)  // Time out.
    {
//      Serial.print(string); // For debugging.
      return "";
    }
  }
}

void Communication::StartESP8266()
{ // This function takes about 4s. Then Wi-Fi hotspot will be available.
//  Serial.println("StartESP8266() start"); // For debugging.

  esp8266Serial.begin(115200);  // ESP8266 module uses 115200 baud rate by default.
  esp8266Serial.setTimeout(1);  // Set a small value to save time.
  /* Some ESP8266 modules may not return "ready".
  esp8266Serial.println("AT+RESTORE");    // Request restore.
  if (ReceiveESP8266("ready", 7, 3000).length() == 0) return;
  */
  if (esp8266CHL == 0)  // If do not specify an AP (Wireless Access Point) channel, get most unblocked AP channel.
  {
    esp8266Serial.println("AT+CWMODE_CUR=1"); // Set to Station mode. To scan AP.
    if (ReceiveESP8266("OK").length() == 0) return;
    /* Some ESP8266 modules may return "ERROR".
    esp8266Serial.println("AT+CWLAPOPT=0,20");// Set only receive AP's Channel and RSSI (Received Signal Strength Indication).
    if (ReceiveESP8266("OK").length() == 0) return;
    */
    esp8266Serial.println("AT+CWLAP");        // Scan all AP.
    String apList = ReceiveESP8266("OK", 0, 3000);
    if (apList.length() == 0) return;

    int channelRssi[13];          // 13 channels available. 1~13.
    for (unsigned int index = 0; index < sizeof(channelRssi) / sizeof(int); index++)
      channelRssi[index] = -100;  // RSSI is from -100 to -55. -100 means the weakest signal and - 55 means the strongest signal.

    for (unsigned int index = 0; index < apList.length();)  // Formatting data.
    {
      int index0 = apList.indexOf('(', index); if (index0 == -1) break;
      int index1 = apList.indexOf(',', index);
      int index2 = apList.indexOf(')', index);

      int rssi = apList.substring(index0 + 1, index1).toInt();
      int channel = apList.substring(index1 + 1, index2).toInt();

      channelRssi[channel - 1] = max(channelRssi[channel - 1], rssi);

      index = index2 + 1;
    }

    int bestChannel = 1;
    int bestChannelRssi = -55;
    for (unsigned int index = 0; index < sizeof(channelRssi) / sizeof(int); index++)  // Find the minimum.
      if (channelRssi[index] < bestChannelRssi)
      {
        bestChannel = index + 1;
        bestChannelRssi = channelRssi[index];
      }

    esp8266CHL = bestChannel;
  }

  esp8266Serial.println("AT+CWMODE_CUR=2"); // Set to SoftAP mode. To creat an AP.
  if (ReceiveESP8266("OK").length() == 0) return;

  esp8266Serial.println("AT+CWSAP_CUR=\"" + esp8266SSID + "\",\"" + esp8266PWD + "\"," + String(esp8266CHL) + "," + "4"); // Creat an AP.
  if (ReceiveESP8266("OK").length() == 0) return;

  esp8266Serial.println("AT+CIPMUX=1");     // Set multi connection mode.
  if (ReceiveESP8266("OK").length() == 0) return;

  esp8266Serial.println("AT+CIPSERVER=1," + String(esp8266Port)); // Creat an TCP Sever .
  if (ReceiveESP8266("OK").length() == 0) return;

  esp8266Serial.println("AT+CIPSTO=0");     // Set the timeout. 0 means no limit.
  if (ReceiveESP8266("OK").length() == 0) return;

  isESP8266Available = true;

//  Serial.println("StartESP8266() end"); // For debugging.
}

void Communication::UpdateESP8266()
{
  if (!isESP8266Available)
    return;

  while (esp8266Serial.available() > 0)
  {
    byte inByte = esp8266Serial.read();

    if (inByte == Orders::transStart && esp8266InDataCounter > 8) // Received start byte and a complete beginning of the message.
    {
      esp8266ClientID = esp8266InData[esp8266InDataCounter - 4] - '0';  // Get Client ID
      esp8266InDataCounter = 9; // Remove the last incomplete message.
    }

    esp8266InData[esp8266InDataCounter++] = inByte;

    if (inByte == Orders::transEnd && esp8266InData[9] == Orders::transStart) // Received end byte and a complete order.
    {
      HandleOrder(esp8266InData + 9, OrderSource::FromESP8266);
      esp8266InDataCounter = 0;
    }

//    Serial.print((char)inByte); // For debugging.
  }
}

void Communication::HandleOrder(byte inData[], OrderSource orderSource)
{
  if (blockedOrder != 0)
    return;

  this->orderSource = orderSource;

  byte outData[outDataSize];
  byte outDataCounter = 0;

  outData[outDataCounter++] = Orders::transStart;

  if (inData[1] == Orders::requestEcho)
  {
    outData[outDataCounter++] = Orders::echo;
  }
  else if (inData[1] == Orders::requestSupplyVoltage)
  {
    float supplyVoltage = GetSupplyVoltage();
    outData[outDataCounter++] = Orders::supplyVoltage;
    outData[outDataCounter++] = (int)(supplyVoltage * 100) / 128;
    outData[outDataCounter++] = (int)(supplyVoltage * 100) % 128;
  }
  else if (inData[1] == Orders::requestChangeIO)
  {
    digitalWrite(pins[inData[2]], inData[3]);
    outData[outDataCounter++] = Orders::orderDone;
  }
  else if (inData[1] == Orders::requestMoveLeg)
  {
    robotAction.LegMoveToRelativelyDirectly(inData[2], Point(inData[3] - 64, inData[4] - 64, inData[5] - 64));
    outData[outDataCounter++] = Orders::orderDone;
  }
  else if (inData[1] == Orders::requestCalibrate)
  {
    robotAction.robot.CalibrateServos();
    outData[outDataCounter++] = Orders::orderDone;
  }
  else if (inData[1] >= 64 && inData[1] <= 108)
  {
    blockedOrder = inData[1];
    outData[outDataCounter++] = Orders::orderStart;
  }
  else if (inData[1] == Orders::requestCrawl)
  {
    blockedOrder = inData[1];
    crawlParameters[0] = inData[2];
    crawlParameters[1] = inData[3];
    crawlParameters[2] = inData[4];
    outData[outDataCounter++] = Orders::orderStart;
  }
  else if (inData[1] == Orders::requestChangeBodyHeight)
  {
    blockedOrder = inData[1];
    changeHeightParameters[0] = inData[2];
    outData[outDataCounter++] = Orders::orderStart;
  }
  else if (inData[1] == Orders::requestMoveBody)
  {
    blockedOrder = inData[1];
    moveBodyParameters[0] = inData[2];
    moveBodyParameters[1] = inData[3];
    moveBodyParameters[2] = inData[4];
    outData[outDataCounter++] = Orders::orderStart;
  }
  else if (inData[1] == Orders::requestRotateBody)
  {
    blockedOrder = inData[1];
    rotateBodyParameters[0] = inData[2];
    rotateBodyParameters[1] = inData[3];
    rotateBodyParameters[2] = inData[4];
    outData[outDataCounter++] = Orders::orderStart;
  }
  else if (inData[1] == Orders::requestTwistBody)
  {
    blockedOrder = inData[1];
    twistBodyParameters[0] = inData[2];
    twistBodyParameters[1] = inData[3];
    twistBodyParameters[2] = inData[4];
    twistBodyParameters[3] = inData[5];
    twistBodyParameters[4] = inData[6];
    twistBodyParameters[5] = inData[7];
    outData[outDataCounter++] = Orders::orderStart;
  }
  outData[outDataCounter++] = Orders::transEnd;

  if (orderSource == OrderSource::FromSerial)
    Serial.write(outData, outDataCounter);
  else if (orderSource == OrderSource::FromESP8266)
    SendESP8266(esp8266ClientID, outData, outDataCounter);
}

void Communication::UpdateBlockedOrder()
{
  byte blockedOrder = this->blockedOrder;
  if (blockedOrder == 0)
    return;

  lastBlockedOrderTime = millis();

  orderState = OrderState::ExecuteStart;

  if (blockedOrder == Orders::requestCrawlForward)
  {
    SaveRobotBootState(Robot::State::Boot);
    robotAction.CrawlForward();
  }
  else if (blockedOrder == Orders::requestCrawlBackward)
  {
    SaveRobotBootState(Robot::State::Boot);
    robotAction.CrawlBackward();
  }
  else if (blockedOrder == Orders::requestCrawlLeft)
  {
    SaveRobotBootState(Robot::State::Boot);
    robotAction.CrawlLeft();
  }
  else if (blockedOrder == Orders::requestCrawlRight)
  {
    SaveRobotBootState(Robot::State::Boot);
    robotAction.CrawlRight();
  }
  else if (blockedOrder == Orders::requestTurnLeft)
  {
    SaveRobotBootState(Robot::State::Boot);
    robotAction.TurnLeft();
  }
  else if (blockedOrder == Orders::requestTurnRight)
  {
    SaveRobotBootState(Robot::State::Boot);
    robotAction.TurnRight();
  }
  else if (blockedOrder == Orders::requestActiveMode)
  {
    SaveRobotBootState(Robot::State::Boot);
    robotAction.ActiveMode();
  }
  else if (blockedOrder == Orders::requestSleepMode)
  {
    SaveRobotBootState(Robot::State::Boot);
    robotAction.SleepMode();
  }
  else if (blockedOrder == Orders::requestSwitchMode)
  {
    SaveRobotBootState(Robot::State::Boot);
    robotAction.SwitchMode();
  }
  else if (blockedOrder == Orders::requestInstallState)
  {
    SaveRobotBootState(Robot::State::Install);
    robotAction.robot.InstallState();
  }
  else if (blockedOrder == Orders::requestCalibrateState)
  {
    SaveRobotBootState(Robot::State::Calibrate);
    robotAction.robot.CalibrateState();
  }
  else if (blockedOrder == Orders::requestBootState)
  {
    SaveRobotBootState(Robot::State::Boot);
    robotAction.robot.BootState();
  }
  else if (blockedOrder == Orders::requestCalibrateVerify)
  {
    robotAction.robot.CalibrateVerify();
  }
  else if (blockedOrder == Orders::requestCrawl)
  {
    SaveRobotBootState(Robot::State::Boot);
    float x = crawlParameters[0] - 64;
    float y = crawlParameters[1] - 64;
    float angle = crawlParameters[2] - 64;
    robotAction.Crawl(x, y, angle);
  }
  else if (blockedOrder == Orders::requestChangeBodyHeight)
  {
    SaveRobotBootState(Robot::State::Boot);
    float height = changeHeightParameters[0] - 64;
    robotAction.ChangeBodyHeight(height);
  }
  else if (blockedOrder == Orders::requestMoveBody)
  {
    SaveRobotBootState(Robot::State::Boot);
    float x = moveBodyParameters[0] - 64;
    float y = moveBodyParameters[1] - 64;
    float z = moveBodyParameters[2] - 64;
    robotAction.MoveBody(x, y, z);
  }
  else if (blockedOrder == Orders::requestRotateBody)
  {
    SaveRobotBootState(Robot::State::Boot);
    float x = rotateBodyParameters[0] - 64;
    float y = rotateBodyParameters[1] - 64;
    float z = rotateBodyParameters[2] - 64;
    robotAction.RotateBody(x, y, z);
  }
  else if (blockedOrder == Orders::requestTwistBody)
  {
    SaveRobotBootState(Robot::State::Boot);
    float xMove = twistBodyParameters[0] - 64;
    float yMove = twistBodyParameters[1] - 64;
    float zMove = twistBodyParameters[2] - 64;
    float xRotate = twistBodyParameters[3] - 64;
    float yRotate = twistBodyParameters[4] - 64;
    float zRotate = twistBodyParameters[5] - 64;
    robotAction.TwistBody(Point(xMove, yMove, zMove), Point(xRotate, yRotate, zRotate));
  }

  this->blockedOrder = 0;
  orderState = OrderState::ExecuteDone;
}

void Communication::CheckBlockedOrder()
{
  if (orderState != OrderState::ExecuteDone)
    return;

  byte outData[outDataSize];
  byte outDataCounter = 0;

  outData[outDataCounter++] = Orders::transStart;
  outData[outDataCounter++] = Orders::orderDone;
  outData[outDataCounter++] = Orders::transEnd;

  if (orderSource == OrderSource::FromSerial)
    Serial.write(outData, outDataCounter);
  else if (orderSource == OrderSource::FromESP8266)
    SendESP8266(esp8266ClientID, outData, outDataCounter);

  orderState = OrderState::ExecuteNone;
}

void Communication::UpdateAutoSleep()
{
  if (lastBlockedOrderTime != 0)
  {
    if (millis() - lastBlockedOrderTime > autoSleepOvertime)
    {
      if (robotAction.robot.state == Robot::State::Action)
        robotAction.SleepMode();
      lastBlockedOrderTime = 0;
    }
  }
}

void Communication::UpdateCommunication()
{
  UpdateStateLED();

  if (commFunction)
  {
    UpdateSerial();
    UpdateRF24();
    UpdateESP8266();
    CheckBlockedOrder();
  }
}

void Communication::UpdateOrder()
{
  UpdateBlockedOrder();
  UpdateAutoSleep();
}

void Communication::UpdateStateLED()
{
  if (ledCounter / ledBlinkCycle < abs(ledState))
  {
    if (ledCounter % ledBlinkCycle == 0)
      SetStateLed(ledState > 0 ? HIGH : LOW);
    else if (ledCounter % ledBlinkCycle == ledBlinkCycle / 2)
      SetStateLed(ledState > 0 ? LOW : HIGH);
  }

  ledCounter++;

  if (ledCounter / ledBlinkCycle >= abs(ledState) + (unsigned int)3)
  {
    ledCounter = 0;

    if (robotAction.robot.state == Robot::State::Action)
      ledState = 1;
    else if (robotAction.robot.state == Robot::State::Boot)
      ledState = 1;
    else if (robotAction.robot.state == Robot::State::Calibrate)
      ledState = 2;
    else if (robotAction.robot.state == Robot::State::Install)
      ledState = 3;

    if (robotAction.robot.power.powerGroupAutoSwitch)
      if (!robotAction.robot.power.powerGroupState)
        ledState = -1;
  }
}

float Communication::GetSupplyVoltage()
{
  return robotAction.robot.power.voltage;
}

void Communication::SaveRobotBootState(Robot::State state)
{
  byte stateByte = 0;

  if (state == Robot::State::Install)
    stateByte = 0;
  else if (state == Robot::State::Calibrate)
    stateByte = 1;
  else if (state == Robot::State::Boot)
    stateByte = 2;

  if (EEPROM.read(EepromAddresses::robotState) != stateByte)
    EEPROM.write(EepromAddresses::robotState, stateByte);
}

Robot::State Communication::GetRobotBootState()
{
  byte stateByte = EEPROM.read(EepromAddresses::robotState);
  Robot::State state;

  if (stateByte == 0)
    state = Robot::State::Install;
  else if (stateByte == 1)
    state = Robot::State::Calibrate;
  else if (stateByte == 2)
    state = Robot::State::Boot;
  else
    state = Robot::State::Install;

  return state;
}

void Communication::SetRobotBootState(Robot::State state)
{
  if (state == Robot::State::Install)
    robotAction.robot.InstallState();
  else if (state == Robot::State::Calibrate)
    robotAction.robot.CalibrateState();
  else if (state == Robot::State::Boot)
    robotAction.robot.BootState();
}

void UpdateService()
{
  sei();

  if (communication != NULL) {
    communication->robotAction.robot.Update();
    communication->UpdateCommunication();
  }
}

#endif
