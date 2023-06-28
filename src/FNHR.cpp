/*
 * File       Class for control Freenove Hexapod Robot
 * Author     Ethan Pan @ Freenove (support@freenove.com)
 * Date       2023/02/09
 * Version    V12.5
 * Copyright  Copyright © Freenove (http://www.freenove.com)
 * License    Creative Commons Attribution ShareAlike 3.0
 *            (http://creativecommons.org/licenses/by-sa/3.0/legalcode)
 * -----------------------------------------------------------------------------------------------*/

#if defined(ARDUINO_AVR_MEGA2560)

#include "FNHR.h"

FNHR::FNHR() {}

void FNHR::Start(bool commFunction)
{
  communication.Start(commFunction);
}

void FNHR::Update()
{
  if (communication.commFunction)
    communication.UpdateOrder();
}

void FNHR::SetWiFi(String name, String password)
{
  communication.SetWiFi(name, password);
}

void FNHR::SetWiFiChannel(byte channel)
{
  communication.SetWiFiChannel(channel);
}

void FNHR::SetRemote(byte byte0, byte byte1, byte byte2, byte byte3, byte byte4)
{
  communication.SetRemote(byte0, byte1, byte2, byte3, byte4);
}

void FNHR::SetRemoteChannel(byte channel)
{
  communication.SetRemoteChannel(channel);
}

void FNHR::ActiveMode()
{
  if (!communication.commFunction)
    communication.robotAction.ActiveMode();
}

void FNHR::SleepMode()
{
  if (!communication.commFunction)
    communication.robotAction.SleepMode();
}

void FNHR::SwitchMode()
{
  if (!communication.commFunction)
    communication.robotAction.SwitchMode();
}

void FNHR::CrawlForward()
{
  if (!communication.commFunction)
    communication.robotAction.CrawlForward();
}

void FNHR::CrawlBackward()
{
  if (!communication.commFunction)
    communication.robotAction.CrawlBackward();
}

void FNHR::CrawlLeft()
{
  if (!communication.commFunction)
    communication.robotAction.CrawlLeft();
}

void FNHR::CrawlRight()
{
  if (!communication.commFunction)
    communication.robotAction.CrawlRight();
}

void FNHR::TurnLeft()
{
  if (!communication.commFunction)
    communication.robotAction.TurnLeft();
}

void FNHR::TurnRight()
{
  if (!communication.commFunction)
    communication.robotAction.TurnRight();
}

void FNHR::Crawl(float x, float y, float angle)
{
  if (!communication.commFunction)
    communication.robotAction.Crawl(x, y, angle);
}

void FNHR::ChangeBodyHeight(float height)
{
  if (!communication.commFunction)
    communication.robotAction.ChangeBodyHeight(height);
}

void FNHR::MoveBody(float x, float y, float z)
{
  if (!communication.commFunction)
    communication.robotAction.MoveBody(x, y, z);
}

void FNHR::RotateBody(float x, float y, float z)
{
  if (!communication.commFunction)
    communication.robotAction.RotateBody(x, y, z);
}

void FNHR::TwistBody(float xMove, float yMove, float zMove, float xRotate, float yRotate, float zRotate)
{
  if (!communication.commFunction)
    communication.robotAction.TwistBody(Point(xMove, yMove, zMove), Point(xRotate, yRotate, zRotate));
}

void FNHR::LegMoveToRelatively(int leg, float x, float y, float z)
{
  if (!communication.commFunction)
    communication.robotAction.LegMoveToRelatively(leg, Point(x, y, z));
}

void FNHR::SetActionSpeed(float speed)
{
  if (!communication.commFunction)
    communication.robotAction.SetSpeedMultiple(speed / 100);
}

void FNHR::SetActionGroup(int group)
{
  if (!communication.commFunction)
    communication.robotAction.SetActionGroup(group);
}

#endif
