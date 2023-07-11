# Hexapod
My project is the Hexapod robot, which is basically a large six-legged "bug" that is able to be remotely controlled. It has many servos that allow each joint of its arm to move independently. In addition, it has two Arduinos that control all of its functions via code downloaded from my computer. It physicaly cannot run into a wall, via an ultrasonic sensor on the front, and can take spy camera pictures!

| **Engineer** | **School** | **Area of Interest** | **Grade** |
|:--:|:--:|:--:|:--:|
| Arjun S | Mountain View High School | Mechanical/Electrical Engineering | Incoming Freshman |



# Final Milestone

<iframe width="1512" height="696" src="https://www.youtube.com/embed/fmRgQdf--kc?list=PLe-u_DjFx7evbB-xhja9iGMLTbCZXLQRI" title="Arjun S. Final Milestone" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

  For my final milestone, I did A LOT. I made four massive modifications, and made sure they all worked! My mods were as follows: An ultrasonic sensor to make sure it can't run into walls, a wooden platform to keep the battery from squishing my components, a 3D printed case for the remote, and a camera to take pictures of the surroundings.

  The ultrasonic sensor was definitely one of the hardest modifications to pull off. The code itself was simple, but implementing it among the thousands of lines of code already in the robot was challenging, to say the least. One of the first problems I discovered was that the crawlForward() function in the code wasn't affecting the robot at all! In fact, if I commented out the entire file and then uploaded it, nothing changed! At this point I knew that what I was uploading was not what I had edited. In fact, I had been editing a copy of the actual library, not the library itself. After delving through my files, I found it in My Arduino folder. Inside the folder was another folder called Libraries, and in there were the files for the libraries. There, was a folder called src, and in there was the actual files I needed. Once  I uploaded the modified code, but it still didn't work! This time, the problem was that the ultrasonic sensor was returning a distance of zero every time!

  When I took the code outside of the main file, and just uploaded that to the ultrasonic  sensor, it worked fine! Even when I changed all of the variables, it wasn't working. Finally, I figured out that I needed to make duplicates of the file so that it could be properly called, and to add it in the .h files! Once I did that, it finally worked fine! 

  Next, I made a platform to hold up the battery. All I really had to do was cut a piece of wood, Attatch some velcro, and raise it up on some standoffs. However, It got in the way of future modifications, as you will see soon. The battery platform was the beginning of a tower that would grow to more than double the hexapod's overall height.

  The remote case is made up of a top and a bottom. The top is completely removeable, and while the bottom is removeable, it will probably stay on the remote for eternity. The CAD for the reomote case is <a href="https://cad.onshape.com/documents/7271dc6e933066dc4955ebfb/w/275ff894963fda49f0d49827/e/79d1dd127091306e85ff1543?renderMode=0&uiState=64adc5a71aca996d04ee6788"> here </a>. In the design, you will see that it is actually three parts -- I hot glued the top two parts together becasue the printing was slightly off. It took quite a few iterations before the remote case was fitting, but when it did, it looked quite nice. 

  Finally, I had the camera. The camera was also a large challenge to finish. I needed to improvise a lot, as I had to get a SD card shield to put on top of the Arduino Uno. That made all of the photos download to an SD card, which I'm pretty sure are compatible with all laptops. The code to download to the Arduino Uno is as follows:

```c++
// ArduCAM Mini demo (C)2018 Lee
// Web: http://www.ArduCAM.com
// This program is a demo of how to use the enhanced functions
// This demo was made for ArduCAM_Mini_2MP_Plus.
// It can  continue shooting and store it into the SD card  in JPEG format
// The demo sketch will do the following tasks
// 1. Set the camera to JPEG output mode.
// 2. Capture a JPEG photo and buffer the image to FIFO
// 3.Write the picture data to the SD card
// 5.close the file
//You can change the FRAMES_NUM count to change the number of the picture.
//IF the FRAMES_NUM is 0X00, take one photos
//IF the FRAMES_NUM is 0X01, take two photos
//IF the FRAMES_NUM is 0X02, take three photos
//IF the FRAMES_NUM is 0X03, take four photos
//IF the FRAMES_NUM is 0X04, take five photos
//IF the FRAMES_NUM is 0X05, take six photos
//IF the FRAMES_NUM is 0X06, take seven photos
//IF the FRAMES_NUM is 0XFF, continue shooting until the FIFO is full
//You can see the picture in the SD card.
// This program requires the ArduCAM V4.0.0 (or later) library and ArduCAM_Mini_2MP_Plus
// and use Arduino IDE 1.6.8 compiler or above

#include <Wire.h>
#include <ArduCAM.h>
#include <SPI.h>
#include <SD.h>
// #include "memorysaver.h"
//This demo can only work on OV5640_MINI_5MP_PLUS or OV5642_MINI_5MP_PLUS platform.
#if !(defined (OV2640_MINI_2MP_PLUS))
#error Please select the hardware platform and camera module in the ../libraries/ArduCAM/memorysaver.h file
#endif
#define   FRAMES_NUM    0x06
// set pin 7 as the slave select for the digital pot:
const int CS = 7;
#define SD_CS 10
bool is_header = false;
int total_time = 0;
#if defined (OV2640_MINI_2MP_PLUS)
ArduCAM myCAM( OV2640, CS );
#endif
uint8_t read_fifo_burst(ArduCAM myCAM);
void setup() {
  // put your setup code here, to run once:
  uint8_t vid, pid;
  uint8_t temp;
#if defined(__SAM3X8E__)
  Wire1.begin();
#else
  Wire.begin();
#endif
  Serial.begin(115200);
  Serial.println(F("ArduCAM Start!"));
  // set the CS as an output:
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
  // initialize SPI:
  SPI.begin();
  //Reset the CPLD
myCAM.write_reg(0x07, 0x80);
delay(100);
myCAM.write_reg(0x07, 0x00);
delay(100);
  while (1) {
    //Check if the ArduCAM SPI bus is OK
    myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
    temp = myCAM.read_reg(ARDUCHIP_TEST1);
    if (temp != 0x55)
    {
      Serial.println(F("SPI interface Error!"));
      delay(1000); continue;
    } else {
      Serial.println(F("SPI interface OK.")); break;
    }
  }
#if defined (OV2640_MINI_2MP_PLUS)
  while (1) {
    //Check if the camera module type is OV2640
    myCAM.wrSensorReg8_8(0xff, 0x01);
    myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
    myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
    if ((vid != 0x26 ) && (( pid != 0x41 ) || ( pid != 0x42 ))) {
      Serial.println(F("ACK CMD Can't find OV2640 module!"));
      delay(1000); continue;
    }
    else {
      Serial.println(F("ACK CMD OV2640 detected.")); break;
    }
  }
#endif
  //Initialize SD Card
  while (!SD.begin(SD_CS))
  {
    Serial.println(F("SD Card Error!")); delay(1000);
  }
  Serial.println(F("SD Card detected."));
  //Change to JPEG capture mode and initialize the OV5640 module
  myCAM.set_format(JPEG);
  myCAM.InitCAM();
  myCAM.clear_fifo_flag();
  myCAM.write_reg(ARDUCHIP_FRAMES, FRAMES_NUM);
}

void loop() {
  // put your main code here, to run repeatedly:
  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
#if defined (OV2640_MINI_2MP_PLUS)
  myCAM.OV2640_set_JPEG_size(OV2640_1600x1200);
#endif
  //Start capture
  myCAM.start_capture();
  Serial.println(F("start capture."));
  total_time = millis();
  while ( !myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
  Serial.println(F("CAM Capture Done."));
  total_time = millis() - total_time;
  Serial.print(F("capture total_time used (in miliseconds):"));
  Serial.println(total_time, DEC);
  total_time = millis();
  read_fifo_burst(myCAM);
  total_time = millis() - total_time;
  Serial.print(F("save capture total_time used (in miliseconds):"));
  Serial.println(total_time, DEC);
  //Clear the capture done flag
  myCAM.clear_fifo_flag();
  delay(100);
}
uint8_t read_fifo_burst(ArduCAM myCAM)
{
  uint8_t temp = 0, temp_last = 0;
  uint32_t length = 0;
  static int i = 0;
  static int k = 0;
  char str[16];
  File outFile;
  byte buf[256];
  length = myCAM.read_fifo_length();
  Serial.print(F("The fifo length is :"));
  Serial.println(length, DEC);
  if (length >= MAX_FIFO_SIZE) //8M
  {
    Serial.println("Over size.");
    return 0;
  }
  if (length == 0 ) //0 kb
  {
    Serial.println(F("Size is 0."));
    return 0;
  }
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();//Set fifo burst mode
  i = 0;
  while ( length-- )
  {
    temp_last = temp;
    temp =  SPI.transfer(0x00);
    //Read JPEG data from FIFO
    if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
    {
      buf[i++] = temp;  //save the last  0XD9
      //Write the remain bytes in the buffer
      myCAM.CS_HIGH();
      outFile.write(buf, i);
      //Close the file
      outFile.close();
      Serial.println(F("OK"));
      is_header = false;
      myCAM.CS_LOW();
      myCAM.set_fifo_burst();
      i = 0;
    }
    if (is_header == true)
    {
      //Write image data to buffer if not full
      if (i < 256)
        buf[i++] = temp;
      else
      {
        //Write 256 bytes image data to file
        myCAM.CS_HIGH();
        outFile.write(buf, 256);
        i = 0;
        buf[i++] = temp;
        myCAM.CS_LOW();
        myCAM.set_fifo_burst();
      }
    }
    else if ((temp == 0xD8) & (temp_last == 0xFF))
    {
      is_header = true;
      myCAM.CS_HIGH();
      //Create a avi file
      k = k + 1;
      itoa(k, str, 10);
      strcat(str, ".jpg");
      //Open the new file
      outFile = SD.open(str, O_WRITE | O_CREAT | O_TRUNC);
      if (! outFile)
      {
        Serial.println(F("File open failed"));
        while (1);
      }
      myCAM.CS_LOW();
      myCAM.set_fifo_burst();
      buf[i++] = temp_last;
      buf[i++] = temp;
    }
  }
  myCAM.CS_HIGH();
  return 1;
}
```
For the memorysaver.h file, download the ArduCAM folder in this link and add it to your libraries (add it to your libraries folder in your arduino folder):
https://github.com/ArduCAM/Arduino

  I had a lot of fun making the hexapod, and I definitely reccomend you to try this project yourself!

# Third Milestone

<iframe width="1038" height="584" src="https://www.youtube.com/embed/J3EQ6ztByLU?list=PLe-u_DjFx7evbB-xhja9iGMLTbCZXLQRI" title="Arjun S. Milestone 3" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

  For my third milestone, I connected the hexapod to my remote and made it control the robot. This step was actually quite simple. All I had to do was upload some code to the remote and it was done! However, a problem I immediately noticed was that all of the cables from the servos were inhibiting the movement of the limbs and in general were an eyesore. I used some sleeve cables to tie them up and keep everything from tangling up.

  While testing the hexapod and remote, I actually discovered a new feature! I realized that I could use one of the potentiometers, dials that change resistance based on how much it is turned, could change the height of the hexapod. The way it does this is that it measures the resistance. When the dial is turned, the resistance changes, and it measures that to change the height.

  Next, I want to make a case for the remote, so it is easier to carry. I also want to make a holder for the battery and use an ultrasonic sensor to make it not run into walls or crawl off cliffs. I think it would be also really  cool to attatch a mini camera so it could take video and wirelessly upload it to my computer. To sum it up, I have a lot of work ahead of me.

  For the remote case, I plan to 3D-print a shell for it, using either tinkercad or onshape. For the battery holder, I will probably use a platform and some velcro. Attatching the ultrasonic sensor might be challenging, as there aren't many pins on the arduino, and the mini camera will be probably the toughest of them all.

# Second Milestone

<iframe width="1008" height="567" src="https://www.youtube.com/embed/BZb4BqTWyt4?list=PLe-u_DjFx7evbB-xhja9iGMLTbCZXLQRI" title="Arjun S. Milestone 2" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

  For my second milestone, I finished my remote. It consists of the battery, joystick, wireless module, potentiometers, power button, power port, and arduino boards. The battery obviously provides power for the whole operation, allowing everything to run smoothly. The joystick is basically user input, measuring what the user, in this case me, wants to do and telling the arduino that. The wireless module is for sending the processed input to the hexapod. It is like a pipe from a chocolate factory to a grocery store or to someone's mouth. The joystick is like the raw cacao beans, and the arduino is like the actual factory that turns the cacao beans into processed chocolate, that is compatible with humans, basically the hexapod. 
  
  Really, assembling the remote was quite easy, but I learned a lot from it, like what a potentiometer is -- basically an adjustable resistor. I took the time to understand how the remote actually works, and I gained much understanding from it. I was able to identify many components on the wireless module, too! There was a crystal oscillator, which keeps time, and LEDs, light-emmitting-diodes.

  The top part of the remote is actually the "remote shield", and the actual processing area is at the bottom, the actual control board. There are two power ports, one on the remote shield, and one on the control board, but I now know that I should plug in the top one, as it has the power button that will actually affect on and off, instead of doing nothing. The remote shield takes input from the user, via the joystick and potentiometer, sends it to the control board below, which sends a signal, through the remote shield, to the wireless module, and from there to the hexapod.

# First Milestone

<iframe width="1008" height="567" src="https://www.youtube.com/embed/BxPI1ASxOl8?list=PLe-u_DjFx7evbB-xhja9iGMLTbCZXLQRI" title="Arjun S. Milestone 1" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

The Hexapod hardware consists of six main parts: 
- The battery
- The servos (with wires attatched)
- The servo disks
- The arduino control board
- The acryllic plates
- The screws

  Together, they make the awesome robot in the video below. Screws are used to screw servo disks to the acryllic plates, and the servo disks are later attatched to the actual servos. Since the disks are firmly screwed on to the acryllic plates, when the servo turns the disks, the acryllic moves with it. All of the wires from the servos are attatched at the bottom of the arduino control board, which is screwed on to the largest acryllic boards. Keep in mind that on some of the acrylic disks that are turned by servos, there are also servos on them, which in turn twist other acryllic plates, all connected to the first servo. The reason the servo disks are tight is because they are going through immense stress close to the body by holding together and lifting a lot of mass.

  Servos are basically motors but don't turn all the way around -- most only turn 180 to 270 degrees. However, the decreased rotation comes with increased control and fine tuning. With an intricate machine like the hexapod, precision is key to achieve a proper walking form.

  The main challenges I faced were definitely screwing in the servo disks and attatching the servos to their respective disks. There are 18 servos on the robot, and each of them attatches to a disk. Each of the disks take four screws to attatch, and each of them are really tight. In addition, after I finished screwing them in, it turned out that I put them in the wrong way! I had to unscrew each of the screws, turn around the servo disks, and rescrew the screws. Also, once I screwed all of them in the right way, I had to attatch the servos to them. The main challenge was that they had to be attatched in a specific orientation, since if they weren't attatched in the right way, the hexapod's range of motion would be compromised.

  Next, I want to make the remote and calibrate it to the hexapod, so it can control it. I also want to add a custom holder for the battery, a case for the remote, and other modifications that I will think of.


# Starter Project

<iframe width="1008" height="567" src="https://www.youtube.com/embed/ozKAnxqSFMM?list=PLe-u_DjFx7evbB-xhja9iGMLTbCZXLQRI" title="Arjun S. Starter Project" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

  My starter project was the digital clock. It can tell the time, temperature, date, and even wake you up with an alarm! The clock has a lot of intricate components, like the crystal oscilator, which tells time and seconds. The whole setup is controlled by two IC (integrated circuit) devices. They came precoded, but I had to install it all into the main board. There are two buttons on the top that allow the user to change values and view different things.

  The alarm music is played through the passive buzzer, which creates the sound. It is basically a speaker. For the alarm itself, you can choose your choice of music, and you can select any time for it to go off. In addition, the crystal oscillator keeps time in perfect sync, and after a few days of use it is still on time.

  There are quite a few Metal Film Resistors, of varying resistance, and they keep components from being overloaded or shorting out. In additions, there are multiple capacitors that temporarily store energy to be kept elsewhere. The thermistor, which changes resistance with temperature. The IC measures this change in resistance to determine the temperature. Finally, the diode at D1 only allows for flow of current in one direction, controlling the flow from going where it shouldn't.


# Schematics 
Here's where you'll put images of your schematics. [Tinkercad](https://www.tinkercad.com/blog/official-guide-to-tinkercad-circuits) and [Fritzing](https://fritzing.org/learning/) are both great resoruces to create professional schematic diagrams, though BSE recommends Tinkercad becuase it can be done easily and for free in the browser. 

  Steps to opening arduino libraries folder: Search Arduino in your file hub (finder for mac, windows button for windows). The folder should simply be named Arduino. In the Arduino folder, there should be a folder called libraries. Open that folder. In that folder are all of the files for your libraries. If you want to change them, simply open them and edit them. If you are looking to edit the hexapod code, open the FNHR folder (still in the libraries folder). In that, open the src folder. You can replace the entire src folder with the modified version included, or you can try to edit it on your own!

  If you want to add a library manually, you can simply drag and drop a folder to it. For example, if you want to add the ArduCAM folder, open the libraries folder, and simply drag and drop the Arducam folder included into the library.
  
# Code
I put the following code in the FNHR.cpp file in the src file of the library FNHR, used to control the robot. The full customizedcode I used to upload is included in this branch, but this is most of the basic code that made sure that the robot doesn't just run into a wall:

```c++

bool RobotAction::UltraSonicSensor(){
  Serial.println("alsdkfjaldkfja;ldkjfa");
  float ultraDistance = readSensorData();
  if (25 < ultraDistance){
    return true;
  }
  return false;
}

float RobotAction::readSensorData(){
  float soundDuration;
  // float sensorDistance;
  
  // Serial.print("trigPin: ");
  // Serial.println(trigPin);
  // Serial.print("echoPin: ");
  // Serial.println(echoPin);

  digitalWrite(trigPin, LOW);// trigPin is 3
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);
  soundDuration = pulseIn(echoPin, HIGH); // echoPin is 2
  sensorDistance = soundDuration * 0.034 / 2;
  // Serial.print("dur ");
  // Serial.println(soundDuration);
  Serial.print("dis ");
  Serial.println(sensorDistance);
  
  // Serial.println("final distance: ");
  // Serial.print(sensorDistance);

  return sensorDistance;
}
```


This is the code to put in the Arduino IDE:

```c++


#ifndef ARDUINO_AVR_MEGA2560
#error Wrong board. Please choose "Arduino/Genuino Mega or Mega 2560"
#endif

// Include FNHR (Freenove Hexapod Robot) library
#include <FNHR.h>

FNHR robot;
// Makes variables for the ports for the ultrasonic sensor:
const int trigPin = 3; 
const int echoPin = 2;

void setup() {
  // Defines the pins of the ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600); // Starts the Serial Monitor
  robot.Start(true);
}

void loop() {
  // Update Freenove Hexapod Robot
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  robot.Update();

}


```
# Bill of Materials
Here's where you'll list the parts in your project. To add more rows, just copy and paste the example rows below.
Don't forget to place the link of where to buy each component inside the quotation marks in the corresponding row after href =. Follow the guide [here]([url](https://www.markdownguide.org/extended-syntax/)) to learn how to customize this to your project needs. 

| **Part** | **Note** | **Price** | **Link** |
|:--:|:--:|:--:|:--:|
| FREENOVE Hexapod Kit with Remote | It's basically everything you need | $129.95 | <a href="https://www.amazon.com/Freenove-Raspberry-Crawling-Detailed-Tutorial/dp/B07FLVZ2DN/ref=sr_1_2?crid=VVKE0UP3OLNP&keywords=FREENOVE+hexapod+kit&qid=1687384706&sprefix=freenove+hexapod+ki%2Caps%2C139&sr=8-2"> Link </a> |
|:--:|:--:|:--:|:--:|
| Screwdriver Kit | Screwing in the screws (the screwdrivers that come with the kit are pretty bad) | $12.99 | <a href="https://www.amazon.com/Amartisan-10-Piece-Screwdrivers-Professional-Screwdriver/dp/B07RFD9JWZ/ref=sr_1_10?crid=MBSH1V7KS2VK&keywords=screwdriver&qid=1687384448&sprefix=screwdriver%2Caps%2C146&sr=8-10"> Link </a> |
|:--:|:--:|:--:|:--:|
| Tenergy NiMH 7.2 Volt Battery | This is a different battery than the one used in the tutorial, but it is safer. I custom soldered on a different connection for the battery. | $35.49 | <a href="https://power.tenergy.com/tenergy-nimh-7-2v-3000mah-battery-pack-w-tamiya-connector-for-rc-cars/"> Link </a> |
|:--:|:--:|:--:|:--:|
| 2-pack Ultrasonic Sensor | Detects if it will run into a wall. 2-pack in case one sensor doesn't work or burns out. | $5.99 | <a href="https://www.amazon.com/Ultrasonic-HC-SR04-Distance-Measuring-Transducer/dp/B077P72HG7/ref=asc_df_B077P72HG7/?tag=hyprod-20&linkCode=df0&hvadid=647333030603&hvpos=&hvnetw=g&hvrand=14530903250853092126&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9032171&hvtargid=pla-1956816726630&psc=1&gclid=CjwKCAjwkeqkBhAnEiwA5U-uM-mI5A-fgRsDspMcfOU3K3fKgvI9xuYYZmAC2xeCcsb7AeINN1U-thoC1KsQAvD_BwE"> Link </a> |
|:--:|:--:|:--:|:--:|
| Camera | Takes Photos | $25.99 | <a href="https://www.amazon.com/Arducam-Module-Megapixels-Arduino-Mega2560/dp/B012UXNDOY/ref=asc_df_B012UXNDOY/?tag=hyprod-20&linkCode=df0&hvadid=309807187084&hvpos=&hvnetw=g&hvrand=13377023022640676006&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9032171&hvtargid=pla-570327153549&psc=1"> Link </a> |
|:--:|:--:|:--:|:--:|
| Arduino Uno | Controls camera | $28.50 | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/ref=sr_1_1_sspa?crid=2JGP4NU4U1TEN&keywords=arduino+uno&qid=1689092480&s=industrial&sprefix=arduino+uno%2Cindustrial%2C175&sr=1-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1"> Link </a> |
|:--:|:--:|:--:|:--:|
| SD card shield | Adds SD card port (just goes straight on top of the Arduino Uno) | $7.69 | <a href="https://www.amazon.com/HiLetgo-Logging-Recorder-Logger-Arduino/dp/B00PI6TQWO/ref=asc_df_B00PI6TQWO/?tag=hyprod-20&linkCode=df0&hvadid=642049291626&hvpos=&hvnetw=g&hvrand=18392829009110491342&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9032171&hvtargid=pla-1948170026853&psc=1&gclid=CjwKCAjw-7OlBhB8EiwAnoOEkw_LXwZ0rSe95R4AOCAtjeEordPJutAuPZYig1Tl1gA3IOEdSDuZMBoC9_0QAvD_BwE"> Link </a> |
|:--:|:--:|:--:|:--:|
| Jumper Wires | Connects components; used for wiring -- this might be overkill | $5.58 | <a href="https://www.amazon.com/Elegoo-EL-CP-004-Multicolored-Breadboard-arduino/dp/B01EV70C78/ref=asc_df_B01EV70C78/?tag=hyprod-20&linkCode=df0&hvadid=222785939698&hvpos=&hvnetw=g&hvrand=9975027516624232934&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9032131&hvtargid=pla-362913641420&th=1"> Link </a> |
|:--:|:--:|:--:|:--:|
| 9V battery | Powers remote. Multiple batteries in case one dies. | $9.29 | <a href="https://www.amazon.com/AmazonBasics-Volt-Everyday-Alkaline-Battery/dp/B081FGCRQQ/ref=sr_1_6?keywords=9V+battery&qid=1689092767&sr=8-6"> Link </a> |
|:--:|:--:|:--:|:--:|
| Micro SD card w/ adapter | Stores photos and data from camera | $10.50 | <a href="https://www.amazon.com/Amazon-Basics-microSDXC-Memory-Adapter/dp/B08TJRVWV1/ref=sr_1_1_ffob_sspa?crid=2T4QPKZRSTSIG&keywords=micro+SD+card&qid=1689092924&sprefix=micro+sd+car%2Caps%2C244&sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1"> Link </a> |



# Other Resources
- [Tutorial and Files](https://freenove.com/fnk0031/)
  - this tutorial was instrumental in helping me understand what was going on and assembling everything correctly. One thing to note is to follow the tutorial to a tee, as if you don't you could skip steps that are actually really important and could have to restart. DO NOT SKIP STEPS!!!
  - The link will download a folder with all of the files needed for the project. For the tutorial itself, open the file Tutorial_for_V3.pdf
  - Keep in mind this is version three (V3) of the Hexapod project.
