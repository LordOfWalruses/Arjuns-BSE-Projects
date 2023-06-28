# Hexapod
My project is the Hexapod robot, which is basically a large six-legged "bug" that is able to be remotely controlled. It has many servos that allow each joint of its arm to move independently. In addition, it has two Arduinos that controlls all of its functions via code downloaded from my computer. It physicaly cannot run into a wall, via an ultrasonic sensor on the front, and can take spy camera footage!

| **Engineer** | **School** | **Area of Interest** | **Grade** |
|:--:|:--:|:--:|:--:|
| Arjun S | Mountain View High School | Mechanical/Electrical Engineering | Incoming Freshman |

<!--
**Replace the BlueStamp logo below with an image of yourself and your completed project. Follow the guide [here](https://tomcam.github.io/least-github-pages/adding-images-github-pages-site.html) if you need help.**

![Headstone Image](logo.svg)
-->
# Final Milestone

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

<!--
# Schematics 
Here's where you'll put images of your schematics. [Tinkercad](https://www.tinkercad.com/blog/official-guide-to-tinkercad-circuits) and [Fritzing](https://fritzing.org/learning/) are both great resoruces to create professional schematic diagrams, though BSE recommends Tinkercad becuase it can be done easily and for free in the browser. 

# Code
Here's where you'll put your code. The syntax below places it into a block of code. Follow the guide [here]([url](https://www.markdownguide.org/extended-syntax/)) to learn how to customize it to your project needs. 

```c++
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Hello World!");
}

void loop() {
  // put your main code here, to run repeatedly:

}
```
-->
# Bill of Materials
Here's where you'll list the parts in your project. To add more rows, just copy and paste the example rows below.
Don't forget to place the link of where to buy each component inside the quotation marks in the corresponding row after href =. Follow the guide [here]([url](https://www.markdownguide.org/extended-syntax/)) to learn how to customize this to your project needs. 

| **Part** | **Note** | **Price** | **Link** |
|:--:|:--:|:--:|:--:|
| FREENOVE Hexapod Kit with Remote | It's basically everything you need | $129.95 | <a href="https://www.amazon.com/Freenove-Raspberry-Crawling-Detailed-Tutorial/dp/B07FLVZ2DN/ref=sr_1_2?crid=VVKE0UP3OLNP&keywords=FREENOVE+hexapod+kit&qid=1687384706&sprefix=freenove+hexapod+ki%2Caps%2C139&sr=8-2"> Link </a> |
|:--:|:--:|:--:|:--:|
| Screwdriver Kit | Screwing in the screws (the screwdrivers that come with the kit are pretty bad) | $12.99 | <a href="https://www.amazon.com/Amartisan-10-Piece-Screwdrivers-Professional-Screwdriver/dp/B07RFD9JWZ/ref=sr_1_10?crid=MBSH1V7KS2VK&keywords=screwdriver&qid=1687384448&sprefix=screwdriver%2Caps%2C146&sr=8-10"> Link </a> |
|:--:|:--:|:--:|:--:|
| Tenergy NiMH 7.2 Volt Battery | This is a different battery than the one used in the tutorial, but it is safer. I custom soddered on a different connection for the battery. | $35.49 | <a href="https://power.tenergy.com/tenergy-nimh-7-2v-3000mah-battery-pack-w-tamiya-connector-for-rc-cars/"> Link </a> |
|:--:|:--:|:--:|:--:|
| 2-pack Ultrasonic Sensor | Detects if it will run into a wall. 2-pack in case one sensor doesn't work or burns out. | $5.99 | <a href="https://www.amazon.com/Ultrasonic-HC-SR04-Distance-Measuring-Transducer/dp/B077P72HG7/ref=asc_df_B077P72HG7/?tag=hyprod-20&linkCode=df0&hvadid=647333030603&hvpos=&hvnetw=g&hvrand=14530903250853092126&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9032171&hvtargid=pla-1956816726630&psc=1&gclid=CjwKCAjwkeqkBhAnEiwA5U-uM-mI5A-fgRsDspMcfOU3K3fKgvI9xuYYZmAC2xeCcsb7AeINN1U-thoC1KsQAvD_BwE"> Link </a> |


# Other Resources
- [Tutorial and Files](https://freenove.com/fnk0031/)
  - this tutorial was instrumental in helping me understand what was going on and assembling everything correctly. One thing to note is to follow the tutorial to a tee, as if you don't you could skip steps that are actually really important and could have to restart. DO NOT SKIP STEPS!!!
  - The link will download a folder with all of the files needed for the project. For the tutorial itself, open the file Tutorial_for_V3.pdf
  - Keep in mind this is version three (V3) of the Hexapod project.
