# Hexapod
My project is the Hexapod robot, which is basically a large six-legged "bug" that is able to be remotely controlled. It has many servos that allow each joint of its arm to move independently. In addition, it has an Arduino that controlls all of its functions via code downloaded from my computer. 

| **Engineer** | **School** | **Area of Interest** | **Grade** |
|:--:|:--:|:--:|:--:|
| Arjun S | Mountain View High School | Electrical Engineering | Incoming Freshman |


**Replace the BlueStamp logo below with an image of yourself and your completed project. Follow the guide [here](https://tomcam.github.io/least-github-pages/adding-images-github-pages-site.html) if you need help.**

![Headstone Image](logo.svg)

# Final Milestone
For your final milestone, explain the outcome of your project. Key details to include are:
- What you've accomplished since your previous milestone
- What your biggest challenges and triumphs were at BSE
- A summary of key topics you learned about
- What you hope to learn in the future after everything you've learned at BSE

**Don't forget to replace the text below with the embedding for your milestone video. Go to Youtube, click Share -> Embed, and copy and paste the code to replace what's below.**


# Second Milestone
For your second milestone, explain what you've worked on since your previous milestone. You can highlight:
- Technical details of what you've accomplished and how they contribute to the final goal
- What has been surprising about the project so far
- Previous challenges you faced that you overcame
- What needs to be completed before your final milestone 

  For my second milestone, I finished my remote. It consists of the battery, joystick, wireless module, potentiometers, power button, power port, and arduino boards. The battery obviously provides power for the whole operation, allowing everything to run smoothly. The joystick is basically user input, measuring what the user, in this case me, wants to do and telling the arduino that. The wireless module is for sending the processed input to the hexapod. It is like a pipe from a chocolate factory to a grocery store or to someone's mouth. The joystick is like the raw cacao beans, and the arduino is like the actual factory that turns the cacao beans into processed chocolate, that is compatible with humans, basically the hexapod. 
  Really, assembling the remote was quite easy, but I learned a lot from it, like what a potentiometer is -- basically an adjustable resistor. I took the time to understand how the remote actually works, and I gained much understanding from it.

<iframe width="1008" height="567" src="https://www.youtube.com/embed/BZb4BqTWyt4?list=PLe-u_DjFx7evbB-xhja9iGMLTbCZXLQRI" title="Arjun S. Milestone 2" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

# First Milestone
For your first milestone, describe what your project is and how you plan to build it. You can include:
- An explanation about the different components of your project and how they will all integrate together
- Technical progress you've made so far
- Challenges you're facing and solving in your future milestones
- What your plan is to complete your project

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


<iframe width="1008" height="567" src="https://www.youtube.com/embed/BxPI1ASxOl8?list=PLe-u_DjFx7evbB-xhja9iGMLTbCZXLQRI" title="Arjun S. Milestone 1" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

# Starter Project

  My starter project was the digital clock. It can tell the time, temperature, date, and even wake you up with an alarm! The clock has a lot of intricate components, like the crystal oscilator, which tells time and seconds. The whole setup is controlled by two IC (integrated circuit) devices. They came precoded, but I had to install it all into the main board. There are two buttons on the top that allow the user to change values and view different things.

  The alarm music is played through the passive buzzer, which creates the sound. It is basically a speaker. For the alarm itself, you can choose your choice of music, and you can select any time for it to go off. In addition, the crystal oscillator keeps time in perfect sync, and after a few days of use it is still on time.

  There are quite a few Metal Film Resistors, of varying resistance, and they keep components from being overloaded or shorting out. In additions, there are multiple capacitors that temporarily store energy to be kept elsewhere. The thermistor, which changes resistance with temperature. The IC measures this change in resistance to determine the temperature. Finally, the diode at D1 only allows for flow of current in one direction, controlling the flow from going where it shouldn't.


<iframe width="1008" height="567" src="https://www.youtube.com/embed/ozKAnxqSFMM?list=PLe-u_DjFx7evbB-xhja9iGMLTbCZXLQRI" title="Arjun S. Starter Project" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>


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

# Bill of Materials
Here's where you'll list the parts in your project. To add more rows, just copy and paste the example rows below.
Don't forget to place the link of where to buy each component inside the quotation marks in the corresponding row after href =. Follow the guide [here]([url](https://www.markdownguide.org/extended-syntax/)) to learn how to customize this to your project needs. 

| **Part** | **Note** | **Price** | **Link** |
|:--:|:--:|:--:|:--:|
| FREENOVE Hexapod Kit with Remote | Its basically everything you need | $129.95 | <a href="(https://www.amazon.com/Freenove-Raspberry-Crawling-Detailed-Tutorial/dp/B07FLVZ2DN/ref=sr_1_2_sspa crid=1XVXZ88Y5IM6&keywords=freenove+hexapod&qid=1687384273&sprefix=freenove+hexapod%2Caps%2C137&sr=8-2-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1)"> Link </a> |
|:--:|:--:|:--:|:--:|
| Screwdriver Kit | Screwing in the screws (the screwdrivers that come with the kit are pretty bad | $12.99 | <a href="(https://www.amazon.com/Amartisan-10-Piece-Screwdrivers-Professional-Screwdriver/dp/B07RFD9JWZ/ref=sr_1_10?crid=MBSH1V7KS2VK&keywords=screwdriver&qid=1687384448&sprefix=screwdriver%2Caps%2C146&sr=8-10)"> Link </a> |
|:--:|:--:|:--:|:--:|
| Item Name | What the item is used for | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
|:--:|:--:|:--:|:--:|

# Other Resources/Examples
One of the best parts about Github is that you can view how other people set up their own work. Here are some past BSE portfolios that are awesome examples. You can view how they set up their portfolio, and you can view their index.md files to understand how they implemented different portfolio components.
- [Example 1](https://trashytuber.github.io/YimingJiaBlueStamp/)
- [Example 2](https://sviatil0.github.io/Sviatoslav_BSE/)
- [Example 3](https://arneshkumar.github.io/arneshbluestamp/)

To watch the BSE tutorial on how to create a portfolio, click here.
