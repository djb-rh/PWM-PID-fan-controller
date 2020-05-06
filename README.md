# pid-controller

This is a sketch created using some sample code for clickButton, the SHT31 NCD device, the PID library, and some glue from me.

It actually does two things...it shows me the status of my gate controllers, which is probably useless for most folks.  But it 
should be easy to unmarry that from the more interesting task...PID control of PWM computer (PC) fans to cool to a target
temperature.  There are up/down buttons to set the target and it's displayed on the LCD. 

This is intended for a situation where the fan will actually be blowing through a heater core filled with water from a salt
water aquarium chiller. The chilled air will cool a space to the target temp by varying how much air blows through ONLY. The
chiller has a temp setting that can't be easily controlled, but that's fine, in my application I need most of the chilling
capability anyway.

My basic situation is a "tent" (that's inside) and is used for altitude training with a device that supplies de-oxygenated air.
That air is at a higher pressure than the tent can contain, so my CO2 and extra air from the tent is pushed out the seems. 
This simulates being at altitude quite well.  It's called hypoxic training.

Problem is, body heat and humidity build up in the tent if you sleep there overnight.  But you can't use a normal air conditioner
in the tent because those exchange air from the outside, which is no good here because it would defeat the de-oxygenating 
situation we have going.  So instead I have a chiller outside the tent along with a water reservoir that has a pump in it. The pump
circulates the water through the chiller and the car (in this case a Miata) heater core inside the tent.  Computer fans push
air through the core.  The core has to be over a pan to collect water that will condense on the core (I run it out the tent into
a bucket and you'll only get maybe a cup or so a day and you empty it every few days). 

It works great, but it's hard to find fan speeds that will maintain the same temperature overnight since my body is constantly
creating heat and the room temperature can vary some as well.

So I'm working on this controller.  As of this initial commit the controller appears to work.  Now to build my new exchanger
box (it'll have a HEPA filter on it, which my previous manually controlled one didn't) and tweak the code more.

# Future enhancements

- Store the last set temp in EEPROM
- add two more buttons to be able to control my gates
- get stabilized and remove all the gate code to a separate module somehow for folks that only want a basic PID controller


Everything below auto-added by Particle CLI

## Welcome to your project!

Every new Particle project is composed of 3 important elements that you'll see have been created in your project directory for pid-controller.

#### ```/src``` folder:  
This is the source folder that contains the firmware files for your project. It should *not* be renamed. 
Anything that is in this folder when you compile your project will be sent to our compile service and compiled into a firmware binary for the Particle device that you have targeted.

If your application contains multiple files, they should all be included in the `src` folder. If your firmware depends on Particle libraries, those dependencies are specified in the `project.properties` file referenced below.

#### ```.ino``` file:
This file is the firmware that will run as the primary application on your Particle device. It contains a `setup()` and `loop()` function, and can be written in Wiring or C/C++. For more information about using the Particle firmware API to create firmware for your Particle device, refer to the [Firmware Reference](https://docs.particle.io/reference/firmware/) section of the Particle documentation.

#### ```project.properties``` file:  
This is the file that specifies the name and version number of the libraries that your project depends on. Dependencies are added automatically to your `project.properties` file when you add a library to a project using the `particle library add` command in the CLI or add a library in the Desktop IDE.

## Adding additional files to your project

#### Projects with multiple sources
If you would like add additional files to your application, they should be added to the `/src` folder. All files in the `/src` folder will be sent to the Particle Cloud to produce a compiled binary.

#### Projects with external libraries
If your project includes a library that has not been registered in the Particle libraries system, you should create a new folder named `/lib/<libraryname>/src` under `/<project dir>` and add the `.h`, `.cpp` & `library.properties` files for your library there. Read the [Firmware Libraries guide](https://docs.particle.io/guide/tools-and-features/libraries/) for more details on how to develop libraries. Note that all contents of the `/lib` folder and subfolders will also be sent to the Cloud for compilation.

## Compiling your project

When you're ready to compile your project, make sure you have the correct Particle device target selected and run `particle compile <platform>` in the CLI or click the Compile button in the Desktop IDE. The following files in your project folder will be sent to the compile service:

- Everything in the `/src` folder, including your `.ino` application file
- The `project.properties` file for your project
- Any libraries stored under `lib/<libraryname>/src`
