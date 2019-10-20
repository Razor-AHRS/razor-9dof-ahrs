![Version tag](https://img.shields.io/badge/version-1.0.1-blue.svg)

# Introduction
This library is for the receiver end of the Razor 9 DOF IMU (reference  [here](https://github.com/Razor-AHRS/razor-9dof-ahrs/tree/master/Arduino/Razor_AHRS)).

# User guide
## Downloading the library
It is suggested that you download the entire repository and then select this folder, so that you can enjoy the benifits of VCS like git. It makes it simpler to update the contents whenever patch fixes are done. You can simply open a terminal (or gitbash on windows), go to the folder where you want to save this repository and type the following command.
```
git clone https://github.com/shashank3199/RazorIMU_9DOF
```

**Not recommended**: You can download _only_ this folder by clicking [here](https://minhaskamal.github.io/DownGit/#/home?url=https://github.com/shashank3199/RazorIMU_9DOF)

## Using the library with Arduino
Move this folder into the arduino libraries folder on your PC. If you don't know where the libraries folder of your arduino is, you can check out the README file of this entire repository for this, click [here](../README.md).<br>

## Prerequisites
Please follow the following steps before working with this library:
- Download the official code folder from [here](https://minhaskamal.github.io/DownGit/#/home?url=https://github.com/Razor-AHRS/razor-9dof-ahrs/tree/master/Arduino/Razor_AHRS), you'll be uploading this to the IMU.
- Extract folder *Razor_AHRS* to an appropriate location, open the *Razor_AHRS* file in Arduino IDE.
- Under the tools dropdown, select the following:
    - **Board**: Arduino Pro or Pro Mini
    - **Processor**: Atmega 328P (3.3V, 8MHz)
    - **Port**: The connected port (/dev/ttyUSB\* for Ubuntu, COM\* for windows).
- In the code of *Razor_AHRS* file, go to **USER SETUP AREA** and uncomment the correct #define HW\_\_VERSION\_CODE. It's 10736 in our examples, check the version from the correct hardware datasheet.
- Upload the code.
- Open serial monitor
- Set baud rate to 57600
- There must be output

More info about setting up software  [here](https://github.com/Razor-AHRS/razor-9dof-ahrs/wiki/Tutorial#setting-up-the-software).

## Usage of the library
In order to use this library, you must do the following:
- Include the header file `RazorIMU_9DOF.h` (the *RazorIMU_9DOF* library folder must be in your arduino libraries folder).
- Create an object of class `RazorIMU_9DOF`. You can pass the constructor parameters to initialize here itself, or leave it as a simple object declaration.
- Initialize the Serial stream on which the IMU is attached using the `AttachIMUSerial` function.
- To avoid wastage of resources, the library doesn't continuously poll the serial. You must call the function `UpdateData` for the library to fetch the values from the serial.
- To get the values, you can use one of the getter functions, for example if you want to access the YAW value, then you could use the function:  **GetYaw** which will give you the continuous YAW value.

# Developers Guide
This library has a single class named `RazorIMU_9DOF`. Let's explore all the contents in detail.

## Library Details

### Files in the library

#### RazorIMU_9DOF.h
The header file for the `RazorIMU_9DOF` class. It only has function declarations, not definitions.

#### RazorIMU_9DOF.cpp
This file consists the code for all the functions declared in the `RazorIMU_9DOF` class.

#### keywords.txt
This file consists the list of keywords and their types to be recognized by the Arduino IDE.

#### README.md
The description file that you're currently reading. All documentation here.

### Class contents
Let's explore the contents of the class, but first, we also have literals defined for general purpose use (using `#define`). They are:

| Name | Value | Purpose |
|:----:| :----: | :----- |
| PITCH | 0 | The index of pitch values in arrays |
| ROLL| 1 | The index of roll values in arrays |
| YAW | 2 | The index of yaw values in arrays |

Let's explore the class now

### Protected members

##### Variables
- **<font color="#CD00FF">float</font> YPR_values[3]**: Stores the values (-180 to 180, as output by the module) of Roll, Pitch and Yaw.
- **<font color="#CD00FF">Stream</font> \*IMU\_Serial**: This is the serial on which IMU operates. The parent **Stream** class allows any kind of serial, **Hardware** or **Software**.

### Public members
#### Constructors
- **<font color="#5052FF">RazorIMU_9DOF</font>()**: Empty constructor for the class.
- **<font color="#5052FF">RazorIMU_9DOF</font>(Stream \*AttachedSerial)**: To attach a pre-initialized serial to the IMU. This function calls the _AttachIMUSerial_ member function.

#### Member functions
- **<font color="#CD00FF">void</font> AttachIMUSerial(<font color="#FF00FF">Stream</font> \*AttachedSerial)**: Connect the IMU Serial.
- **<font color="#CD00FF">void</font> UpdateData()**: Updates the IMU readings stored in variables of the class.
- **<font color="#CD00FF">float</font> GetRoll()**: To get the continuous ROLL values.
- **<font color="#CD00FF">float</font> GetPitch()**: To get the continuous PITCH values.
- **<font color="#CD00FF">float</font> GetYaw()**: To get the continuous YAW values.

[![Developers Tag](https://img.shields.io/badge/Developer-shashank3199-red.svg)](https://github.com/shashank3199)