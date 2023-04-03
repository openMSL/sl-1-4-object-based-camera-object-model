# SL 1-4 Object Based Camera Object Model

## Model Description

This model is a parameterizable object based video perception sensor and tracking model using the interface OSI. 
The model was developed in the project SetLevel by Bosch. 

## Modeling Approach

### Modeling Framework / Model Overall Structure

The "object based camera object model" is based on object lists and all modeling is performed on object level.
The model output are object lists for OSI SenorData moving and stationary objects.
The outer layer of the model is the OSI Sensor Model Packaging (OSMP).
It specifies ways in which models  using the Open Simulation Interface (OSI) are to be packaged for their use in simulation environments using FMI 2.0.
For more detailed information see the official documentation.
The actual logic of the model is packed in a so called strategy.
The apply function of the strategy is called by the do_calc function of the OSMPFramework.
The strategy itself is structured into four modules as shown in the image below.
<img src="img\2020-11-25_08h21_52.png" width="800" />

### Modeling Approach, general info

The first module in the figure above brings the received ground truth stationary and moving objects 
(potentially also traffic signs and traffic lights from sensor_view.global_ground_truth) into a common format.
This enables iterations over all objects regardless of classification.
Then they are transformed to the sensor coordinate system for the following calculations.
In the last module, the tracked objects are transformed to the virtual sensor coordinate system (here: vehicle coordinate system) and the fields in the sensor data requested by the HADf are filled.

### Modeling of Specific Effects

It includes typical sensor artifacts like 

- soft FoV transitions
- different detection ranges for different targets
- occlusion effects depending on the sensor technology
- existence probability 
- tracker simulation 

The detection of moving objects, stationary objects, traffic signs and traffic lights is implemented. 

## Model Parameterization

Parametrizable parameters are
 
- FOV
- mounting position 
- view distance 

## Inferfaces

Configuration:

- Windows 10
- Visual Studio 17
- CMake 3.10.2
- OSI 3.2.0
- Protobuf 3.11.4

## Configuration and installation

Settings:
OSI, Protobuf and the Modul have to be build with the same configuration:
Debug/Release-Build
Project-Properties >> C/C++ >> Code-Generation >> Runtime Library
Multi-threaded Debug DLL (/MDd) for all solutions
under Project-Properties >> VC++ Directories: set the correct include paths
in CMake set Protobuf_INCLUDE_DIRS and Protobuf_LIBRARIES

## Build Instructions in Windows 

When building and installing, the framework will build an fmu package, which can be used with a simulation tool like CarMaker, dSpace ASM or others.

## Build Instructions in Ubuntu 

Testet with Ubuntu 16.04. Install the modules OSI and Protobuf. 

## Licensing

The work created by Robert Bosch GmbH is licensed under the terms of the Mozilla Public License, v. 2.0

## Disclaimers

This sensor model is very basic and not validated to provide a realistic sensor behaviour. 
The model shows generic behaviour and is thus not sensor specific. This is a prototype. Use at own risk. 
Aboslutely not warranty is given for functionality or statements made and we are not liable in case of any damage.

## Acknowledgements and Credits

This work received funding from the research project 
"[SET Level](https://setlevel.de/)" of the [PEGASUS](https://pegasus-family.de) project family,
 promoted by the German Federal Ministry for Economic Affairs and Energy based on a decision of the German Bundestag.
 
| SET Level                                                                                                | PEGASUS Family                                                                                                       | BMWi                                                                                                                                                                                 |
|----------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| <a href="https://setlevel.de"><img src="https://setlevel.de/assets/logo-setlevel.svg" width="100" /></a> | <a href="https://pegasus-family.de"><img src="https://setlevel.de/assets/logo-pegasus-family.svg" width="100" /></a> | <a href="https://www.bmwi.de/Redaktion/DE/Textsammlungen/Technologie/fahrzeug-und-systemtechnologien.html"><img src="https://setlevel.de/assets/logo-bmwi-en.svg" width="100" /></a> |

list of relevant additional credits
