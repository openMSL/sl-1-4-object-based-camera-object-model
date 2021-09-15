# VideoSensorModel - Milestone 2

**Folderstructer "Project"/Documentation/... needs to remain unchanged becaus of relative links!**

## Documentation

This folder contains the documentation of the work done for Milestone 2 according to the Credible Modelling Process (CMP) which is documented [here](https://gitlab.sl4to5.de/deliverables/credible-simulation-process/credible-simulation-process/-/blob/0182678762e6e3f9910246913259ae5c9fa7313b/credible_simulation_process.md#introduction) . The documentation is devided in 4 seperate markdown files corresponding to the 4 process steps of the CMP, each containing the work done in this phase and having distinct inputs and outputs:

[**Phase 1: Analyze the Engineering Task**](Documentation/CMP_Phase1_Analyze.md)

[**Phase 2: Requirement Specification**](Documentation/CMP_Phase2_RequirementSpec.md)

[**Phase 3: Design Specification**](Documentation/CMP_Phase3_DesignSpec.md)

[**Phase 4: Implement and Assure quality for Simulation Setup**](Documentation/CMP_Phase4_Implementation.md)

## Model

This folder contains the actual model (Code, FMU, Documentation (how to use), etc)

## Metadata

**Pleas try to add as much metadata as possible in the [VideoSensorModel_Metadata.srmd](VideoSensorModel_Metadata.srmd)-File in this repo.**
(Metadata Template coresponding to [Model Metadata](https://gitlab.sl4to5.de/deliverables/data-management/metadata-format/-/tree/master/Metadata%20format%20for%20models)


## Bosch Camera Object Based Model with the OSI Sensor Model Packaging Framework

OSI Sensor Model Packaging specifies ways in which models (like e.g. environmental effect models, sensor models and logical models) using the [Open Simulation Interface (OSI)][] are to be packaged for their use in simulation environments using FMI 2.0.
For more detailed information see the [official documentation](https://opensimulationinterface.github.io/osi-documentation/osi-sensor-model-packaging/README.html).

[Open Simulation Interface (OSI)]: https://github.com/OpenSimulationInterface/open-simulation-interface

### Usage
The actual logic of the model is packed in a so called strategy. The apply function of the strategy is called by the do_calc-function of the OSMPFramework.

When building and installing, the framework will build an fmu package, which can be used with a simulation tool like CarMaker, dSpace ASM or others.

### Installation
##### Dependencies

Install `cmake` 3.10.2:
```bash
$ sudo apt-get install cmake
```
Install `protobuf` 3.0.0:
```bash
$ sudo apt-get install libprotobuf-dev protobuf-compiler
```


##### Clone OSI, Build and install
```bash
$ git submodule update --init
$ mkdir -p build
$ cd build
$ cmake ..
$ make
$ sudo make install
```

### Contents
[Interface Description](https://gitlab.sl4to5.de/pm/tp2/documents-tp2/-/blob/master/02_Durchfuehrung/AP2.1_Integrationsarchitektur/UAP211_schnittstellenarchitektur_gesamtsystem/06_sensormodelle_schnittstellen/video_object_based_sensor_system_model_interface.md)
##### Required Fields in OSI3 Sensor_View
- global_ground_truth.timestamp
- global_ground_truth.moving_object.id
- global_ground_truth.moving_object.base.position
- global_ground_truth.moving_object.base.orientation
- global_ground_truth.moving_object.base.orientation_rate
- global_ground_truth.moving_object.base.velocity
- global_ground_truth.moving_object.base.acceleration
- global_ground_truth.moving_object.base.dimension
- global_ground_truth.moving_object.type
- global_ground_truth.moving_object.vehicle_classification.type

- global_ground_truth.stationary_object.id
- global_ground_truth.stationary_object.base.position
- global_ground_truth.stationary_object.base.orientation
- global_ground_truth.stationary_object.base.dimension
- global_ground_truth.stationary_object.classification

- global_ground_truth.traffic_light.id
- global_ground_truth.traffic_light.base.position
- global_ground_truth.traffic_light.base.orientation
- global_ground_truth.traffic_light.base.dimension
- global_ground_truth.traffic_light.classification


##### Filled Fields in OSI3 Sensor_Data
- sensor_data.timestamp
- sensor_data.stationary_object_header.measurement_time
- sensor_data.stationary_object_header.set_cycle_counter
- sensor_data.stationary_object_header.set_data_qualifier
- sensor_data.moving_object_header.measurement_time
- sensor_data.moving_object_header.cycle_counter
- sensor_data.moving_object_header.data_qualifier
- sensor_data.moving_object.header.ground_truth_id
- sensor_data.moving_object.header.tracking_id
- sensor_data.moving_object.header.age
- sensor_data.moving_object.base.position
- sensor_data.moving_object.base.orientation
- sensor_data.moving_object.base.orientation_rate
- sensor_data.moving_object.base.velocity
- sensor_data.moving_object.base.acceleration
- sensor_data.moving_object.base.dimension
- sensor_data.moving_object.reference_point
- sensor_data.moving_object.movement_state
- sensor_data.moving_object.candidate.probability (set const to 1)
- sensor_data.moving_object.candidate.type



### Build Process
Known issues and solutions for building the FMU

##### 
Configuration:
*  Windows 10
*  Visual Studio 17
*  CMake 3.10.2
*  OSI 3.2.0
*  Protobuf 3.11.4

Settings:
* OSI, Protobuf and the Modul have to be build with the same configuration:
    * Debug/Release-Build
    * Project-Properties >> C/C++ >> Code-Generation >> Runtime Library 
    * Multi-threaded Debug DLL (/MDd) for all solutions
* under Project-Properties >> VC++ Directories: set the correct include paths
* in CMake set Protobuf_INCLUDE_DIRS and Protobuf_LIBRARIES 

##### Ubuntu
* Ubuntu 16.04

## Additional Information

