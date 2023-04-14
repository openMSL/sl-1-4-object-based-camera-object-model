# SL 1-4 Object Based Camera Object Model

[![Credibility Assessment Level 0](../../actions/workflows/cl0.yml/badge.svg)](https://github.com/openMSL/sl-1-5-sensor-model-testing/blob/main/doc/test_architecture.md#cl-0-license-check)
[![Credibility Assessment Level 1](../../actions/workflows/cl1.yml/badge.svg)](https://github.com/openMSL/sl-1-5-sensor-model-testing/blob/main/doc/test_architecture.md#cl-1-code-verification)
[![Credibility Assessment Level 2](../../actions/workflows/cl2.yml/badge.svg)](https://github.com/openMSL/sl-1-5-sensor-model-testing/blob/main/doc/test_architecture.md#cl-2-qualitative-verification)

This model is a parameterizable object based video perception sensor and tracking model using the interface OSI.
The model was developed in the project SetLevel by Bosch. The model should simulate some basic video typical effects in a phenomenological way.
The "object based camera object model" is based on object lists and all modeling is performed on object level. The model output are object lists for OSI SenorData moving and stationary objects.
The outer layer of the model is the OSI Sensor Model Packaging (OSMP).
It specifies ways in which models  using the Open Simulation Interface (OSI) are to be packaged for their use in simulation environments using FMI 2.0.
For more detailed information see the official documentation.
<img src="doc/img/Detection_Example_Video.png" width="800" />

## Modeling Approach

The actual logic of the model is packed in a so called strategy.
The apply function of the strategy is called by the do_calc function of the OSMPFramework.
The strategy itself is structured into four modules as shown in the image below.
<img src="doc/img/2020-11-25_08h21_52.png" width="800" />

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

## Parameterization

- FOV: describes the horizontal Field of View of the Camera System
- Range: deschribes the view distance of the sensor

## Interface

### Input: Required Fields in OSI3::SensorView

- `sensor_view.mounting_position`
- `sensor_view.global_ground_truth.timestamp`
- `sensor_view.global_ground_truth.host_vehicle_id`
- `sensor_view.global_ground_truth.stationary_object.id`
- `sensor_view.global_ground_truth.stationary_object.base.position`
- `sensor_view.global_ground_truth.stationary_object.base.orientation`
- `sensor_view.global_ground_truth.stationary_object.base.dimension`

### Output: Fields in OSI3::SensorData Filled by the Sensor Model

- `sensor_data.timestamp`
- `sensor_data.moving_object.header.ground_truth_id`
- `sensor_data.moving_object.header.tracking_id`
- `sensor_data.moving_object.header.existence_probability`
- `sensor_data.moving_object.header.measurement_state`
- `sensor_data.moving_object.header.sensor_id`
- `sensor_data.moving_object.base.position`
- `sensor_data.moving_object.base.dimension`

## Build Instructions

The following is an example for building a model as an FMU in Ubuntu.

### Build Model in Ubuntu 18.04 / 20.04

1. Clone this repository **with submodules**:

    ```bash
    git clone https://github.com/openMSL/your-model.git --recurse-submodules
    ```

2. Build the model by executing in the extracted project root directory:

    ```bash
    mkdir cmake-build
    cd cmake-build
    # If FMU_INSTALL_DIR is not set, CMAKE_BINARY_DIR is used
    cmake -DCMAKE_BUILD_TYPE=Release -DFMU_INSTALL_DIR:PATH=/tmp ..
    make
    ```

3. Take FMU from `FMU_INSTALL_DIR`

### Configuration for Windows

- Windows 10
- Visual Studio 2019
- CMake 3.10.2 or higher
- OSI 3.2.0
- Protobuf 3.11.4 or higher (tested with Protobuf 21.x)

1. Build protobuf for your special Visual Studio Version
2. Build sensor model with linker to build protobuf version
Settings:
OSI, Protobuf and the Modul have to be build with the same configuration:
Debug/Release-Build
Project-Properties >> C/C++ >> Code-Generation >> Runtime Library
Multi-threaded Debug DLL (/MDd) for all solutions
under Project-Properties >> VC++ Directories: set the correct include paths
in CMake set Protobuf_INCLUDE_DIRS and Protobuf_LIBRARIES

## Licensing

The work created by Robert Bosch GmbH is licensed under the terms of the Mozilla Public License, v. 2.0

## Disclaimers

This sensor model is very basic and not validated to provide a realistic sensor behaviour.
The model shows generic behaviour and is thus not sensor specific. This is a prototype. Use at own risk.
Aboslutely not warranty is given for functionality or statements made and we are not liable in case of any damage.

## Credits

This work received funding from the research project
"[SET Level](https://setlevel.de/)" of the [PEGASUS](https://pegasus-family.de) project family,
promoted by the German Federal Ministry for Economic Affairs and Energy based on a decision of the German Bundestag.

| SET Level                                                                                                | PEGASUS Family                                                                                                       | BMWi                                                                                                                                                                                 |
|----------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| <a href="https://setlevel.de"><img src="https://setlevel.de/assets/logo-setlevel.svg" width="100" /></a> | <a href="https://pegasus-family.de"><img src="https://setlevel.de/assets/logo-pegasus-family.svg" width="100" /></a> | <a href="https://www.bmwi.de/Redaktion/DE/Textsammlungen/Technologie/fahrzeug-und-systemtechnologien.html"><img src="https://setlevel.de/assets/logo-bmwi-en.svg" width="100" /></a> |

## References
