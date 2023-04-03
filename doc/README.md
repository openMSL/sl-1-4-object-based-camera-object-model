# SL 1-4 Object Based Camera Object Model 

## Model Description

This model is a parameterizable object based video perception sensor and tracking model using the interface OSI. The model was developed in the project SetLevel by Bosch. 

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
