# r1-models

This repository hosts the configuration files for generating R1Mk3 urdf.

## Installation

### Dependencies

Before installing `r1-models`, please be sure that you've installed [`YARP`](https://www.yarp.it/latest//) 3.10.0 or higher on your machine.

### Compiling from source

For installing it just:

```sh
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=<install-prefix> ..
make
(make install)
```

`R1Mk3` for `Gazebo Classic` simulation model needs a `gazebo-yarp-plugins` [latest master](https://github.com/robotology/gazebo-yarp-plugins/commit/c89280295d53279049bb7521cf6b6b3400130f23).

In order to use the model, the following env variables must be configured:
```sh
# R1Mk3 model in YARP
export YARP_DATA_DIRS=${YARP_DATA_DIRS}:<install-prefix>/share/R1Mk3
# R1Mk3 model in Gazebo
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:<install-prefix>/share/R1Mk3/robots
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:<install-prefix>/share/
```

In order to make the device `couplingXCubHandMk5` detectable, add `<installation_path>/share/yarp` to the `YARP_DATA_DIRS` environment variable of the system.

Alternatively, if `YARP` has been installed using the [robotology-superbuild](https://github.com/robotology/robotology-superbuild), it is possible to use `<directory-where-you-downloaded-robotology-superbuild>/build/install` as the `<installation_path>`.

To use `R1Mk3` models with `gz sim`, instead, you can rely on [gz-sim-yarp-plugins](https://github.com/robotology/gz-sim-yarp-plugins?tab=readme-ov-file#installation) and the following env variable mush be configured:

```sh
export GZ_SIM_RESOURCE_PATH = ${GZ_SIM_RESOURCE_PATH}:<install-prefix>/share/R1Mk3/robots
```

## URDF generation

To generate the URDF, you need to have access to the `cad-mechanics` repo (that is currently private, if you need access ask it to the r1-models mantainer) and install the following repos and software:
* You need to install the version of Creo required by [`cad-mechanics`](https://github.com/icub-tech-iit/cad-mechanics/).
* You need to install the repos that contain the CAD models, i.e. cad-libraries (see https://github.com/icub-tech-iit/cad-libraries/wiki/Configure-PTC-Creo-with-cad-libraries) and cad-mechanics https://github.com/icub-tech-iit/cad-mechanics/.
* You need to install creo2urdf following the README in https://github.com/icub-tech-iit/creo2urdf, either from source or using the binary available for each release.

>[!NOTE]
> For generating R1Mk3 URDF `creo2urdf` [v0.4.8](https://github.com/icub-tech-iit/creo2urdf/releases/tag/v0.4.8) or greater is needed

The CAD files used to generate the URDF models have been prepare according to the procedure described in https://github.com/icub-tech-iit/cad-libraries/wiki/Prepare-PTC-Creo-Mechanism-for-URDF .

You can find there the relative documentation on how write those configuration files, and more details in the README of the following folders:
* [`urdf/creo2urdf/data/R1Mk3`](./urdf/creo2urdf/data/R1Mk3/README.md)

## Maintainers
This repository is maintained by:

| | |
|:---:|:---:|
| [<img src="https://github.com/Nicogene.png" width="40">](https://github.com/Nicogene) | [@Nicogene](https://github.com/Nicogene) |
| [<img src="https://github.com/martinaxgloria.png" width="40">](https://github.com/martinaxgloria) | [@martinaxgloria](https://github.com/martinaxgloria) |
