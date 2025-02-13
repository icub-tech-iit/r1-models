# r1-models

## URDF generation

This repository hosts the configuration files for generating R1Mk3 urdf. To generate the URDF, you need to have access to the `cad-mechanics` repo (that is currently private, if you need access ask it to the r1-models mantainer) and install the following repos and software:
* You need to install the version of Creo required by [`cad-mechanics`](https://github.com/icub-tech-iit/cad-mechanics/).
* You need to install the repos that contain the CAD models, i.e. cad-libraries (see https://github.com/icub-tech-iit/cad-libraries/wiki/Configure-PTC-Creo-with-cad-libraries) and cad-mechanics https://github.com/icub-tech-iit/cad-mechanics/.
* You need to install creo2urdf following the README in https://github.com/icub-tech-iit/creo2urdf, either from source or using the binary available for each release.

>[!NOTE]
> For generating R1Mk3 URDF `creo2urdf` [v0.4.8](https://github.com/icub-tech-iit/creo2urdf/releases/tag/v0.4.8) or greater is needed

The CAD files used to generate the URDF models have been prepare according to the procedure described in https://github.com/icub-tech-iit/cad-libraries/wiki/Prepare-PTC-Creo-Mechanism-for-URDF .

You can find there the relative documentation on how write those configuration files, and more details in the README of the following folders:
* [`urdf/creo2urdf/data/R1Mk3`](./urdf/creo2urdf/data/R1Mk3/README.md)
