
NTN_IAB_Sim
=====================
## _Open source simulator developed in the context of the [SATIABLE][1] project that extends the functionality of the [5g-air-simulator][2] by giving the possibility to adopt IAB and satellite nodes_
 
---
### Table of Contents:
1. Getting NTN_IAB_Sim
2. Compiling NTN_IAB_Sim
3. Running NTN_IAB_Sim
4. Useful links
---
### 1. Getting NTN_IAB_Sim
NTN_IAB_Sim is available via Git at [this link][3]. 
To obtain NTN_IAB_Sim enter into the your prefered folder and write the following syntax:

    $ git clone https://github.com/telematics-lab/NTN_IAB_Sim.git
To synchronize the project repository with the local copy, you can run the pull sub-command. The syntax is as follows:

    $ git pull

###  2. Compiling NTN_IAB_Sim
First, you need to install make utility and the armadillo library.
On recent Linux systems, you can run:

    $  sudo apt install make libarmadillo-dev
Then you can build NTN_IAB_Sim with the following command:

	$ cd 5G-air-simulator; make
To clear the project, you can use the following command:

	$ make clean
To be sure of using the correct version of libarmadillo-dev, it can be directly installed from the .deb packages in the following way (Tested on Ubuntu 22.04):

    $ cd libraries
    $ sudo apt install ./libarmadillo9_9.800.4+dfsg-1build1_amd64.deb 
    $ sudo apt install ./libarmadillo-dev_9.800.4+dfsg-1build1_amd64.deb 

### 3. Running NTN_IAB_Sim
Several scenarios have been developed. To run a simple simulation and test that everything is working, you can use the following command:

	$ ./5G-air-simulator Simple
The scenarios developed in the project can be found in the folders:
- ./src/scenarios/P_001_REG
- ./src/scenarios/P_002_SENSOR_DATA
- ./src/scenarios/P_003_BB_DATA
- ./src/scenarios/P_004_SAT_NODE_SENSOR_BH
- ./src/scenarios/P_005_SAT_DONOR_BB_BH
- ./src/scenarios/V_003_MULTI_UE
- ./src/scenarios/V_004_SATELLITE

These scenarios can be launched by executing:

    $ ./5g-air-simulator <scenario_name> <parameters>
Details about the parameters can be found within the scenarios themselves. 

### 4. Useful links

1: [SATIABLE project page][1]

2: [5G-air-simulator Official Repository][2]

3: [NTN_IAB_Sim Official Repository][3]

[1]: https://romars.tech/project/satiable/ "SATIABLE project page"
[2]: https://github.com/telematics-lab/5G-air-simulator "5G-air-simulator Official Repository"
[3]: https://github.com/telematics-lab/NTN_IAB_Sim "NTN_IAB_Sim Official Repository"

---
© 2024 - TELEMATICS LAB - Politecnico di Bari
