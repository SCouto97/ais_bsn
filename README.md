# Introduction

In this page, we provide information on how to install the Body Sensor Network (BSN) implementation for the [IEEE Computer Special Issue on Formal Methods Applied to Cyber-Physical Systems](https://www.computer.org/digital-library/magazines/co/call-for-papers-special-issue-on-formal-methods-applied-to-cyber-physical-systems/). In this extension of the BSN, we apply the principles of Artificial Immune Systems to safely and systematically devise the observers automata at design time, which are further implemented as runtime monitors. An explanation of the UPPAAL models including the observers can be found [here](https://github.com/lesunb/ais_bsn/wiki/UPPAAL). The performance of our experiments themselves are detailed [here](https://github.com/lesunb/ais_bsn/wiki/Experiments).

![Goalmodel for the BSN](img/bsn_goalmodel.png)

The image above represents the Goal Model of the BSN. The contexts considered in this experiment are:

- C1) SPO2 is available
- C2) SPO2 valid data range
- C3) ECG is available
- C4) ECG valid data range
- C5) TEMP is available
- C6) TEMP in valid data range
- C7) Bodyhub battery level

# BSN implementation on ROS

This is a Body Sensor Network implementation on ROS. So far, the BSN was used for experimentation on solutions for adaptation on the Self-Adaptive Software Systems domain, refer to https://doi.org/10.1145/3194133.3194147 and https://doi.org/10.1109/SEAMS.2019.00020 for more information. The following instructions will guide you to to compile, deploy and run the BSN on Linux Ubuntu 18.04 with ROS Melodic distributions. We have not yet tested on other distributions.

## Dependencies:
* [Ros Melodic](http://wiki.ros.org/melodic) which provides software libraries for BSN engines.
* [Lepton](https://github.com/rdinizcal/lepton) ("lightweight expression parser") is a small C++ library for parsing, evaluating, differentiating, and analyzing mathematical expressions.
* [Bsn Library](https://github.com/rdinizcal/libbsn)  provides the implementation of sensors, data fusers and emergency detection
* [Bsn arch](https://github.com/rdinizcal/arch)

## ROS:
First it is required to install ROS Melodic. Our development team is strictly using Ubuntu 18.04 (Bionic). To install it please follow this [link](http://wiki.ros.org/melodic/Installation/Ubuntu).  
Also, it is strongly advised to use catkin for managing the ROS packages, refer to this [link](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) after installing ROS Melodic. As such you will need to create a catkin workspace. You can do so by following the steps:

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

## Lepton, Libbsn and Archlib Build
For these 3 dependencies, one must first clone this repository into the 'src' folder inside the catkin workspace previously created:

```
cd ~/catkin_ws/src
git clone https://github.com/rdinizcal/bsn_ros.git
```

When cloned, go to the 'bsn_ros' folder and then proceed to install the dependencies

```
cd bsn_ros
```

Inside the 'bsn_ros' folder, simply run the following command to effectively install and build the 3 dependencies:

```
bash install.sh
```

## Configuration and Execution

3. Configure roslaunch files for personalized execution under '/catkin_ws/src/bsn/configurations';

4. Execute the BSN either by executing the pre-set run.sh file, that executes all nodes, 
or use roslaunch x.launch to execute a single node:
```
cd ~/catkin_ws/src/bsn/ && 
bash run.sh 600
``` 

For this example we chose to use the run.sh script with 600 seconds of execution time, but it can be any user-defined value. If no value is provided , the default value used is 300 seconds (5 minutes).

### Analysis of Logs
After running the system, four log files of the execution will appear inside the 'bsn_ros/src/knowledge_repository/resource/logs' folder. Remember that each execution generates different log files in the form logName_logID, where the logID is the timestamp of the beginning of the execution.

In order to analyze these log files, one can run the Analyzer python code inside the 'bsn_ros/src/simulation/analyzer' folder using the following command

```
cd ~/catkin_ws/src/bsn_ros/src/simulation/analyzer/
python3 analyzer.py [logID] [metric] [plot_component_metrics] [setpoint]
```

where:

* [logID] is the ID for the execution log files previously mentioned
* [metric] is the metric to be analyzed. This can be reliability or cost depending on what was the metric used for the execution
* [plot_component_metrics] is a parameter that defines if the graphic which will be shown at the end of analysis will contain components local reliabilities or not. The components local reliabilities will be present if this is equal to True or true and only the system's global reliability will be present otherwise
* [setpoint] will be the value of the setpoint used in the execution. This needs to be the same as the one defined in the System Manager launch file

One example of the command usage would be:

```
python3 analyzer.py 1610549979516318295 reliability False 0.9
```

### In case or error due to the ROS path

You might want to source the setup.bash inside the catkin workspace:
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### In case or error due to cmake not finding libcrypto

If you see the following error
```
/usr/bin/ld: CMakeFiles/main.dir/main.cpp.o: undefined reference to symbol 'CONF_modules_unload@@OPENSSL_1_1_0'
//usr/lib/x86_64-linux-gnu/libcrypto.so.1.1: error adding symbols: DSO missing from command line
collect2: error: ld returned 1 exit status
ninja: build stopped: cannot make progress due to previous errors.
```

Follow the instructions in [issue#1388@cpprest](https://github.com/microsoft/cpprestsdk/issues/1388#issuecomment-619570350).
Where you must add `INTERFACE_LINK_LIBRARIES "OpenSSL::SSL"` in:
```
set_target_properties(cpprestsdk::cpprestsdk_openssl_internal PROPERTIES
  INTERFACE_COMPILE_DEFINITIONS "CPPREST_NO_SSL_LEAK_SUPPRESS"
  INTERFACE_LINK_LIBRARIES "OpenSSL::SSL"
)
```


## Main Authors

* **Ricardo D. Caldas** - https://github.com/rdinizcal
* **Samuel Couto** - https://github.com/SCouto97
* **Gabriel Araújo (Moisés)** - https://github.com/Gastd
* **Gabriel Levi** - https://github.com/gabrielevi10
* **Léo Moraes** - https://github.com/leooleo  
* **Eric B. Gil** - https://github.com/ericbg27/

Adviser: **Genaína N. Rodrigues** - https://cic.unb.br/~genaina/
