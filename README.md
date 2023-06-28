# MyNMPC
## European Master on Advanced RObotics (EMARO+)
### Nonlinear Model Predictive Control (NMPC) for Self-Driving Vehicles

<br>

### Introduction
An introduction to this repository.<br>
[Go to Introduction](#intro)

### How it works
A rapid description of how the program works (pseudo-code).<br>
[Go to How it works](#how)

### Prerequisites
Needed prerequisites to correctly install and execute this program.<br>
[Go to Prerequisites](#pre)

### Installation
How to install and run this program in Linux.<br>
[Go to Installation](#installation)

### Execution
How to execute examples.<br>
[Go to Execution](#execution)

<a name="intro"></a>
# Introduction

This repository provides a Nonlinear Model Predictive Control (NMPC) program wrote in C++. <br>
The implemented NMPC uses the Sequential Quadratic Programming (SQP) approach by recursively calling the <a href="https://hal.inria.fr/hal-03683733/file/Yet_another_QP_solver_for_robotics_and_beyond.pdf">ProxQP</a> solver. <br>
I developed this program for my Master Thesis in Robotics Engineering (University of Genoa) at the LS2N (Laboratoire des sciences du numérique de Nantes) during my 1-year period at the École Centrale de Nantes with the EMARO+ double degree program. 

<a name="how"></a>
# How it works

The program runs in a ROS 2 (Robot Operating System) environment. In particular, it has been developed using the Foxy version. <br>
It is structured over 3 levels:
<ol>
    <li>Model: a model file must be defined to describe the vehicle (e.g. unicycle or bicycle).</li>
    <li>MPC: a MPC file must be defined to describe the tuning of the Model Predictive Control (e.g. weight matrices).</li>
    <li>Controller: a control file must be written in order to opportunely use the model and MPC files to define the problem, and then managing the related data to recursively solve it and gives the optimal control input to the vehicle.</li>
</ol>

Other information:
<ul>
    <li><a href="">Mathematical (and other) details</a>.</li>
    <li><a href="">Doxygen documentation</a>.</li>
</ul>

<a name="pre"></a>
# Prerequisites

<ol>
    <li>Install the Eigen3 library:
    <pre><code>sudo apt install libeigen3-dev</code></pre>
    </li>
    <li><a href="https://github.com/Simple-Robotics/proxsuite">Install</a> the ProxQP solver.<br></li>
</ol>

<a name="installation"></a>
# Installation 

<ol>
    <li>Go into the src folder of your ROS 2 workspace.<br></li> 
    <li>Download this repository:
    <pre><code>git clone https://github.com/simone-contorno/mynmpc</code></pre>
    </li>
    <li>Go into the root folder of your ROS 2 workspace and build it: 
    <pre><code>colcon build --packages-select mynmpc</code></pre>
    </li>
</ol>

<a name="execution"></a>
# Execution

For trying the program you need to run one already implemented controller which use a visualizer provided by the <a href="">mobro_sim</a> package based on the <a href="https://github.com/oKermorgant/simple_launch">simple_launch</a> and <a href="https://github.com/oKermorgant/map_simulator">map_simulator</a> ones.<br>

<ol>
    <li>Go into the src folder of your ROS 2 workspace.</li> 
    <li>Download these repositories:
    <pre><code>git clone https://github.com/oKermorgant/simple_launch</code></pre>
    <pre><code>git clone https://github.com/oKermorgant/map_simulator</code></pre>
    <pre><code>git clone https://github.com/simone-contorno/mobile_robot_simulator</code></pre>
    </li>
    <li>Go into the root folder of your ROS 2 workspace and build them: 
    <pre><code>colcon build --packages-select simple_launch map_simulator mobro_sim</code></pre>
    </li>
</ol>

## Example
<ol>
    <li>Go into the mynmpc folder and change branch:
    <pre><code>git checkout simulation</code></pre>
    </li>
    <li>Go into the root folder of your ROS 2 workspace and re-build it:
    <pre><code>colcon build --packages-select mynmpc</code></pre>
    </li>
    <li>Launch the simulation:
    <pre><code>ros2 launch mynmpc simulation.launch.py</code></pre>
    </li>
    <li>Run the controller: <pre><code>ros2 run mynmpc controller</code></pre>
    </li>
</ol>
