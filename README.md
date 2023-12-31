# MyNMPC
## Nonlinear Model Predictive Control (NMPC) for Self-Driving Vehicles
### European Master on Advanced RObotics (EMARO+) Thesis
#### Author: Simone Contorno

<br>

### Introduction
An introduction to this repository.<br>
[Go to Introduction](#intro)

### How it works
A rapid description of how the program works (pseudo-code).<br>
[Go to How it works](#how)

### Prerequisites
Needed prerequisites to correctly install.<br>
[Go to Prerequisites](#pre)

### Installation
How to install.<br>
[Go to Installation](#install)

### Execution
How to execute the examples.<br>
[Go to Execution](#exec)

### Improvements
Possible improvements.<br>
[Go to Improvements](#improve)

### Conclusion
Brief conclusion and BibTeX for citing.<br>
[Go to Conclusion](#con)

<a name="intro"></a>
# Introduction

This repository provides a Nonlinear Model Predictive Control (NMPC) program written in C++. <br>
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

## Pseudo-code

### ProxQP
<pre><code>FUNCTION init
    SET model info
    SET equality constraints
    SET inequality constraints

    INITIALIZE QP problem
    
    SET Hessian matrix
    
    DEFINE sparse problem
    DEFINE dense problem
ENDFUNCTION

FUNCTION solve WITH horizon states,
                    horizon controls,
                    last control,
                    slack variable,
                    horizon state goals,
                    horizon control goals

    SET linear coefficients vector
    SET equality constraints matrix
    SET equality constraints bounds vector
    SET inequality constraints matrix
    SET inequality constraints bounds vector

    SOLVE sparse OR dense problem

    UPDATE decision variables
    
    RETURN optimal decision variables and QP info
ENDFUNCTION
</code></pre>

### MPC
<pre><code>FUNCTION solve
    UPDATE state and control 
    DO SQP
        SOLVE ProxQP sub-problem
        UPDATE state, control, and slack variable 
    UNTIL problem is solved || max. SQP iterations reached

    UPDATE first control input 

    RETURN optimal states and controls
ENDFUNCTION
</code></pre>

Other information:
<ul>
    <li><a href="https://github.com/simone-contorno/mynmpc/blob/main/CONTORNO_Simone_Thesis.pdf">Mathematical (and other) details</a>.</li>
    <li><a href="https://simone-contorno.github.io/mynmpc/">Doxygen documentation</a>.</li>
</ul>

<a name="pre"></a>
# Prerequisites

<ol>
    <li>Install the Eigen3 library:
    <pre><code>sudo apt install libeigen3-dev</code></pre>
    </li>
    <li><a href="https://github.com/Simple-Robotics/proxsuite">Install</a> the ProxQP solver.<br></li>
</ol>

<a name="install"></a>
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

<a name="exec"></a>
# Execution

To try the program you first need to install a visualizer provided by the <a href="https://github.com/simone-contorno/mobro_sim">mobro_sim</a> package based on the <a href="https://github.com/oKermorgant/simple_launch">simple_launch</a> and <a href="https://github.com/oKermorgant/map_simulator">map_simulator</a> ones:<br>

<ol>
    <li>Go into the src folder of your ROS 2 workspace.</li> 
    <li>Download these repositories:
    <pre><code>git clone https://github.com/oKermorgant/simple_launch</code></pre>
    <pre><code>git clone https://github.com/oKermorgant/map_simulator</code></pre>
    <pre><code>git clone https://github.com/simone-contorno/mobro_sim</code></pre>
    </li>
    <li>Go into the root folder of your ROS 2 workspace and build them: 
    <pre><code>colcon build --packages-select simple_launch map_simulator mobro_sim</code></pre>
    </li>
</ol>

Then, you need to switch to the <a href="https://github.com/simone-contorno/mynmpc/tree/simulation">simulation</a> branch and follow the instructions to run the controller example. <br>
Switching to the <a href="https://github.com/simone-contorno/mynmpc/tree/rosbot">rosbot</a> branch you can also have a test for the <a href="https://husarion.com/manuals/rosbot/">ROSbot 2R</a> by Husarion.

<a name="improve"></a>
# Improvements

A discussion about the possible improvements can be read <a href="https://github.com/simone-contorno/mynmpc/blob/main/CONTORNO_Simone_Thesis.pdf">here</a> in Section "Possible future works".

<a name="con"></a>
# Conclusion
I hope that this work can be useful to your purposes, if you have any questions feel free to ask. <br>
Videos of the given examples can be found on <a href="https://www.youtube.com/playlist?list=PLXuiUDYHp72dNS846t7llz_p0f-Nf_IVd">my YouTube channel</a>. <br>
To cite this work please use the following BibTeX:

<pre><code>@misc{MyNMPC,
    TITLE       = {MyNMPC: A Nonlinear Model Predictive Control for Self-Driving Vehicles},
    AUTHOR      = {Simone Contorno},
    YEAR        = {2023},
    URL         = {https://github.com/simone-contorno/mynmpc}
}
</code></pre>
