# MyNMPC
## European Master on Advanced RObotics (EMARO+)
### Nonlinear Model Predictive Control (NMPC) for Self-Driving Vehicles

<br>

# Example Execution
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
