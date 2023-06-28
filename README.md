# MyNMPC
## European Master on Advanced RObotics (EMARO+)
### Nonlinear Model Predictive Control (NMPC) for Self-Driving Vehicles

<br>

# ROSbot 2R Example

The following steps are provided for a ROSbot 2R with ROS 1 installed and for which, as consequence, it is necessary the <a href="https://github.com/ros2/ros1_bridge">ROS 1 bridge</a>.

<ol>
    <li><a href="https://github.com/ros2/ros1_bridge#prerequisites">Install</a> the ROS 1 bridge choosing the branch corresponding to your ROS 2 version.</li>
    <li>Go into the mynmpc folder and change branch:
    <pre><code>git checkout rosbot</code></pre>
    </li>
    <li>Go into the root folder of your ROS 2 workspace and re-build it:
    <pre><code>colcon build --packages-select mynmpc</code></pre>
    </li>
</ol>

After having connected to your ROSbot through the same wifi connection, open 4 different shells and respectively follow these instructions:
<ol>
    <li>First shell:
        <ol>
            <li>Source ROS 1 installation:
            <pre><code>source /opt/ros/$ROS_1_DISTRO$/setup.bash</code></pre>
            </li>
            <li>Run the core:
            <pre><code>roscore</code></pre>
            </li>
        </ol>
    </li>
    <li>Start the ROSbot.</li>
    <li>Second shell:
        <ol>
            <li>Source ROS 1 installation:
            <pre><code>source /opt/ros/$ROS_1_DISTRO$/setup.bash</code></pre>
            </li>
            <li>Source ROS 2 workspace:
            <pre><code>source ~/$ROS_2_WS_WITH_BRIDGE$/install/setup.bash</code></pre>
            </li>
            <li>Run the ROS 1 bridge:
            <pre><code>ros2 run ros1_bridge dynamic_bridge</code></pre>
            </li>
        </ol>
    </li>
    <li>Third shell:
        <ol>
            <li>Source ROS 2 installation:
            <pre><code>source /opt/ros/$ROS_2_DISTRO$/setup.bash</code></pre>
            </li>
            <li>Source ROS 2 workspace:
            <pre><code>source ~/$ROS_2_WS$/install/setup.bash</code></pre>
            </li>
            <li>Launch the simulation:
            <pre><code>ros2 launch mynmpc simulation.launch.py</code></pre>
            </li>
        </ol>
    </li>
    <li>Fourth shell:
        <ol>
            <li>Source ROS 2 installation:
            <pre><code>source /opt/ros/$ROS_2_DISTRO$/setup.bash</code></pre>
            </li>
            <li>Source ROS 2 workspace:
            <pre><code>source ~/$ROS_2_WS$/install/setup.bash</code></pre>
            </li>
            <li>Run the controller: <pre><code>ros2 run mynmpc controller</code></pre>
            </li>
        </ol>
    </li>
</ol>

### Hints

<ul>
    <li>Since you will need to source different shells with different ROS distributions, make sure of not having an automatic source command in your .bashrc file, and (personal suggestion) create and alias to run commands faster.
    </li>
    <li>If the dynamic bridge does not correctly map all the needed ROS 1 topics to ROS 2, run it with this additional option:
    <pre><code>ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics</code></pre>
    This will map all the ROS 1 topics to ROS 2.
    </li>
</ul>
If you have doubts or are still encountering problems, try to consult <a href="https://github.com/mmatteo-hub/rosbot_ws">this</a> guide.
