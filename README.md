# omni_turtlesim

| dashing-devel |
| ---- |
| [![industrial_ci (dashing)](https://github.com/Tiryoh/omni_turtlesim_ros2/workflows/industrial_ci%20(dashing)/badge.svg?branch=dashing-devel)](https://github.com/Tiryoh/omni_turtlesim_ros2/actions?query=workflow%3A%22industrial_ci+%28dashing%29%22+branch%3Adashing-devel) |

ROS 2 package of omni_turtlesim

omni-directional version of [turtlesim](http://wiki.ros.org/turtlesim)

![Image from Gyazo](https://i.gyazo.com/974e67e38431b10c9985c9b033eed577.gif)

## Usage

```bash
# On Terminal 1
$ ros2 run omni_turtlesim turtlesim_node
# On Terminal 2
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/turtle1/cmd_vel
```

## ROS Nodes

### turtlesim_node

turtlesim_node provides a simple simulator.

#### Subscribed Topics

* __turtleX/cmd_vel__ (geometry_msgs/Twist)
    * The linear and angular command velocity for turtleX. The turtle will execute a velocity command for 1 second then time out. Twist.linear.x is the forward velocity and Twist.angular.z is the angular velocity.

#### Published Topics

* __turtleX/pose__ (omni_turtlesim/Pose)
    * The x, y, theta, linear velocity, and angular velocity of turtleX.
* __turtleX/color_sensor__ (omni_turtlesim/Color)
    * The RGB color on the color sensor on turtleX.

#### Services

* __clear__ (std_srvs/Empty)
    * Clears the turtlesim background and sets the color to the value of the background parameters.
* __reset__ (std_srvs/Empty)
    * Resets the turtlesim to the start configuration and sets the background color to the value of the background.
* __kill__ (omni_turtlesim/Kill)
    * Kills a turtle by name.
* __spawn__ (omni_turtlesim/Spawn)
    * Spawns a turtle at (x, y, theta) and returns the name of the turtle. Also will take name for argument but will fail if a duplicate name.
* __turtleX/set_pen__ (omni_turtlesim/SetPen)
    * Sets the pen's color (r g b), width (width), and turns the pen on and off (off).
* __turtleX/teleport_absolute__ (omni_turtlesim/TeleportAbsolute)
    * Teleports the turtleX to (x, y, theta).
* __turtleX/teleport_relative__ (omni_turtlesim/TeleportRelative)
    * Teleports the turtleX a linear and angular distance from the turtles current position.

#### Parameters
* __~background_b__ (int, default: 255)
    * Sets the blue channel of the background color.
* __~background_g__ (int, default: 86)
    * Sets the green channel of the background color.
* __~background_r__ (int, default: 69)
    * Sets the red channel of the background color.

### mimic

mimic provides a simple interface for making one turtlesim mimic another.

#### Subscribed Topics
* __input/pose__ (omni_turtlesim/Pose)
    * The input topic for the mimic node. The topic must be remapped to the pose topic of the desired turtle to mimic.

#### Published Topics
* __output/cmd_vel__ (geometry_msgs/Twist)
    * The output topic for the mimic node. The topic must be remapped to the cmd_vel topic of the mimicking turtle.


## License

(C) 2020 Daisuke Sato

This repository is released under the 3-Clause BSD License ([BSD-3-Clause](https://spdx.org/licenses/BSD-3-Clause.html)), see [LICENSE](./LICENSE).

### Acknowledgements

This software is based on [turtlesim](http://wiki.ros.org/turtlesim), released under the 3-Clause BSD license.

```cpp
/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 ```
