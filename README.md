# arcane-daffodil
This repo contains a take-home challenge from Analog Devices.

## Problem Statement
Design and implement a modular control system that adjusts the robot's speed based on proximity input and responds
immediately to an emergency stop signal.

- Speed Control Logic:
    - FULL_SPEED : No object within 800 mm
    - SLOW : Object within 800–400 mm
    - STOP : Object within 400 mm
- Emergency Stop:
    - When the emergency stop is triggered, the cobot must immediately stop, regardless of proximity input.
    - Once cleared, the system should resume normal operation.

 ## How to Run

 ### Prerequisites
 1. Install ROS2
 2. Create a ros2 workspace
 3. Clone this repo into the `src` directory of your ros2 workspace
 4. Install GTest

### Running the Code
1. Open a new terminal and source your ROS2 installation so that `ros2` commands will work
2. Navigate to your ros workspace directory
3. Build with colcon
   ```
   $ colcon build --packages-select arcane-daffodil
   ```
   This will create four different executables:
    - `run_speed_controller` - determines and logs Speed State and E-Stop State
    - `run_estop` - user indicates the e-stop state with keyboard input
    - `run_proximity_sensor` - user indicates proximity sensor reading with keyboard input
    - `keyboard_demo` - for manual testing of `KeyboardInputProcessor` class (unrelated to main program)
    
5. Run the `run_speed_controller` executable in your current terminal
   ```
   $ ros2 run arcane-daffodil run_speed_controller
   ```
   This starts up the `speed_controller` node.  You should see the E-Stop State and Speed State being logged to the terminal upon startup, which should look something like this:
   ```
   [INFO] [1754945392.841241308] [speed_controller]: E-Stop: OFF - Speed State: FULL_SPEED
   [INFO] [1754945393.341237205] [speed_controller]: E-Stop: OFF - Speed State: FULL_SPEED
   [INFO] [1754945393.841323111] [speed_controller]: E-Stop: OFF - Speed State: FULL_SPEED
   ...
   ```
   Right now, there is no estop or proximity sensor up and running, so we are seeing the default state of the `speed_controller`, with the estop off and the proximity sensor reading at it's max input.  You'll also notice that this node is publishing to the `/robot/cmd_vel` topic. Once we get the `estop` and `proximity_sensor` nodes running, we'll be able to observe changes here accordingly. 
   
7. Open a new terminal, source uour ROS2 installation, and run the `run_estop` executable
   ```
   $ ros2 run arcane-daffodil run_estop
   ```
   This starts up the `estop` node, which has a default state of OFF.  You should see a message explaining the E-Stop controls pop up, like this:
   ```
   ////////////////////////////////////////////////////////////////////////
   //                                                                    //
   // E-STOP Controls:                                                   //
   //                                                                    //
   // >> Press 'E' to turn the E-STOP on                                 //
   // >> Press 'R' to reset and shut the E-STOP off                      //
   //                                                                    //
   // >> Press 'Q' to quit the program                                   //
   //                                                                    //
   // Warning: if you do not follow instructions and exit the program    //
   // with Ctrl-C instead of Q, the terminal will not be reset properly. //
   // If this happens, you can fix it with the following command:        //
   //                                                                    //
   // $ reset                                                            //
   //                                                                    //
   ////////////////////////////////////////////////////////////////////////
   ```
   Simply follow the stated instructions to control the state of the estop.  When you activate and deactivate the estop, you'll notice that the E-Stop State and Speed State both update accordingly in the terminal with the `speed_controller`.
   
9. Open a new terminal, source uour ROS2 installation, and run the `run_proximity_sensor` executable
   ```
   $ ros2 run arcane-daffodil run_proximity_sensor
   ```
   This starts up the `proximity_sensor` node, which has a default state of a 2.0 meter sensor reading (the max reading of the sensor).  You should see a message explaining the Laser Controls pop up, like this:
   ```
   ////////////////////////////////////////////////////////////////////////
   //                                                                    //
   // Laser Controls:                                                    //
   //                                                                    //
   // >> Press '<' to decrease the values on the laser ranges            //
   // >> Press '>' to increase the values on the laser ranges            //
   //                                                                    //
   // >> Press 'Q' to quit the program                                   //
   //                                                                    //
   // Warning: if you do not follow instructions and exit the program    //
   // with Ctrl-C instead of Q, the terminal will not be reset properly. //
   // If this happens, you can fix it with the following command:        //
   //                                                                    //
   // $ reset                                                            //
   //                                                                    //
   ////////////////////////////////////////////////////////////////////////
   //                                                                    //
   // +-------+                                                          //
   // | robot =<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<|                  //
   // +-------+                                                          //
   //         |       |       |       |       |       |                  //
   //        0.0 m   0.4 m   0.8 m   1.2 m   1.6 m   2.0 m               //
   //                                                                    //
   ////////////////////////////////////////////////////////////////////////

   ```
   Simply follow the stated instructions to control the sensor readings of the proximity_sensor.  When you increment or decrement the values of the sensor, you'll notice that the little ascii visualization of the laser beam and the Speed State in the terminal with the `speed_controller` updates accordingly.

### Running the Tests
1. Source uour ROS2 installation so that `ros2` commands will work
2. Navigate to your ros workspace directory
2. Build the tests with colcon
   ```
   $ colcon build --cmake-args -DBUILD_TESTING=ON
   ```
3. Run the testing executable
   ```
   $ ./build/arcane-daffodil/tests
   ```

   
## Design Notes
The best way to visualize the architecture of the program is to look at the graph created by `rqt`:
<img width="834" height="300" alt="image" src="https://github.com/user-attachments/assets/d1a60366-d617-4d9f-b3e5-a8d2833263e6" />

The `e_stop` node is publishing to the `/estop` topic.  The `proximity_sensor` node is publishing to the `/robot/base_scan` topic.  The `speed_controller` node subscribes to both the `/estop` and `/robot/base_scan` topics, and based on those inputs, determines and logs the E-Stop State and SpeedState, and then publishes the resulting velocity to the `/robot/cmd_vel` topic (not shown here).

The source files themselves are well documented, but I'll take a second to talk about each node.

#### e_stop -----------------------------------------------------------------------
The estop is a simple node which awaits keyboard input and publishes a `std_msgs::msg::Bool` to the `/estop` topic whenever a key is read from the keyboard.  If an `E` is pressed, `true` is published.  If an `R` is pressed, `false` is published.  If any other key besides `Q` (to quit the program) is pressed, the last published bool is republished because state change was not indicated by the user action.  The publish rate is not consistent for the estop because we are blocked on keyboard input for the user, but since this is an estop, I think that is fine.

#### proximity_sensor -------------------------------------------------------------
I actually made two proximity_sensors over the course of this project:
- `DummyProximitySensor`
- `KeyboardProximitySensor`

which I then tied together with a base class, `ProximitySensor`.

Both of these sensors share the same set of parameters:
```
 const int NUM_BEAMS = 5;
 const float ANGLE_MIN_RAD = -M_PI / 2.0;
 const float ANGLE_MAX_RAD = M_PI / 2.0;
 const float ANGLE_INCREMENT_RAD = M_PI / this->NUM_BEAMS;
 const float TIME_INCREMENT_SEC = 0.0001;
 const float SCAN_TIME_SEC = 0.5;
 const float RANGE_MIN_METERS = 0.1;
 const float RANGE_MAX_METERS = 2.0;
```
So, the sensor has an 180* field of view, with 5 evenly spaced beams.  The sensor has a range between 0.1 meters and 2.0 meters.  And the sensor publishes a scan every half-second, with a very small time delta between the first and last beam that is read.

I want to talk for a minute about why some of these parameters are unrealistic for a real sensor.  First, having just 5 beams (spaced 45* apart) instead of 180 beams (spaced 1* apart) is not great quality, since you could have obstacles slip between the beams of the sensor undetected.  Additionally, having such a slow scan time is not great quality, since a very fast obstacle could slip in between sensor readings undetected.  However, the reason why I made these choices is because having a small list of sensor readings and a slow update time is great for having an easily readable topic, and that was important to me.  It is trivially easy to update these two fields to have a sensor with a higher frequency and greater density of beams, so I would rather things be readable for the sake of a toy problem like this. 

The `DummyProximitySensor` was the first thing I wrote for this project, and I decided to have it just spit out canned values, oscillating between the min sensor reading and the max sensor reading.  It publishes these values at a constant rate.  This is fine, but it was annoying to have to visibly check the topic against the output of the `speed_controller`.

Later, after I had created the estop functionality, and hence the `KeyboardInputProcessor`, it occurred to me that it would be nice to be able to have the user control the proximity sensor input as well, and since I had made the `KeyboardInputProcessor` it's own class, it was very simple to make a new proximity sensor.  The main difference between the `DummyProximitySensor` and the `KeyboardProximity` sensor is that, because the `KeyboardProximitySensor` is dependant on user input, the publisher is blocked on keyboard input, so it isn't publishing constantly.  This is very unrealistic, but makes for a nice crisp demo.

Once I had made a second proximity sensor, it felt morally right to make a base class rather than just delete my least favorite sensor.  I wanted to show off my object oriented design principles.  If you want to try using the `DummyProximitySensor`, just open up `src/run_proximity_sensor.cpp`, uncomment the line which defines the `DummyProximitySensor`, comment out the line which defines the `KeyboardProximitySensor`, rebuild, and rerun. 

If there was a simulated or real sensor which publishes a `sensor_msgs::msg::LaserScan` message, you could easily plug that into the code as well.  You would just need to update the `SpeedController::laser_subscription` to point to the appropriate topic. 

#### speed_controller -------------------------------------------------------------
This is where the magic happens! The `speed_controller` subscribes to both `/estop` and `/robot/base_scan`.  The former has a callback function which updates an internal class member which keeps track of the E-Stop state.  The latter has a callback function which updates an internal class member which keeps track of the min sensor reading in the most recent sensor message. Then, the publisher is called.  The publisher calls `SpeedController::determine_speed_state` which is where the core logic is.  If the E-Stop state is ON, it sets the `SpeedState` to `STOP` no matter whet the min sensor reading is.  Otherwise, it sets the `SpeedState` to whichever state is appropriate given the sensor reading.  Then, the publisher publishes the appropriate velocity as a `geometry_msgs::msg::Twist` to the `/robot/cmd_vel` topic, which could be used to control a simple motor on a mobile robot.  I chose to use 5mps for the fast speed and 2mps for the slow speed, but these are kind of random values; you should tune these parameters to whatever the situation you're dealing with if you were to use a real robot.

## Frequently Asked Questions

#### Did you use any code generator tools?
No.  I wrote everything in vim, and while I used google, etc the way that all developers do in the course of their jobs, I dodn't use any code generator tools.

#### Why didn't you use the ros2_ur5_interface for simulation?
I sank some time initially into this simulator.  I got it up and working locally, but the only sensor in there was a camera.  I didn't know how to add a new sensor to the simulator (I have never had to do so) and so I timeboxed that investigation and ended up deciding that it would be more worthwhile to just focus on writing the code than getting that specific simulator to work.  After all, the requirements does say you can use scripted/keyboard inputs, and I did add a cute ascii visualization (bonus point).

#### Why are there so few tests?
I ended up running out of time to do as much testing as I would really like.  I would say that the absolute core of the program is covered by the tests (the important logic in the `speed_controller`).  With more time, I would have liked to test that all the publishers and subscribers and other auxiliary functions are properly covered by unit testing as well.

#### Why is the repo name so random?
I have never worked on a repo whose name made sense!  I think that in general, naming is very important, but when starting a project, I don't like to let the analysis-paralysis of naming impact my ability to start, so I like choosing something random.  

But the a___-d___ pattern was intentional.
