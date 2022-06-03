# simple_localmap_creator

A very simple ROS node with LaserScan input and OccupancyGrid output.  
There are some advanced ROS nodes, but this simple node was unexpectedly missing.

<img src="https://user-images.githubusercontent.com/60866331/171795354-f01d3734-d052-4aa6-9498-3eae7cef0239.png" width="300">

## Environment

ROS Noetic

## Setup

Clone this repository to your catkin workspace and build.  
Sample of setup flow follows.

```sh
cd ~/catkin_ws/src
git clone git@github.com:YasunoriHirakawa/simple_localmap_creator.git
cd simple_localmap_creator
catkin build --this
```

## I/O

### Inputs
`sensor_msgs/LaserScan`
- default topic name: `/scan`

### Outputs
`nav_msgs/OccupancyGrid`
- default topic name: `/localmap`
- default frame id: `base_link`

## Params
| name | type | about |
| - | - | - |
| hz | int | Roop rate of this node |
| map_resolution | double | Meters per grid of the occupancy grid |
| map_width | double | Map y-axis length (meters) |
| map_height | double | Map x-axis length (meters) |

## Usage

Sample launch file can be used as below.

```sh
roslaunch simple_localmap_creator localmap_creator.launch
```
