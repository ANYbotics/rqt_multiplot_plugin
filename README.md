# Rqt Multiplot Plugin

## Overview

**Author(s):** Ralf Kaestner

**Maintainer:** Ralf Kaestner <ralf.kaestner@gmail.com>

**License:** GNU Lesser General Public License (LGPL)

**Operating system(s):** Debian-based Linux, Mac OS X

**Package PPA:** ppa:anybotics/ros

## Content

This project provides a GUI plugin for visualizing numeric values in multiple 2D plots using the [Qwt](http://qwt.sourceforge.net) plotting backend.

## Installation
### Dependencies

- [rqt](http://wiki.ros.org/rqt)

    ```shell
    sudo apt-get install ros-indigo-rqt
    ```

- [variant_topic_tools](https://github.com/anybotics/variant)

  Consult the [installation instructions](https://github.com/anybotics/variant/blob/master/README.md#installation) provided by this project.

- [qwt](http://qwt.sourceforge.net/)

    ```shell
    sudo apt-get install libqwt-dev
    ```

### ROS Distribution

The package is in the ROS (melodic, noetic) distribution.

```shell
sudo apt-get update
sudo apt-get install ros-melodic-rqt-multiplot
sudo apt-get install ros-noetic-rqt-multiplot

```

### Building from Source

Create a symlink in your catkin source folder, e.g.:

```shell
ln -s ~/git/rqt_multiplot_plugin ~/catkin_ws/src
cd ~/catkin_ws
catkin build rqt_multiplot
```

### Debian Package

```shell
sudo add-apt-repository ppa:anybotics/ros
sudo apt-get update
sudo apt-get install ros-indigo-rqt-multiplot
```

## Usage

To launch the standalone rqt plugin, run

```shell
rosrun rqt_multiplot rqt_multiplot
```

To launch the rqt GUI without a perspective, run

```shell
rqt --force-discover
```

This will discover all plugins, which can then be loaded manually.

To delete the default configuration files (in case of problems):

```shell
rqt --clear-config
```

### Import ROS Bag

To plot an imported ROS bag, the desired curves have to be configured before
importing the bag.

### Example Views

![Example views](http://wiki.ros.org/rqt_multiplot?action=AttachFile&do=get&target=multiplot_1_legend.png "Overview")

#### Configure Plot

![Configure plot](http://wiki.ros.org/rqt_multiplot?action=AttachFile&do=get&target=multiplot_configure_plot.png "Configure plot")

#### Edit Curve

![Edit curve](http://wiki.ros.org/rqt_multiplot?action=AttachFile&do=get&target=multiplot_edit_curve.png "Edit curve")

## Bugs & Feature Requests

Please report bugs and feature requests on the [Issue Tracker](https://github.com/anybotics/rqt_multiplot_plugin).

## Build Status

### Devel Job Status

| | Melodic  | Noetic |
| --- | --- | --- |
| rqt_multiplot_plugin | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mdev__rqt_multiplot_plugin__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__rqt_multiplot_plugin__ubuntu_bionic_amd64/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ndev__rqt_multiplot_plugin__ubuntu_focal_amd64)](http://build.ros.org/job/Ndev__rqt_multiplot_plugin__ubuntu_focal_amd64/) |

### Release Job Status

| | Melodic | Noetic |
| --- | --- | --- |
| rqt_multiplot | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__rqt_multiplot__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__rqt_multiplot__ubuntu_bionic_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__rqt_multiplot__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__rqt_multiplot__ubuntu_focal_amd64__binary/) |
