# Rqt Multiplot Plugin

## Overview

**Author(s):** Ralf Kaestner

**Maintainer:** Ralf Kaestner <ralf.kaestner@gmail.com>

**License:** GNU Lesser General Public License (LGPL)

**Operating system(s):** Debian-based Linux, Mac OS X

**Package PPA:** ppa:ethz-asl/ros

## Content

This project provides a GUI plugin for visualizing numeric values in multiple 2D plots using the [Qwt](http://qwt.sourceforge.net) plotting backend.

## Installation
### Dependencies

- [rqt](http://wiki.ros.org/rqt)

    ```shell
    sudo apt-get install ros-indigo-rqt
    ```

- [variant_topic_tools](https://github.com/ethz-asl/variant)

  Consult the [installation instructions](https://github.com/ethz-asl/variant/blob/master/README.md#installation) provided by this project.

- [qwt](http://qwt.sourceforge.net/)

    ```shell
    sudo apt-get install libqwt-dev
    ```

### Building

Create a symlink in your catkin source folder, e.g.:

```shell
ln -s ~/git/rqt_multiplot_plugin ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```

###Debian Package

```shell
sudo add-apt-repository ppa:ethz-asl/ros
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

### Example Views

![enter image description here](https://lh3.googleusercontent.com/-EF4aCvEV3ZU/V0Vku40VueI/AAAAAAAAajg/rdRvc-YWkPw50gPOGbGrtMtzMjgmBANfACLcB/s700/multiplot_1_legend.png "Overview")

####Configure Plot

![enter image description here](https://lh3.googleusercontent.com/-E14yRrgKars/V0VlFJdDX5I/AAAAAAAAajo/2Nfo_ovj5dABrF7OQPExlMJY1gMAKK43QCLcB/s700/multiplot_configure_plot.png "Configure plot")

####Edit Curve

![enter image description here](https://lh3.googleusercontent.com/-Ei_j84gwJ7U/V0VlWrjUumI/AAAAAAAAaj0/dEB0dkE2YJ8rCWpmql6ZW4f6iMlJgxv8ACLcB/s700/multiplot_edit_curve.png "Edit curve")

## Bugs & Feature Requests

Please report bugs and feature requests on the [Issue Tracker](https://github.com/ethz-asl/rqt_multiplot_plugin).

## Build Status

### Devel Job Status

| | Indigo  |
| --- | --- |
| rqt_multiplot_plugin | [![Build Status](http://build.ros.org/buildStatus/icon?job=Idev__rqt_multiplot_plugin__ubuntu_trusty_amd64)](http://build.ros.org/job/Idev__rqt_multiplot_plugin__ubuntu_trusty_amd64/) |

### Release Job Status

| | Indigo |
| --- | --- |
| rqt_multiplot | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__rqt_multiplot__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Ibin_uT64__rqt_multiplot__ubuntu_trusty_amd64__binary/) |
