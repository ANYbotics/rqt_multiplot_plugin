# Rqt Multiplot Plugin

## Overview

**Author(s):** Ralf Kaestner

**Maintainer:** Ralf Kaestner <ralf.kaestner@gmail.com>

**Licsense:** GNU Lesser General Public License (LGPL)

**Operating system(s):** Debian-based Linux, Mac OS X

**Package PPA:** Not available

## Content

This project provides a GUI plugin for visualizing numeric values in multiple
2D plots using the [Qwt](http://qwt.sourceforge.net) plotting backend.

## Installation

### Dependencies

- [rqt](http://wiki.ros.org/rqt)

  ```shell
  sudo apt-get install ros-indigo-rqt
  ```

- [variant_topic_tools](https://github.com/ethz-asl/ros-variant)

  Consult the [installation instructions]
  (https://github.com/ethz-asl/ros-variant/blob/master/README.md#installation)
  provided by this project.

### Building

Create a symlink in your catkin source folder, e.g.:

  ```shell
  ln -s ~/git/ros-rqt-multiplot-plugin ~/catkin_ws/src
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

## Bugs & Feature Requests

Please report bugs and feature requests on the
[Issue Tracker](https://github.com/ethz-asl/ros-rqt-multiplot-plugin).
