^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_multiplot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.5 (2016-08-05)
------------------
* fixes `#2 <https://github.com/ethz-asl/rqt_multiplot_plugin/issues/2>`_ a QT API change
* rename two tooltips (copy and past curve) in PlotConfigWidget
* qt5 ready
* Contributors: Samuel Bachmann

0.0.4 (2016-05-23)
------------------
* add install command for resource icons
* Contributors: Samuel Bachmann

0.0.3 (2016-05-22)
------------------
* fixes for release
* Contributors: Samuel Bachmann

0.0.2 (2016-05-20)
------------------
* Fixes related to OS X clang compatibility and added support for Qwt >= 6.1
* Added copy and paste for curve configurations
* Fixed segmentation fault when removing curves
* * Refactored configuration classes
  * Added drag and drop feature for copying and moving curves from the
  legend of one plot to another plot
  * Added possibility to double-click legend items for editing curve
  properties
* * Implemented match filter combo box as class
  * Added match filter completer to type combo box
* Added match filter completer to topic combo box
* * Curve axis topic and type are now automatically filled if initially empty
  * Added feature to copy between axes configurations
* Bag reader is now using topic queries to reduce the number of processed messages
* Added adaptive-precision output format to tracker coordinate labels
* Fixed bag reader to not block event processing
* Implemented bag import feature
* * Curve axis config now starts with no type selected
  * Fixed validations in curve axis config widget
* Fixed message field completer to correctly handle dynamic arrays
* Bug fixing and stability improvements on the configuration and subscriber backend
* * Added feature for cleaning the configuration file history
  * Fixed threads to terminate on destruction
  * Added package and message type registry updates to main widget
  constructor
  * Fixed curve config widget components to initialize correctly
* Added feature to specify custom axis titles
* Fixed configuration file loading/saving bug, now producing ROS console output
* Plot widgets cannot change state anymore if only one plot is available in the plot table
* Fixed a graphical bug which was caused by bad masks for the plot cursor tracked point labels
* Implemented topic interpolation feature
* About to get topic interpolation right
* Added feature to export plots
* Added feature to maximize/restore a plot
* Added axis titles and legend
* * Added curve styles
  * Revised plot controls
  * Efficient curve data storage
* Plot scaling works
* Enhanced plot table configuration and widget
* Added configuration file handling
* Contributors: Ralf Kaestner
