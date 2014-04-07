Overview
========

rocon_app_utilities is a python-module to manipulate rapps. It provides a simple command-line tool as well as importable python module. 
The main function is to gather rapps in ROS_PACKAGE_PATH, read and parse the .rapp, and recomposite with parent specification.

Commands
--------

* rocon_app list - display a full list of available rapps
* rocon_app compat <rocon_uri> - displays a list of rapps that are compatible with given rocon uri
* rocon_app info - display a fully resolved rapp specification
* rocon_app rawinfo - display a raw spec of rapp

.. * rapp list - return a list of available apps in ROS_PACKAGE_PATH
   * rapp info <package_name>/<rapp> - return a full specification of rapp. 
   * rapp depends <package_name>/<rapp> - return a list of all of rapp's dependencies
   * rapp depends-on <package_name>/<rapp> - return a list of rapps that depend on the given package

