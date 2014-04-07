Rapp Specification
==================

Rapp infers `rocon_app` or `robot_app` used in `Robotics in Concert`_. 

.. _`Robotics in Concert`: http://www.robotconcert.org

Rapp Types
----------

The following table describes the characteristics of each rapp types and requirements

* R = required
* O = optional
* N = not Allowed
* i = inherited from nearest parent if not present


.. table:: 

  +-----------------------+-------------------------+-------------------------+-------------------------+-----------------------------------+
  | Field                 |  Virtual Rapp           | Implementation Rapp                               |  Type                             | 
  +=======================+=========================+=========================+=========================+===================================+
  |                       | Ancestor                | Ancestor                | Child                   |                                   |
  +-----------------------+-------------------------+-------------------------+-------------------------+-----------------------------------+
  | display               |     R                   | R                       | O :sup:`i`              |   str                             |
  +-----------------------+-------------------------+-------------------------+-------------------------+-----------------------------------+
  | description           |     R                   | R                       | O :sup:`i`              |   str                             |
  +-----------------------+-------------------------+-------------------------+-------------------------+-----------------------------------+
  | icon                  |     O                   | O                       | O :sup:`i`              | <package>/<png, jpg, jpeg>        |
  +-----------------------+-------------------------+-------------------------+-------------------------+-----------------------------------+
  | public_interface      |     O                   | O                       | N :sup:`i`              | <package>/<.interface>            |
  +-----------------------+-------------------------+-------------------------+-------------------------+-----------------------------------+
  | public_parameters     |     O                   | O                       | N :sup:`i`              | <package>/<.parameters>           |
  +-----------------------+-------------------------+-------------------------+-------------------------+-----------------------------------+
  | compatibility         |     N                   | R                       | R                       | `Rocon URI`_                      |
  +-----------------------+-------------------------+-------------------------+-------------------------+-----------------------------------+
  | launch                |     N                   | R                       | R                       | <package>/<.launch>               |
  +-----------------------+-------------------------+-------------------------+-------------------------+-----------------------------------+
  | parent_name           |     N                   | N                       | R                       | <package>/<rapp_name>             |
  +-----------------------+-------------------------+-------------------------+-------------------------+-----------------------------------+
  | required_capabilities |     N                   | O                       | O                       |  TODO                             |
  +-----------------------+-------------------------+-------------------------+-------------------------+-----------------------------------+ 

.. _`Rocon URI`: http://docs.ros.org/indigo/api/rocon_uri/html/


Thre are two kinds of classifications.

Virutal Rapp vs Rapp Implementaion
``````````````````````````````````

If the following two are present, it is a rapp implementation. Otherwise it is a virtual rapp.

* compatibility : Rocon URI
* launch - <package_name>/</.launch>

If it is a rapp impementation, the following three parameters are optional

* icon 
* capabilities

Note: Rapp implementations may be rapps at the top of a rapp chain.

Child Rapps Vs Ancestor Rapp
````````````````````````````

If the following is present, it is a child rapp. Otherwise it is an ancestor rapp.

* parent_name : <package_name>/<rapp name>


Note:

* parent_name and public_interface are mutually exclusive. 
* Child rapps must be rapp implementations
* Ancestor rapps can be either virtual or implementation rapps




.. _`talker.rapp`: https://github.com/robotics-in-concert/rocon_app_platform/blob/hydro-devel/rocon_apps/apps/talker/talker.rapp 

Examples
--------

*Chirp Virtual Ancestor Rapp*

.. code-block:: yaml

    # rocon_apps/chirp
    display: Chirp
    description: Make an audible "chirp" sound.
    icon: rocon_apps/chirp_bubble_icon.png
    public_interface: rocon_apps/chirp.interface
    public_parameters: rocon_apps/chirp.parameters

*Chirp Child Implementation Rapp*

.. code-block:: yaml

    # turtlebot_apps/chirp
    description: Make a "moo" sound.
    launch: turtlebot_apps/chirp.launch
    compatibility: rocon:/turtlebot
    parent_name: rocon_apps/chirp



Export
------

Rapp is exported via package.xml. Indexer searches for `rocon_app` in export tag to collect all available rapps.

.. code-block:: xml

    ...
    <export>
      <rocon_app>RELATIVE_PATH_IN_PACKAGE</rocon_app>
      <!--<rocon_app>apps/chirp/chirp.rapp</rocon_app>-->
    </export>
    ...

