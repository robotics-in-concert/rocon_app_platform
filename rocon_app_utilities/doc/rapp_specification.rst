Rapp Specification
==================

Rapp infers `rocon_app` or `robot_app` used in `Robotics in Concert`_. This is a meta data installed and executed through `Rapp Manager`_.
It is designed to allow higher level controllers to employ a system which provides the demanded public interface regardless of its platform.
Rapps are classified as *Implementation/Virtual* and *Ancestor/Child* according to the presence of platform-dependent parameters(e.g launch) and Rapp inheritance.

Rapp Types
----------

* **Virtual Ancestor** : is meta rapp; contains a public interface and no platform-dependent information; not-executable; example:`rocon_apps/teleop`_
* **Implementation Child** : is incomplete rapp; contains platform dependent data but no public APIs. It inherits parameters from parents to become a complete Rapp; example: `turtle_concert/teleop`_ 
* **Implementation Ancestor** : is complete rapp; contains a full infomration to execute; example : `turtle_concert/turtle_stroll`_ 
* **Virtual Child** : is invalid rapp; See Design decisions for more information. 

The detailed parameter specifications described in Rapp Parameters.

.. _`Robotics in Concert`: http://www.robotconcert.org
.. _`Rapp Manager`: http://wiki.ros.org/rocon_app_manager
.. _`Rapp Parameters`: ref:Rapp Parameters
.. _`rocon_apps/teleop`: http://www.github.com/robotics-in-concert/rocon_app_platform/tree/hydro-devel/rocon_apps/apps/teleop/teleop.rapp
.. _`turtle_concert/teleop`: http://www.github.com/robotics-in-concert/rocon_tutorials/tree/hydro-devel/concert_tutorials/turtle_concert/rapps/teleop/teleop.rapp
.. _`turtle_concert/turtle_stroll`: http://www.github.com/robotics-in-concert/rocon_tutorials/tree/hydro-devel/concert_tutorials/turtle_concert/rapps/turtle_stroll/turtle_stroll.rapp

Rapp Parameters
---------------

The following table describes the characteristics of each rapp types and requirements

* R = required
* O = optional
* N = not Allowed
* i = inherited from parent if not present

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
  | icon                  |     O                   | O                       | O :sup:`i`              | relative path to <png, jpg, jpeg> |
  +-----------------------+-------------------------+-------------------------+-------------------------+-----------------------------------+
  | public_interface      |     O                   | O                       | N :sup:`i`              | relative path to <.interface>     |
  +-----------------------+-------------------------+-------------------------+-------------------------+-----------------------------------+
  | public_parameters     |     O                   | O                       | N :sup:`i`              | relative path to <.parameters>    |
  +-----------------------+-------------------------+-------------------------+-------------------------+-----------------------------------+
  | compatibility         |     N                   | R                       | R                       | `Rocon URI`_                      |
  +-----------------------+-------------------------+-------------------------+-------------------------+-----------------------------------+
  | launch                |     N                   | R                       | R                       | relative path to <.launch>        |
  +-----------------------+-------------------------+-------------------------+-------------------------+-----------------------------------+
  | parent_name           |     N                   | N                       | R                       | <package>/<rapp_name>             |
  +-----------------------+-------------------------+-------------------------+-------------------------+-----------------------------------+
  | required_capabilities |     N                   | O                       | O                       | dict(details below)               |
  +-----------------------+-------------------------+-------------------------+-------------------------+-----------------------------------+ 

.. _`Rocon URI`: http://docs.ros.org/indigo/api/rocon_uri/html/


Capability Dependencies
-----------------------

The notion of Capabilities in ROS is that there exists a higher level interface for most "capabilities" that robots posses. 
The format follows the `Capability Interface Specification`_.

.. _`Capability Interface Specification`: http://docs.ros.org/hydro/api/capabilities/html/capabilities.specs.html#module-capabilities.specs.interface 

.. code-block:: yaml

    - name: <capability interface name>
        interface:
          topics:
            requires:
              <interface topic>: <rapp topic>
            provides:
              <interface topic>: <rapp topic>
          services:
            requires:
              <interface service>: <rapp service>
            provides:
              <interface service>: <rapp service>
          actions:
            requires:
              <interface action>: <rapp action>
            provides:
              <interface action>: <rapp action>


Design Decisions
----------------

Virutal Rapp vs Implementaion Rapp
``````````````````````````````````

If the following two are present, it is a rapp implementation. Otherwise it is a virtual rapp.

* compatibility : Rocon URI
* launch - <package_name>/</.launch>

If it is a rapp impementation, the following three parameters are optional

* icon 
* capabilities

Parent VS Ancestor
`````````````````

Inheritance may involve multiple rapps if rapps are chained. *Parent* is a rapp where child inherits from. A child can at the same time be a parent, if anoter child inherits from it. A *Parent* not inheriting from another Child/Parent is a *Ancestor*.

.. code-block:: xml

    child -> parent/child -> parent/child -> parent(ancestor)

Child Rapp Vs Ancestor Rapp
````````````````````````````

If the following is present, it is a child rapp. Otherwise it is an ancestor rapp.

* parent_name : <package_name>/<rapp name>

**Note**

* parent_name and public_interface are mutually exclusive. 
* Ancestor rapps can be either virtual or implementation rapps
* Child rapps must be rapp implementations


Why no Virtual Child? 
`````````````````````

Separation of virtual and implementation rapp is introduced to simplify rapp composition and maximize portability among various platforms.
Virtual Child is dropped because all rapp design choices are satisfiable with the other three.


Examples
--------

**Chirp - Virtual Ancestor Rapp**

.. code-block:: yaml

    # rocon_apps/chirp
    display: Chirp
    description: Make an audible "chirp" sound.
    icon: chirp_bubble_icon.png
    public_interface: chirp.interface
    public_parameters: chirp.parameters

**Chirp - Implementation Child Rapp**

.. code-block:: yaml

    # turtlebot_apps/chirp
    description: Make a "moo" sound.
    launch: chirp.launch
    compatibility: rocon:/turtlebot
    parent_name: rocon_apps/chirp

**Kobuki Random Walker - Capability Enabled Rapp**

.. code-block:: yaml

    # kobuki_apps/random_walker
    display:          Random Walker
    description:      Makes Kobuki wander off and explorer the world
    compatibility:    rocon:/kobuki|turtlebot2
    launch:           random_walker.launch.xml
    public_interface: random_walker.interface
    icon:             random_walker.png
    required_capabilities:
      - name: std_capabilities/DifferentialMobileBase
        interface:
          topics:
            requires:
              '/cmd_vel': 'kobuki_random_walker_controller/commands/velocity'
            provides: []
      - name: kobuki_capabilities/KobukiLED1
        interface:
          topics:
            requires:
              '/kobuki_led1': 'kobuki_random_walker_controller/commands/led1'
            provides: []
      - name: kobuki_capabilities/KobukiLED2
        interface:
          topics:
            requires:
              '/kobuki_led2': 'kobuki_random_walker_controller/commands/led2'
            provides: []

Export
------

Rapp is exported via package.xml. Indexer searches for **rocon_app** in export tag to collect all available rapps.

.. code-block:: xml

    ...
    <export>
      <rocon_app>RELATIVE_PATH_IN_PACKAGE</rocon_app>
      <!--<rocon_app>apps/chirp/chirp.rapp</rocon_app>-->
    </export>
    ...
