Rapp Specification
==================

Rapp infers `rocon_app` or `robot_app` used in `Robotics in Concert`_. This is a meta data installed and executed through `Rapp Manager`_.
It is designed to allow higher level controllers to employ a system which provides the demanded public interface regardless of its platform.
Rapps are classified as *Implementation/Virtual* and *Ancestor/Child* according to the presence of platform-dependent parameters(e.g launch) and Rapp inheritance.

Rapp Types
----------

* **Virtual Ancestor** : is meta rapp contains public interface but any platform-dependent information. not-executable. e.g) `rocon_apps/teleop`_
* **Implementation Child** : is incomplete rapp contains platform dependent data but public APIs. It inherits parameters from parents to become a complete Rapp. e.g) `turtle_concert/teleop`_ 
* **Implementation Ancestor** : is complete rapp contains a full infomration to execute. e.g) `turtle_concert/turtle_stroll`_ 
* **Virtual Child** : is invalid rapp. See Design decisions for more information. 

The detailed parameter specifications described in `Rapp Parameters`_.

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


Design Decisions
----------------

Virutal Rapp vs Rapp Implementaion
``````````````````````````````````

If the following two are present, it is a rapp implementation. Otherwise it is a virtual rapp.

* compatibility : Rocon URI
* launch - <package_name>/</.launch>

If it is a rapp impementation, the following three parameters are optional

* icon 
* capabilities

Child Rapps Vs Ancestor Rapp
````````````````````````````

If the following is present, it is a child rapp. Otherwise it is an ancestor rapp.

* parent_name : <package_name>/<rapp name>

**Note**

* parent_name and public_interface are mutually exclusive. 
* Ancestor rapps can be either virtual or implementation rapps
* Child rapps must be rapp implementations

Parent VS Ancestor
`````````````````

Inheritance may involve multiple rapps if rapps are chained. *Parent* is a rapp where child inherits from. Ancestor is the root of parent if child is referencing another child as parent.
In the example case below, the right most *parent* is an ancestor.

.. code-block:: xml

    child -> parent-child -> parent-child -> parent

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
    icon: rocon_apps/chirp_bubble_icon.png
    public_interface: rocon_apps/chirp.interface
    public_parameters: rocon_apps/chirp.parameters

**Chirp - Implementation Child Rapp**

.. code-block:: yaml

    # turtlebot_apps/chirp
    description: Make a "moo" sound.
    launch: turtlebot_apps/chirp.launch
    compatibility: rocon:/turtlebot
    parent_name: rocon_apps/chirp


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
