Changelog
=========

0.7.6 (2015-01-12)
------------------

0.7.5 (2015-01-08)
------------------

0.7.4 (2014-12-30)
------------------
* volume is now configurable
* Contributors: Jihoon Lee

0.7.3 (2014-11-21)
------------------
* migrate teleop implementation rapps to yujin_ocs
* add waypoint navigation app in package.xml
* [rocon_apps] waypoint navigation meta-rapp.
* Contributors: Daniel Stonier, Jihoon Lee

0.7.2 (2014-08-25)
------------------
* type information added
* install chirp_apps, icons, and talker script
* remove rocon_bubble_icons dependency
* relative path rapps
* cleanup teleop rapp and enable video_teleop virtual rapp
* disable robot teleop and video teleop until resolving capabilitiy
* update rapp description
* merging changes
* postfix for chirp apss
* rename angry_cat to angry_cat_chirp
* multiple chirp working
* have robot_teleop and video teleop
* Merge branch 'indigo' into teleop_rapp_anchestor
* Explain defaults
* Add defaults
* Add rocon license
* Remove Willow Garage license
* parses parameters from file
* launch args for public parameters
* configurable talker
* 0.7.1
* configurable talker
* rocon_apps: splits teleop rapp into virtual and implementation rapp
* rocon_apps: implements teleop rapp anchestor
* Contributors: Daniel Stonier, Jihoon Lee, Kent Sommer, Marcus Liebhardt

0.7.0 (2014-05-06)
------------------
* added local index generator.
* remove legacy pairing information (now handled by `rocon_interactions <http://wiki.ros.org/rocon_interactions>`_).
* new icons, new teleop ancestor rapp.
* upgrade from tuples to rocon uri's for compatibility checks.
* rename interface into public_interface
* remove required from chirp launcher - creates alot of bleeding when it naturally dies!
* make use of required roslaunch tag in rapps, closes `#127 <https://github.com/robotics-in-concert/rocon_app_platform/issues/127>`_.
* rapps now loading from package manifest exports instead of rapp lists, `#121 <https://github.com/robotics-in-concert/rocon_app_platform/issues/121>`_.
* updated maintainers and authors.
* capability suport
* limit sharing of the listener.
* share option for infinite shares, closes `#94 <https://github.com/robotics-in-concert/rocon_app_platform/issues/94>`_
* deprecate the old platform info message, `#88 <https://github.com/robotics-in-concert/rocon_app_platform/issues/88>`_
* capability integration: adds retrievel of available capabilities from the server
* Contributors: Daniel Stonier, Dirk Thomas, Esteve Fernandez, Jihoon Lee, Marcus Liebhardt

0.6.1 (2013-09-11)
------------------
* don't delete listener - chatter_concert uses it.

0.5.4 (2013-08-07)
------------------
* add rospy_tutorials as dependency of rocon_apps

0.5.3 (2013-07-22)
------------------
* chirp became catkin compatible

0.5.2 (2013-07-17)
------------------
* remove ambiguous parameters.
* listener android app moved to android_apps repo.
* pairing clients infra.
* new icons

0.2.0 (2013-02-05 13:18)
------------------------
* install icons.
* .app -> .rapp
* updating jihoon email
* no more concert client

0.1.1 (2013-01-31)
------------------
* screen output for talker, listene and bugfix listener's flip (was
  trying to flip a publisher)
* talker, listener rocon apps.
* refactoring app->rapp.
* catkinized.
* turtle concert apps moved out, also deleted old directory walking app finder.
* alternative app list, ala old style via resource names.
* removing subdirectory navigation
* bugfixes to turtle_stroll_sim
* fix broken turtle_stroll_sim launcher.
* turtle_stroll_sim added.
* platform tuples corrected.
* updates for orchestration.
* turtle updates.
* eclipse files.
* License comments
* chirp.
* migrate demo related packages to rocon_demo
* first moo working for me.
* updates
* updates
* adding kitchen
* updates
* updates
* updates
* updates
* eclipse project files.
* updates
* updates
* adding apps
