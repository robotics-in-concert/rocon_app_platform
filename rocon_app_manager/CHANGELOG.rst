Changelog
=========

0.7.6 (2015-01-12)
------------------
* add log and typo fix
* use auto_rapp_installation param to enable disable installer
* disabl installer logic
* Contributors: Jihoon Lee

0.7.5 (2015-01-08)
------------------
* add rocon_interactions as run_depend closes `#268 <https://github.com/robotics-in-concert/rocon_app_platform/issues/268>`_
* Contributors: Jihoon Lee

0.7.4 (2014-12-30)
------------------
* ignore unncessay remapping rule `#265 <https://github.com/robotics-in-concert/rocon_app_platform/issues/265>`_
* Contributors: Jihoon Lee

0.7.3 (2014-11-21)
------------------
* Merge pull request `#263 <https://github.com/robotics-in-concert/rocon_app_platform/issues/263>`_ from robotics-in-concert/sim_param
  simulated robot support in app manager
* rename sim to simulation
* add sim param in standalone launcher
* sim parameter passing
* process sim param
* add sim param
* [rocon_app_manager] permit esoteric names once more.
* [rocon_app_manager] bugfix stray hub whitelist param, lower casing base topic names, catching the right exception
* bugfix stopping of rapps after virtual implementation upgrade.
* Contributors: Daniel Stonier, Jihoon Lee

0.7.2 (2014-08-25)
------------------
* instead of exception. deprecated warning
* add error message for having old style
* type information added
* re align the args
* align the args
* add the rosbridge setting for using rosbridge on pairing mode
* remove legacy app store url.
* Refined the comment for the preferred defaults
  Also removed legacy app_store_url.
* use preferred instead of selected and defaults
* use yaml format for default app parsing
* preferred or default rapp selection
* rewrote get_available rapp logic
* multiple chirp working
* Fix import to include copy
* Remove copy. Wrong branch :/
* Fix import to include copy
* merging work on public parameters for the rapps/rapp manager.
* Move to use copy.deepcopy
* Set rapp manager namespace handles to be private
* Fix if else re-use
* Correct testing defaults back to standard
* public parameter works
* parses parameters from file
* 0.7.1
* use proper lists for hubs/concerts now roslaunch can handle it.
* fix defaults
* Remove debug prints
* Interactions for turtlebot on indigo update. Remove namespacing for standalone
* rocon_app_manager: CMakeLists.txt(12): error: missing COMPONENTS keyword before 'roslint
* Contributors: Daniel Stonier, DongWook Lee, Jihoon Lee, Kent Sommer, kentsommer

0.7.1 (2014-05-26)
------------------
* use lists instead of semi-colon separated strings for hub/concert whitelists/blacklists now roslaunch can handle it.
* don't try and direct connect to a local hub by default.
* catch and handle a shutdown exception.
* update publisher queue_size to avoid warning in indigo.
* Contributors: Daniel Stonier

0.7.0 (2014-05-06)
------------------
* support for rapp indexing over a remote repository's cached tarball.
* support for rapp indexing over the local ROS_PACKAGE_PATH.
* complete capability support for rapps.
* relieve rapp_manager of pairing responsibilities, now done via `rocon_interactions <http://wiki.ros.org/rocon_interactions>`_.
* revamped launcher file configuration for standalone, multimaster.
* move from tuples to rocon_uri's for platform specifications.
* support for rapp remappings.
* Contributors: Daniel Stonier, Jihoon Lee, Marcus Liebhardt, Piyush Khandelwal, Yujin

0.6.1 (2013-09-11)
------------------
* report details of currently running app.
* disable uuid arg shunting was not enabled for concert clients.

0.6.0 (2013-08-30)
------------------
* disable uuids by default, also fire up the paired invitations by default for convenience.
* use a proper regular expression for the target.
* zeroconf name should match app manager name.
* bugfix remaps which shouldn't remap.
* pass on screen parameter settings from rocon_launch.
* missed an update for the new resource finding rapp lists.
* protect services from initialising in parallel.
* diagnostic flips for pairing mode.

0.5.4 (2013-08-07)
------------------
* public is now 11311
* now private master is 11312
* apply rosparm to set zeroconf parameter
* add gateway and hub as dependeny

0.5.3 (2013-07-22)
------------------
* install concert directory
* adding install rule
* installing pairing_master

0.5.2 (2013-07-17)
------------------
* force faster initialisation of the gateway advertisements in standalone and public pairing.
* push application namespace underneath the node name in standalone mode to match remote control mode styles - for android apps.
* app manager icon parameters as resource names.
* use resource names for rapp lists instead of full paths.
* flag for disabling the cleanup watchdog and consolidating services locally.
* pairing mode cleanup when android device is gone.
* manual pairing invitations now working.
* convenience pause to ensure small apps flip promptly.
* no longer need app manager robot_xxx parameters.
* bugfix missing shutdown of start and stop app services when remote control changes.
* pairing clients infra.
* bugfix the list apps service to respond with correct running apps signature.
* make the default application namespace with a gateway underneath the gateway name, not root.
* publish an icon with the platform information.
* fix publishing of listed/running apps.
* renamed paired launchers to be less confusing.
* remove trivial debug print
* about to move on start app
* latched list apps publisher

0.5.1 (2013-06-10)
------------------
* 0.5.0

0.5.0 (2013-05-27)
------------------
* Point to correct license file
* Removed (now) incorrect comments
* fix bad reference to non-exsistant parameter file.
* fix bad reference to non-exsistant parameter file.
* fix remappings to match roslaunch style
* Merge pull request `#41 <https://github.com/robotics-in-concert/rocon_app_platform/issues/41>` from robotics-in-concert/fix_app_list_file_not_found
  Fix app list file not found
* warnings and errors if app list file not found, fixes `#40 <https://github.com/robotics-in-concert/rocon_app_platform/issues/40>`.
* app list to rapp list
* app_lists args to rapp_lists
* trivial cleanup of a comment.
* auto invite false in paired master.
* trivial comment.
* eliminating duplicated code between paired and concert client launchers.
* minor reorginisation of app manager launchers (more modular).
* android can now finnd us via robot type and name parameters.
* close down quietly if gateway shut down before the app manager.
* flip with default application namespace
* remove old services before updating with new.
* don't do the hard work of advertisements.
* pairing updates.
* a few bugfixes
* starting the pairing
* starting to add components for pairing.
* return values from error status was wrong
* better errors messages for stop app.
* fix stop app for naturally terminating apps.
* create a useful pointer to the running rapp in the manager while it runs.
* better errors messages for stop app.
* fix stop app for naturally terminating apps.
* create a useful pointer to the running rapp in the manager while it runs.
* apps starts with human readable namespace
* standalone app manager.
* 0.4.0
* gateway info now a msg.
* minor pep8 stuff.
* robot namespace back
* robot namespacing fix
* now it supports action_client and action_server public interface
* remove screen flag in concert_client/gateway
* logs out app compatibility.

0.3.0 (2013-02-05 15:23)
------------------------

0.2.0 (2013-02-05 13:18)
------------------------
* adding rocon_apps dependency
* .app -> .rapp
* correcting wiki url
* no more concert client
* taking the concert client out of the loop
* concert status -> app manager status, part of first redesign.
* has its own status now, labelled statusd till concert client swaps its own out.
* remote_control -> invite, start on general app design
* concert_msgs dependency removed
* parameter cleanup
* common create_rule code moved to rocon_utilities
* much minor refactoring.
* collapse advertisements.

0.1.1 (2013-01-31)
------------------
* advertising list apps, also correcting advertising behaviour in the client.
* remove unused logger.
* stop flipping the platform info.
* advertising the platform info service.
* platform info to rocon_app_manager_msgs
* revert loginfo Rapp->App Manager
* launch apps under a unique namespace so caller_id's are guaranteed to be
  unique.
* refactoring app->rapp.
