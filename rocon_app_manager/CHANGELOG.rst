^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rocon_app_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
