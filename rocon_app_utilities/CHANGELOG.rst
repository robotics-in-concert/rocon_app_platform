Changelog
=========

0.7.13 (2015-07-09)
-------------------
* Merge pull request `#292 <https://github.com/robotics-in-concert/rocon_app_platform/issues/292>`_ from robotics-in-concert/gopher
  Delivery ancestor rapp and some bugfixes
* [rocon_app_manager] trivial
* [rocon_app_utilities] syntax highlighting for rocon_app cmd output.
* [rocon_app_utilites] pep8 for rapp_cmd.py
* [rocon_app_utilities] bugfix undefined variable call to ancestor_name.
  Must be in scope before the exception branching is executed.
* [rocon_app_utilities] pep8 fixes.
* Contributors: Daniel Stonier, Jihoon Lee

0.7.12 (2015-05-27)
-------------------

0.7.11 (2015-05-06)
-------------------

0.7.10 (2015-05-04)
-------------------

0.7.9 (2015-04-28)
------------------

0.7.8 (2015-03-23)
------------------

0.7.7 (2015-02-27)
------------------

0.7.6 (2015-01-12)
------------------
* remove custom platform checker and use rospkg.os_detect in dependency checker closes `#269 <https://github.com/robotics-in-concert/rocon_app_platform/issues/269>`_
* Contributors: Jihoon Lee

0.7.5 (2015-01-08)
------------------

0.7.4 (2014-12-30)
------------------

0.7.3 (2014-11-21)
------------------
* rename sim to simulation
* sim parameter passing
* [rocon_app_manager] bugfix stray hub whitelist param, lower casing base topic names, catching the right exception
* Contributors: Daniel Stonier, Jihoon Lee

0.7.2 (2014-08-25)
------------------
* it updates its indices when it adds or removes repo
* bugfix for `#260 <https://github.com/robotics-in-concert/rocon_app_platform/issues/260>`_ `#253 <https://github.com/robotics-in-concert/rocon_app_platform/issues/253>`_
* now indexer works fine. rapp stores resources path in yaml_data
* now it uses normalised path and provides more debug messages
* add optional arguments on rocon_app cmd tool closes issue `#259 <https://github.com/robotics-in-concert/rocon_app_platform/issues/259>`_
* improve top-level arg parser logic. Now it does not validate the full launch xml though.
* update comments. absolute to relative
* remove debug message
* It marks invalid rapp if it contains tuple based resource. and give error
* update rapp spec document
* relative path working
* in the middle of change to relative path
* add simple indicator
* multiple chirp working
* parses parameters from file
* 0.7.1
* Merge branch 'indigo' into hydro-devel
* rocon_app_utilities: error: unconfigured build_depend on 'rocon_python_utils rocon_uri
* Contributors: Daniel Stonier, Jihoon Lee

0.7.0 (2014-05-06)
------------------
* command line suite for rapp indexing, caching, dependency installing.
* remote repository index caching.
* local ros package path rapp indexer.
* Contributors: Jihoon Lee
