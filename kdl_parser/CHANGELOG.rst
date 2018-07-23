^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kdl_parser
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.13.1 (2018-07-23)
-------------------
* Fix up missing link tags in some XML files. (`#15 <https://github.com/ros/kdl_parser/issues/15>`_)
* Contributors: Chris Lalancette

1.13.0 (2018-04-05)
-------------------
* kdl_parser: switch from TinyXML to TinyXML2 (`#4 <https://github.com/ros/kdl_parser/issues/4>`_)
* Style fixes from ros2 (`#11 <https://github.com/ros/kdl_parser/issues/11>`_)
* Make rostest a test_depend (`#3 <https://github.com/ros/kdl_parser/issues/3>`_)
* update links now that this is in its own repo
* Contributors: Chris Lalancette, Dmitry Rozhkov, Mikael Arguedas

1.12.10 (2017-05-17)
--------------------
* Use result of find_package(orocos_kdl) properly (`#200 <https://github.com/ros/robot_model/issues/200>`_)
  orocos_kdl_LIBRARY_DIRS was not set

1.12.9 (2017-04-26)
-------------------

1.12.8 (2017-03-27)
-------------------
* add Chris and Shane as maintainers (`#184 <https://github.com/ros/robot_model/issues/184>`_)
* fix missed mandatory -std=c++11 flag (`#181 <https://github.com/ros/robot_model/issues/181>`_)
  collada_parser,kdl_parser,urdf: add c++11 flag,
  collada_parser: replace typeof with ansi __typeof\_\_
  builded/tested on gentoo
  Thanks den4ix for the contribution!
* Contributors: Denis Romanchuk, William Woodall

1.12.7 (2017-01-26)
-------------------

1.12.6 (2017-01-04)
-------------------
* Now using ``urdf::*ShredPtr`` instead of ``boost::shared_ptr`` (`#144 <https://github.com/ros/robot_model/issues/144>`_)
* Contributors: Jochen Sprickerhof

1.12.5 (2016-10-27)
-------------------
* fix segfault: safely handle empty robot model (`#154 <https://github.com/ros/robot_model/issues/154>`_)
* Contributors: Robert Haschke

1.12.4 (2016-08-23)
-------------------

1.12.3 (2016-06-10)
-------------------

1.12.2 (2016-04-12)
-------------------

1.12.1 (2016-04-10)
-------------------

1.11.8 (2015-09-11)
-------------------

1.11.7 (2015-04-22)
-------------------

1.11.6 (2014-11-30)
-------------------
* add version dependency on orocos_kdl >= 1.3.0
* Contributors: William Woodall

1.11.5 (2014-07-24)
-------------------
* Update KDL SegmentMap interface to optionally use shared pointers
  The KDL Tree API optionally uses shared pointers on platforms where
  the STL containers don't support incomplete types.
* Contributors: Brian Jensen

1.11.4 (2014-07-07)
-------------------

1.11.3 (2014-06-24)
-------------------
* kdl_parser: Adding kdl library explicitly so that dependees can find it
* Contributors: Jonathan Bohren

1.11.2 (2014-03-22)
-------------------

1.11.1 (2014-03-20)
-------------------

1.11.0 (2014-02-21)
-------------------
* fix test at kdl_parser
* Contributors: YoheiKakiuchi

1.10.18 (2013-12-04)
--------------------
* add DEPENDS for kdl_parser
* Contributors: Ioan Sucan

1.10.16 (2013-11-18)
--------------------
* check for CATKIN_ENABLE_TESTING

1.10.15 (2013-08-17)
--------------------
* fix `#30 <https://github.com/ros/robot_model/issues/30>`_
