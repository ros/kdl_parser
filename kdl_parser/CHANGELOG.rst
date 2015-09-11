^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kdl_parser
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
