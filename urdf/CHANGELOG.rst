^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package urdf
^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.8 (2015-09-11)
-------------------
* Removed pcre hack for newer released collada-dom.
* Fixed link order of libpcrecpp.
* Contributors: Kei Okada

1.11.7 (2015-04-22)
-------------------
* Removed the exporting of Boost and pcre as they are not used in the headers, and added TinyXML because it is.
* Fixed a bug with pcrecpp on Ubuntu > 13.04.
* Contributors: Kei Okada, William Woodall

1.11.6 (2014-11-30)
-------------------
* Add install for static libs needed for Android cross-compilation
* Contributors: Gary Servin

1.11.5 (2014-07-24)
-------------------

1.11.4 (2014-07-07)
-------------------
* moving to new dependency for urdfdom and urdfdom_headers. https://github.com/ros/rosdistro/issues/4633
* Contributors: Tully Foote

1.11.3 (2014-06-24)
-------------------
* fix urdfdom_headers find_package re `ros/rosdistro#4633 <https://github.com/ros/rosdistro/issues/4633>`_
* Contributors: Tully Foote

1.11.2 (2014-03-22)
-------------------

1.11.1 (2014-03-20)
-------------------

1.11.0 (2014-02-21)
-------------------
* fix urdf files for test
* fix test at urdf
* Contributors: YoheiKakiuchi

1.10.18 (2013-12-04)
--------------------
* add DEPENDS for kdl_parser
* Contributors: Ioan Sucan

1.10.16 (2013-11-18)
--------------------
* check for CATKIN_ENABLE_TESTING
* fix for using collada_parser_plugin

1.10.15 (2013-08-17)
--------------------
* fix `#30 <https://github.com/ros/robot_model/issues/30>`_
