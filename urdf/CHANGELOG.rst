^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package urdf
^^^^^^^^^^^^^^^^^^^^^^^^^^

1.12.5 (2016-10-27)
-------------------
* Added urdf_compatibility.h header to define SharedPtr types (`#160 <https://github.com/ros/robot_model/issues/160>`_)
  This provides portability for downstream packages allowing them to use urdfdom 0.3 or 0.4.
* urdf: Explicitly cast shared_ptr to bool in unit test. (`#158 <https://github.com/ros/robot_model/issues/158>`_)
* Add smart ptr typedefs (`#153 <https://github.com/ros/robot_model/issues/153>`_)
* Addressed gcc6 build error in urdf which was related to use of the isystem flag (`#157 <https://github.com/ros/robot_model/issues/157>`_)
* Remove unneeded dependency on libpcrecpp (`#155 <https://github.com/ros/robot_model/issues/155>`_)
* Contributors: Bence Magyar, Jochen Sprickerhof, Lukas Bulwahn, Maarten de Vries, Robert Haschke

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
