^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package collada_urdf
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.8 (2015-09-11)
-------------------
* Removed pcre hack for newer released collada-dom.
* Contributors: Kei Okada

1.11.7 (2015-04-22)
-------------------
* Fixed `#89 <https://github.com/ros/robot_model/issues/89>`_
  Accomplished by loading libpcrecpp before collada-dom.
* Contributors: Kei Okada, William Woodall

1.11.6 (2014-11-30)
-------------------

1.11.5 (2014-07-24)
-------------------

1.11.4 (2014-07-07)
-------------------
* moving to new dependency for urdfdom and urdfdom_headers. https://github.com/ros/rosdistro/issues/4633
* Fix clash with assimp 3.1 in CMake.
* Contributors: Benjamin Chrétien, Tully Foote

1.11.3 (2014-06-24)
-------------------
* Merge pull request `#69 <https://github.com/ros/robot_model/issues/69>`_ from YoheiKakiuchi/indigo-devel-store-original-mesh-name
  storing original mesh file name and location
* storing original mesh file name and location
* Contributors: Ioan A Sucan, YoheiKakiuchi

1.11.2 (2014-03-22)
-------------------
* use new  urdfdom_headers API
* Contributors: Ioan Sucan

1.11.1 (2014-03-20)
-------------------
* Use assimp-dev dep for building
* Contributors: Scott K Logan

1.11.0 (2014-02-21)
-------------------
* Use VERSION_LESS instead of STRLESS
  The version comparison routines were added in cmake 2.8.0
* Fix export API detection (for assimp < 2.0.885)
  It looks like this API was added in Assimp 2.0.885:
  https://github.com/assimp/assimp/commit/ae23c03bd9a0b5f1227dc0042fd98f7206c770a8
* Invert Assimp version detect logic for greater accuracy
* Updated Assimp defines to be more flexible
  This commit is a follow-up to 85b20197671e142044e471df603debd0faf08baf
  Why was export.h removed from assimp < 3.0.0?
* Better feature detection for assimp version
  The unified headers were introduced in Assimp 2.0.1150, so checking for Assimp 3.0.0 is not quite the best solution.
  See https://github.com/assimp/assimp/commit/6fa251c2f2e7a142bb861227dce0c26362927fbc
* Contributors: Scott K Logan

1.10.18 (2013-12-04)
--------------------
* add DEPENDS for kdl_parser
* Contributors: Ioan Sucan

1.10.16 (2013-11-18)
--------------------
* check for CATKIN_ENABLE_TESTING
* fix for compiling collada_to_urdf, add dependency to tf
* add collada_to_urdf.cpp for converting collada file to urdf file

1.10.15 (2013-08-17)
--------------------
* fix `#30 <https://github.com/ros/robot_model/issues/30>`_
