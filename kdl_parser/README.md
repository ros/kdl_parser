![height="362",width="524"](doc/parser2.png)

## What is the KDL parser

If you want to take advantage of the powerful features of the
[Kinematics and Dynamics Library](https://index.ros.org/p/orocos_kinematics_dynamics),
the kdl parser provides an easy way to construct a full KDL Tree object.
Starting from either a
[URDF xml description](https://index.ros.org/p/urdf) or a Collada xml
description of your robot, the kdl parser automatically generates a KDL
Tree.

## Tutorials

- [Start using the KDL parser](#start-using-the-kdl-parser):
  This tutorial teaches you how to create a KDL Tree from a
  [URDF](https://index.ros.org/p/urdf) file

## Start using the KDL parser

- **Note:** This tutorial assumes you have already
  [created your own URDF file](https://index.ros.org/p/urdf) or that you
  are working with the existing PR2 URDF file.
- **Description:** This tutorial teaches you how to create a KDL Tree
  from a [URDF](https://index.ros.org/p/urdf) file
- **Tutorial Level:** BEGINNER

### Building the KDL parser



```
$ rosdep install kdl_parser
```

 This will install all the external dependencies
for the `kdl_parser`. To build the package, run:



```
$ rosmake kdl_parser
```



### Using in your code

First, add the KDL parser as a dependency to your package.xml (in ROS
[fuerte](fuerte) or earlier it's `manifest.xml`) file:



```
<package>
    ...
    <build_depend package="kdl_parser" />
    ...
    <run_depend package="kdl_parser" />
    ...
  </package>
```

 To start using the KDL parser in your C++ code,
include the following file:



```cplusplus
  #include <kdl_parser/kdl_parser.hpp>
```

 Now there are different ways to proceed. You
can construct a KDL tree from a urdf in various forms:

#### From a file



```cplusplus
   KDL::Tree my_tree;
   if (!kdl_parser::treeFromFile("filename", my_tree)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
   }
```

 To create the PR2 URDF file, run the following
command: 

```
$ rosrun xacro xacro.py `rospack find pr2_description`/robots/pr2.urdf.xacro > pr2.urdf
```



#### From the parameter server



```cplusplus
   KDL::Tree my_tree;
   ros::NodeHandle node;
   std::string robot_desc_string;
   node.param("robot_description", robot_desc_string, std::string());
   if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
   }
```



#### From an xml element



```cplusplus
   KDL::Tree my_tree;
   TiXmlDocument xml_doc;
   xml_doc.Parse(xml_string.c_str());
   xml_root = xml_doc.FirstChildElement("robot");
   if (!xml_root){
      ROS_ERROR("Failed to get robot from xml document");
      return false;
   }
   if (!kdl_parser::treeFromXml(xml_root, my_tree)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
   }
```



#### From a URDF model



```cplusplus
   KDL::Tree my_tree;
   urdf::Model my_model;
   if (!my_model.initXml(....)){
      ROS_ERROR("Failed to parse urdf robot model");
      return false;
   }
   if (!kdl_parser::treeFromUrdfModel(my_model, my_tree)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
   }
```



For more details, take a look at the
[API documentation](http://www.ros.org/doc/api/kdl_parser/html/namespacekdl__parser.html).

## Tutorials

- [Start using the KDL parser](#start-using-the-kdl-parser):
  This tutorial teaches you how to create a KDL Tree from a
  [URDF](https://index.ros.org/p/urdf) file

## Troubleshooting

### Reporting Errors

If you encounter a problem that isn't addressed on this page, please
[file a ticket](https://code.ros.org/trac/ros-pkg/newticket?component=robot_model&keywords=kdl_parser&owner=meeussen)

The current list of open issues is available on
[trac](https://code.ros.org/trac/ros-pkg/query?status=assigned&status=new&status=reopened&group=milestone&order=priority&col=id&col=summary&col=status&col=type&col=priority&col=milestone&col=component&keywords=~kdl_parser)

### Common Problems When Using Package

(There are currently no recorded commonly occurring errors)
