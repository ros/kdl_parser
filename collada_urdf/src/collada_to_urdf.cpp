/* Author: Yohei Kakiuchi */
#include <ros/ros.h>
#include <urdf/model.h>

#include <collada_parser/collada_parser.h>
#include <urdf_parser/urdf_parser.h>

#if defined(ASSIMP_UNIFIED_HEADER_NAMES)
#include <assimp/IOSystem.hpp>
#include <assimp/IOStream.hpp>
#include <assimp/Importer.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#else
#include <assimp.hpp>
#if defined(ASSIMP_EXPORT_API)
#include <assimp/export.hpp>
#endif
#include <aiScene.h>
#include <aiPostProcess.h>
#endif

#include <iostream>
#include <fstream>

#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Quaternion.h>

#undef GAZEBO_1_0
#undef GAZEBO_1_3

//#define GAZEBO_1_0
#define GAZEBO_1_3

using namespace urdf;
using namespace std;

bool use_simple_visual = false;
bool use_simple_collision = false;
bool add_gazebo_description = false;
bool use_assimp_export = false;
bool use_same_collision_as_visual = true;
bool rotate_inertia_frame = true;
bool export_collision_mesh = false;

string mesh_dir = "/tmp";
string arobot_name = "";
string output_file = "";
string mesh_prefix = "";

#define PRINT_ORIGIN(os, origin) \
os << "xyz: " << origin.position.x << " " << origin.position.y << " " << origin.position.z << " "; \
{ double r,p,y; origin.rotation.getRPY(r, p, y); \
  os << "rpy: " << r << " " << p << " " << y; }

#define PRINT_ORIGIN_XML(os, origin) \
  os << "xyz=\"" << origin.position.x << " " << origin.position.y << " " << origin.position.z << "\""; \
  { double h___r, h___p, h___y; \
     origin.rotation.getRPY(h___r, h___p, h___y); \
  os << " rpy=\"" << h___r << " " << h___p << " " << h___y << "\""; }

#define PRINT_GEOM(os, geometry) \
  if ( geometry->type == urdf::Geometry::MESH ) { os << "geom: name: " << ((urdf::Mesh *)geometry.get())->filename; }

void assimp_file_export(std::string fname, std::string ofname,
                        std::string mesh_type = "collada") {
#if defined(ASSIMP_EXPORT_API)
  if (fname.find("file://") == 0) {
    fname.erase(0, strlen("file://"));
  }
  Assimp::Importer importer;
  /*
  { // ignore UP_DIRECTION tag in collada
    bool existing;
    importer.SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, true, &existing);
    if(existing) {
      fprintf(stderr, ";; OverWrite : Ignore UP_DIRECTION", existing);
    }
  }
  */
  const aiScene* scene = importer.ReadFile(fname.c_str(),
                                           aiProcess_Triangulate            |
                                           aiProcess_GenNormals             |
                                           aiProcess_JoinIdenticalVertices  |
                                           aiProcess_SplitLargeMeshes       |
                                           aiProcess_OptimizeMeshes         |
                                           aiProcess_SortByPType);

  if (!scene) {
    std::string str( importer.GetErrorString() );
    std::cerr << ";; " << str << std::endl;
    return;
  }

  Assimp::Exporter aexpt;
  aiReturn ret = aexpt.Export(scene, mesh_type, ofname);
  if ( ret != AI_SUCCESS ) {
    std::string str( "assimp error" );
    std::cerr << ";; " << str << std::endl;
  }
#endif
}

// assimp bounding box calculation
void assimp_calc_bbox(string fname, float &minx, float &miny, float &minz,
                      float &maxx, float &maxy, float &maxz) {

  if (fname.find("file://") == 0) {
    fname.erase(0, strlen("file://"));
  }

  Assimp::Importer importer;
  const aiScene* scene = importer.ReadFile(fname.c_str(),
                                           aiProcess_Triangulate            |
                                           aiProcess_JoinIdenticalVertices  |
                                           aiProcess_SortByPType);   // aiProcess_GenNormals
                                                                     // aiProcess_GenSmoothNormals
                                                                     // aiProcess_SplitLargeMeshes
  if (!scene) {
    std::string str( importer.GetErrorString() );
    std::cerr << ";; " << str << std::endl;
    return;
  }

  aiNode *node = scene->mRootNode;

  bool found = false;
  if(node->mNumMeshes > 0 && node->mMeshes != NULL) {
    std::cerr << "Root node has meshes " << node->mMeshes << std::endl;;
    found = true;
  } else {
    for (unsigned int i=0; i < node->mNumChildren; ++i) {
      if(node->mChildren[i]->mNumMeshes > 0 && node->mChildren[i]->mMeshes != NULL) {
        std::cerr << "Child " << i << " has meshes" << std::endl;
        node = node->mChildren[i];
        found = true;
        break;
      }
    }
  }
  if(found == false) {
    std::cerr << "Can't find meshes in file" << std::endl;
    return;
  }

  aiMatrix4x4 transform = node->mTransformation;

  // copy vertices
  maxx = maxy = maxz = -100000000.0;
  minx = miny = minz =  100000000.0;

  std::cerr << ";; num meshes: " << node->mNumMeshes << std::endl;
  for (unsigned int m = 0; m < node->mNumMeshes; m++) {
    aiMesh *a = scene->mMeshes[node->mMeshes[m]];
    std::cerr << ";; num vertices: " << a->mNumVertices << std::endl;

    for (unsigned int i = 0 ; i < a->mNumVertices ; ++i) {
      aiVector3D p;
      p.x = a->mVertices[i].x;
      p.y = a->mVertices[i].y;
      p.z = a->mVertices[i].z;
      p *= transform;

      if ( maxx < p.x ) {
        maxx = p.x;
      }
      if ( maxy < p.y ) {
        maxy = p.y;
      }
      if ( maxz < p.z ) {
        maxz = p.z;
      }
      if ( minx > p.x ) {
        minx = p.x;
      }
      if ( miny > p.y ) {
        miny = p.y;
      }
      if ( minz > p.z ) {
        minz = p.z;
      }
    }
  }
}

void addChildLinkNamesXML(boost::shared_ptr<const Link> link, ofstream& os)
{
  os << "  <link name=\"" << link->name << "\">" << endl;
  if ( !!link->visual ) {
    os << "    <visual>" << endl;

    if (!use_simple_visual) {
      os << "      <origin ";
      PRINT_ORIGIN_XML(os, link->visual->origin);
      os << "/>" << endl;
      os << "      <geometry>" << endl;
      if ( link->visual->geometry->type == urdf::Geometry::MESH ) {
        std::string ifname (((urdf::Mesh *)link->visual->geometry.get())->filename);
        if (ifname.find("file://") == 0) {
          ifname.erase(0, strlen("file://"));
        }
        std::string ofname (mesh_dir + "/" + link->name + "_mesh.dae");

        if (use_assimp_export) {
          // using collada export
          assimp_file_export (ifname, ofname);
        } else {
          // copy to ofname
          std::ofstream tmp_os;
          tmp_os.open(ofname.c_str());
          std::ifstream is;
          is.open(ifname.c_str());
          std::string buf;
          while(is && getline(is, buf)) tmp_os << buf << std::endl;
          is.close();
          tmp_os.close();
        }
        if (mesh_prefix != "") {
          os << "        <mesh filename=\"" << mesh_prefix + "/" + link->name + "_mesh.dae" << "\" scale=\"1 1 1\" />" << endl;
        } else {
          os << "        <mesh filename=\"" << "file://" << ofname << "\" scale=\"1 1 1\" />" << endl;
        }
      }
      os << "      </geometry>" << endl;
    } else {
      // simple visual
      float ax,ay,az,bx,by,bz;
      if ( link->visual->geometry->type == urdf::Geometry::MESH ) {
        assimp_calc_bbox(((urdf::Mesh *)link->visual->geometry.get())->filename,
                         ax, ay, az, bx, by, bz);
      }
      os << "      <origin ";
      urdf::Pose pp = link->visual->origin;

      pp.position.x += ( ax + bx ) / 2 ;
      pp.position.y += ( ay + by ) / 2 ;
      pp.position.z += ( az + bz ) / 2 ;
      PRINT_ORIGIN_XML(os, pp);
      os << "/>" << endl;

      os << "      <geometry>" << endl;
      os << "         <box size=\"" << bx - ax << " " << by - ay << " " << bz - az << "\"/>" << endl;
      os << "      </geometry>" << endl;
    }
    os << "    </visual>" << endl;
  }
  if ( !!link->collision ) {
    os << "    <collision>" << endl;
    if (!use_simple_collision) {
      os << "      <origin ";
      PRINT_ORIGIN_XML(os, link->collision->origin);
      os << "/>" << endl;
      os << "      <geometry>" << endl;

      if ( link->visual->geometry->type == urdf::Geometry::MESH ) {
        std::string ifname;
        if (use_same_collision_as_visual) {
          ifname.assign (((urdf::Mesh *)link->visual->geometry.get())->filename);
        } else {
          ifname.assign (((urdf::Mesh *)link->collision->geometry.get())->filename);
        }
        if (ifname.find("file://") == 0) {
          ifname.erase(0, strlen("file://"));
        }
        std::string oofname;
        if (export_collision_mesh) {
          oofname = link->name + "_mesh.stl";
        } else {
          oofname = link->name + "_mesh.dae";
        }
        std::string ofname = (mesh_dir + "/" + oofname);

        if (use_assimp_export) {
          // using collada export
          if (export_collision_mesh) {
            assimp_file_export (ifname, ofname, "stl");
          } else {
            assimp_file_export (ifname, ofname);
          }
        } else {
          // copy to ofname
          std::ofstream tmp_os;
          tmp_os.open(ofname.c_str());
          std::ifstream is;
          is.open(ifname.c_str());
          std::string buf;
          while(is && getline(is, buf)) tmp_os << buf << std::endl;
          is.close();
          tmp_os.close();
        }
        if (mesh_prefix != "") {
          os << "        <mesh filename=\"" << mesh_prefix + "/" + oofname;
        } else {
          os << "        <mesh filename=\"" << "file://" << ofname;
        }
        os << "\" scale=\"1 1 1\" />" << endl;
      }
      os << "      </geometry>" << endl;
    } else {
      // simple collision
      float ax,ay,az,bx,by,bz;
      if ( link->visual->geometry->type == urdf::Geometry::MESH ) {
        assimp_calc_bbox(std::string ( ((urdf::Mesh *)link->visual->geometry.get())->filename ),
                         ax, ay, az, bx, by, bz);
      }
      os << "      <origin ";
      urdf::Pose pp = link->visual->origin;
      pp.position.x += ( ax + bx ) / 2 ;
      pp.position.y += ( ay + by ) / 2 ;
      pp.position.z += ( az + bz ) / 2 ;
      PRINT_ORIGIN_XML(os, pp);
      os << "/>" << endl;

      os << "      <geometry>" << endl;
      os << "         <box size=\"" << bx - ax << " " << by - ay << " " << bz - az << "\"/>" << endl;
      os << "      </geometry>" << endl;
    }
    os << "    </collision>" << endl;
  }
  if ( !!link->inertial ) {
    if (!rotate_inertia_frame) {
      os << "    <inertial>" << endl;
      os << "      <mass value=\"" << link->inertial->mass << "\" />" << endl;
      os << "      <origin ";
      PRINT_ORIGIN_XML(os, link->inertial->origin);
      os << "/>" << endl;
      os << "      <inertia ixx=\"" << link->inertial->ixx << "\" ";
      os << "ixy=\"" << link->inertial->ixy << "\" ";
      os << "ixz=\"" << link->inertial->ixz << "\" ";
      os << "iyy=\"" << link->inertial->iyy << "\" ";
      os << "iyz=\"" << link->inertial->iyz << "\" ";
      os << "izz=\"" << link->inertial->izz << "\"/>" << endl;
      os << "    </inertial>" << endl;
    } else {
      // rotation should be identity
      os << "    <inertial>" << endl;
      os << "      <mass value=\"" << link->inertial->mass << "\" />" << endl;
      os << "      <origin ";

      tf::Quaternion qt (link->inertial->origin.rotation.x,
                         link->inertial->origin.rotation.y,
                         link->inertial->origin.rotation.z,
                         link->inertial->origin.rotation.w);
      tf::Matrix3x3 mat (qt);
      tf::Matrix3x3 tmat (mat.transpose());
      tf::Matrix3x3 imat (link->inertial->ixx, link->inertial->ixy, link->inertial->ixz,
                          link->inertial->ixy, link->inertial->iyy, link->inertial->iyz,
                          link->inertial->ixz, link->inertial->iyz, link->inertial->izz);
#define DEBUG_MAT(mat)                                                  \
      cout << "#2f((" << mat[0][0] << " " << mat[0][1] << " " << mat[0][2] << ")"; \
      cout << "(" << mat[1][0] << " " << mat[1][1] << " " << mat[1][2] << ")"; \
      cout << "(" << mat[2][0] << " " << mat[2][1] << " " << mat[2][2] << "))" << endl;

#if DEBUG
      DEBUG_MAT(mat);
      DEBUG_MAT(tmat);
      DEBUG_MAT(imat);
#endif

      imat = ( mat * imat * tmat );

#if DEBUG
      DEBUG_MAT(imat);
#endif

      urdf::Pose t_pose (link->inertial->origin);
      t_pose.rotation.clear();

      PRINT_ORIGIN_XML(os, t_pose);
      os << "/>" << endl;

      os << "      <inertia ixx=\"" << imat[0][0] << "\" ";
      os << "ixy=\"" << imat[0][1] << "\" ";
      os << "ixz=\"" << imat[0][2] << "\" ";
      os << "iyy=\"" << imat[1][1] << "\" ";
      os << "iyz=\"" << imat[1][2] << "\" ";
      os << "izz=\"" << imat[2][2] << "\"/>" << endl;
      os << "    </inertial>" << endl;
    }
  }
  os << "  </link>" << endl;

#ifdef GAZEBO_1_0
  if ( add_gazebo_description ) {
    os << "  <gazebo reference=\"" << link->name << "\">" << endl;
    os << "    <material>Gazebo/Grey</material>" << endl;
    //os << "    <mu1>0.9</mu1>" << endl;
    //os << "    <mu2>0.9</mu2>" << endl;
    os << "    <turnGravityOff>false</turnGravityOff>" << endl;
    os << "  </gazebo>" << endl;
  }
#endif

#ifdef GAZEBO_1_3
  if ( add_gazebo_description ) {
    os << "  <gazebo reference=\"" << link->name << "\">" << endl;
    os << "    <mu1>0.9</mu1>" << endl;
    os << "    <mu2>0.9</mu2>" << endl;
    os << "  </gazebo>" << endl;
  }
#endif

  for (std::vector<boost::shared_ptr<Link> >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++)
    addChildLinkNamesXML(*child, os);
}

void addChildJointNamesXML(boost::shared_ptr<const Link> link, ofstream& os)
{
  double r, p, y;
  for (std::vector<boost::shared_ptr<Link> >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++){
    (*child)->parent_joint->parent_to_joint_origin_transform.rotation.getRPY(r,p,y);
    std::string jtype;
    if ( (*child)->parent_joint->type == urdf::Joint::UNKNOWN ) {
      jtype = std::string("unknown");
    } else if ( (*child)->parent_joint->type == urdf::Joint::REVOLUTE ) {
      jtype = std::string("revolute");
    } else if ( (*child)->parent_joint->type == urdf::Joint::CONTINUOUS ) {
      jtype = std::string("continuous");
    } else if ( (*child)->parent_joint->type == urdf::Joint::PRISMATIC ) {
      jtype = std::string("prismatic");
    } else if ( (*child)->parent_joint->type == urdf::Joint::FLOATING ) {
      jtype = std::string("floating");
    } else if ( (*child)->parent_joint->type == urdf::Joint::PLANAR ) {
      jtype = std::string("planar");
    } else if ( (*child)->parent_joint->type == urdf::Joint::FIXED ) {
      jtype = std::string("fixed");
    } else {
      ///error
    }

    os << "  <joint name=\"" <<  (*child)->parent_joint->name << "\" type=\"" << jtype << "\">" << endl;
    os << "    <parent link=\"" << link->name << "\"/>" << endl;
    os << "    <child  link=\"" <<  (*child)->name << "\"/>" << endl;
    os << "    <origin xyz=\"" <<  (*child)->parent_joint->parent_to_joint_origin_transform.position.x << " ";
    os << (*child)->parent_joint->parent_to_joint_origin_transform.position.y << " ";
    os << (*child)->parent_joint->parent_to_joint_origin_transform.position.z;
    os << "\" rpy=\"" << r << " " << p << " " << y << " " << "\"/>" << endl;
    os << "    <axis   xyz=\"" <<  (*child)->parent_joint->axis.x << " ";
    os << (*child)->parent_joint->axis.y << " " << (*child)->parent_joint->axis.z << "\"/>" << endl;
    {
      boost::shared_ptr<urdf::Joint> jt((*child)->parent_joint);

      if ( !!jt->limits ) {
        os << "    <limit ";
        os << "lower=\"" << jt->limits->lower << "\"";
        os << " upper=\"" << jt->limits->upper << "\"";
        if (jt->limits->effort == 0.0) {
          os << " effort=\"100\"";
        } else {
          os << " effort=\"" << jt->limits->effort << "\"";
        }
        os << " velocity=\"" << jt->limits->velocity << "\"";
        os << " />" << endl;
      }
      if ( !!jt->dynamics ) {
        os << "    <dynamics ";
        os << "damping=\"" << jt->dynamics->damping << "\"";
        os << " friction=\"" << jt->dynamics->friction << "\"";
        os << " />" << endl;
      } else {
        os << "    <dynamics ";
        os << "damping=\"0.2\"";
        os << " friction=\"0\"";
        os << " />" << endl;
      }
#ifdef GAZEBO_1_3
#if 0
      os << "    <safety_controller";
      os << " k_position=\"10\"";
      os << " k_velocity=\"10\"";
      os << " soft_lower_limit=\"-10\"";
      os << " soft_upper_limit=\"10\"";
      os << "/>" << endl;
#endif
#endif
    }

    os << "  </joint>" << endl;

    if ( add_gazebo_description ) {
      os << "  <transmission type=\"pr2_mechanism_model/SimpleTransmission\" name=\"";
      os << (*child)->parent_joint->name << "_trans\" >" << endl;
      os << "    <actuator name=\"" << (*child)->parent_joint->name << "_motor\" />" << endl;
      os << "    <joint name=\"" << (*child)->parent_joint->name << "\" />" << endl;
      os << "    <mechanicalReduction>1</mechanicalReduction>" << endl;
      //os << "    <motorTorqueConstant>1</motorTorqueConstant>" << endl;
      //os << "    <pulsesPerRevolution>90000</pulsesPerRevolution>" << endl;
      os << "  </transmission>" << endl;
#ifdef GAZEBO_1_3
      os << "  <gazebo reference=\"" << (*child)->parent_joint->name << "\">" << endl;
      os << "    <cfmDamping>0.4</cfmDamping>" << endl;
      os << "  </gazebo>" << endl;
#endif
    }
    addChildJointNamesXML(*child, os);
  }
}

void printTreeXML(boost::shared_ptr<const Link> link, string name, string file)
{
  std::ofstream os;
  os.open(file.c_str());
  os << "<?xml version=\"1.0\"?>" << endl;
  os << "<robot name=\"" << name << "\"" << endl;
  os << "       xmlns:xi=\"http://www.w3.org/2001/XInclude\"" << endl;
  os << "       xmlns:gazebo=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#gz\"" << endl;
  os << "       xmlns:model=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#model\"" << endl;
  os << "       xmlns:sensor=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor\"" << endl;
  os << "       xmlns:body=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#body\"" << endl;
  os << "       xmlns:geom=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#geom\"" << endl;
  os << "       xmlns:joint=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#joint\"" << endl;
  os << "       xmlns:interface=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#interface\"" << endl;
  os << "       xmlns:rendering=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#rendering\"" << endl;
  os << "       xmlns:renderable=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#renderable\"" << endl;
  os << "       xmlns:controller=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#controller\"" << endl;
  os << "       xmlns:physics=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#physics\">" << endl;

  addChildLinkNamesXML(link, os);

  addChildJointNamesXML(link, os);

  if ( add_gazebo_description ) {
#ifdef GAZEBO_1_0
    // old gazebo (gazebo on ROS Fuerte)
    os << " <gazebo>" << endl;
    os << "   <controller:gazebo_ros_controller_manager" << endl;
    os << "      name=\"gazebo_ros_controller_manager\"" << endl;
    os << "      plugin=\"libgazebo_ros_controller_manager.so\">" << endl;
    os << "     <alwaysOn>true</alwaysOn>" << endl;
    os << "     <updateRate>1000.0</updateRate>" << endl;
    os << "   </controller:gazebo_ros_controller_manager>" << endl;
    os << "  </gazebo>" << endl;
#endif
  }

  os << "</robot>" << endl;
  os.close();
}

namespace po = boost::program_options;
// using namespace std;

int main(int argc, char** argv)
{
  string inputfile;

  po::options_description desc("Usage: collada_to_urdf input.dae [options]\n  Options for collada_to_urdf");
  desc.add_options()
    ("help", "produce help message")
    ("simple_visual,V", "use bounding box for visual")
    ("simple_collision,C", "use bounding box for collision")
    ("export_collision_mesh", "export collision mesh as STL")
    ("add_gazebo_description,G", "add description for using on gazebo")
    ("use_assimp_export,A", "use assimp library for exporting mesh")
    ("use_collision,U", "use collision geometry (default collision is the same as visual)")
    ("original_inertia_rotation,R", "does not rotate inertia frame")
    ("robot_name,N", po::value< vector<string> >(), "output robot name")
    ("mesh_output_dir", po::value< vector<string> >(), "directory for outputing")
    ("mesh_prefix", po::value< vector<string> >(), "prefix of mesh files")
    ("output_file,O", po::value< vector<string> >(), "output file")
    ("input_file", po::value< vector<string> >(), "input file")
    ;

  po::positional_options_description p;
  p.add("input_file", -1);

  po::variables_map vm;
  try {
    po::store(po::command_line_parser(argc, argv).
              options(desc).positional(p).run(), vm);
    po::notify(vm);
  }
  catch (po::error e) {
    cerr << ";; option parse error / " << e.what() << endl;
    return 1;
  }

  if (vm.count("help")) {
    cout << desc << "\n";
    return 1;
  }
  if (vm.count("simple_visual")) {
    use_simple_visual = true;
    cerr << ";; Using simple_visual" << endl;
  }
  if (vm.count("simple_collision")) {
    use_simple_collision = true;
    cerr << ";; Using simple_collision" << endl;
  }
  if (vm.count("add_gazebo_description")) {
    add_gazebo_description = true;
    cerr << ";; Adding gazebo description" << endl;
  }
  if (vm.count("use_assimp_export")) {
#if defined(ASSIMP_EXPORT_API)
    use_assimp_export = true;
#endif
    cerr << ";; Use assimp export" << endl;
  }
  if (vm.count("original_inertia_rotation")) {
    rotate_inertia_frame = false;
    cerr << ";; Does not rotate inertia frame" << endl;
  }
  if (vm.count("export_collision_mesh")) {
    export_collision_mesh = true;
    cerr << ";; erxport collision mesh as STL" << endl;
  }
  if (vm.count("output_file")) {
    vector<string> aa = vm["output_file"].as< vector<string> >();
    cerr << ";; output file is: "
         <<  aa[0] << endl;
    output_file = aa[0];
  }
  if (vm.count("robot_name")) {
    vector<string> aa = vm["robot_name"].as< vector<string> >();
    cerr << ";; robot_name is: "
         <<  aa[0] << endl;
    arobot_name = aa[0];
  }
  if (vm.count("mesh_prefix")) {
    vector<string> aa = vm["mesh_prefix"].as< vector<string> >();
    cerr << ";; mesh_prefix is: "
         <<  aa[0] << endl;
    mesh_prefix = aa[0];
  }
  if (vm.count("mesh_output_dir")) {
    vector<string> aa = vm["mesh_output_dir"].as< vector<string> >();
    cerr << ";; Mesh output directory is: "
         <<  aa[0] << endl;
    mesh_dir = aa[0];
    // check directory existence
    boost::filesystem::path mpath( mesh_dir );
    try {
      if ( ! boost::filesystem::is_directory(mpath) ) {
        boost::filesystem::create_directory ( mpath );
      }
    }
    catch ( boost::filesystem::filesystem_error e ) {
      cerr << ";; mesh output directory error / " << e.what() << endl;
      return 1;
    }
  }
  if (vm.count("input_file")) {
    vector<string> aa = vm["input_file"].as< vector<string> >();
    cerr << ";; Input file is: "
         <<  aa[0] << endl;
    inputfile = aa[0];
  }

  if(inputfile == "") {
    cerr << desc << endl;
    return 1;
  }

  std::string xml_string;
  std::fstream xml_file(inputfile.c_str(), std::fstream::in);
  while ( xml_file.good() )
  {
    std::string line;
    std::getline( xml_file, line);
    xml_string += (line + "\n");
  }
  xml_file.close();

  boost::shared_ptr<ModelInterface> robot;
  if( xml_string.find("<COLLADA") != std::string::npos )
  {
    ROS_DEBUG("Parsing robot collada xml string");
    robot = parseCollada(xml_string);
  }
  else
  {
    ROS_DEBUG("Parsing robot urdf xml string");
    robot = parseURDF(xml_string);
  }
  if (!robot){
    std::cerr << "ERROR: Model Parsing the xml failed" << std::endl;
    return -1;
  }

  if (arobot_name == "") {
    arobot_name = robot->getName();
  }
  if (output_file == "") {
    output_file =  arobot_name + ".urdf";
  }
  printTreeXML (robot->getRoot(), arobot_name, output_file);

  return 0;
}
