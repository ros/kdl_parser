/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#ifndef KDL_PARSER__KDL_PARSER_HPP_
#define KDL_PARSER__KDL_PARSER_HPP_

#include <kdl/tree.hpp>
#include <string>
#include <urdf_model/model.h>
#include <tinyxml2.h>
#include <tinyxml.h>  // NOLINT

#include "kdl_parser/visibility_control.hpp"

namespace kdl_parser
{

/** Constructs a KDL tree from a file, given the file name
 * \param file The filename from where to read the xml
 * \param tree The resulting KDL Tree
 * returns true on success, false on failure
 */
KDL_PARSER_PUBLIC
bool treeFromFile(const std::string & file, KDL::Tree & tree);

/** Constructs a KDL tree from the parameter server, given the parameter name
 * \param param the name of the parameter on the parameter server
 * \param tree The resulting KDL Tree
 * returns true on success, false on failure or if built without ROS
 */
KDL_PARSER_PUBLIC
bool treeFromParam(const std::string & param, KDL::Tree & tree);

/** Constructs a KDL tree from a string containing xml
 * \param xml A string containing the xml description of the robot
 * \param tree The resulting KDL Tree
 * returns true on success, false on failure
 */
KDL_PARSER_PUBLIC
bool treeFromString(const std::string & xml, KDL::Tree & tree);

/** Constructs a KDL tree from a TinyXML2 document
 * \param[in] xml_doc The document containing the xml description of the robot
 * \param[out] tree The resulting KDL Tree
 * \return true on success, false on failure
 */
KDL_PARSER_PUBLIC
bool treeFromXml(const tinyxml2::XMLDocument * xml_doc, KDL::Tree & tree);

/** Constructs a KDL tree from a TinyXML document
 * \param[in] xml_doc The document containing the xml description of the robot
 * \param[out] tree The resulting KDL Tree
 * returns true on success, false on failure
 */
KDL_PARSER_PUBLIC
KDL_PARSER_DEPRECATED("TinyXML API is deprecated, use the TinyXML2 version instead")
bool treeFromXml(TiXmlDocument * xml_doc, KDL::Tree & tree);

/** Constructs a KDL tree from a URDF robot model
 * \param robot_model The URDF robot model
 * \param tree The resulting KDL Tree
 * returns true on success, false on failure
 */
KDL_PARSER_PUBLIC
bool treeFromUrdfModel(const urdf::ModelInterface & robot_model, KDL::Tree & tree);
}  // namespace kdl_parser

#endif  // KDL_PARSER__KDL_PARSER_HPP_
