// Copyright 2020 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <stdio.h>
#include "rbdl/Model.h"
#include "rbdl/Joint.h"
#include "rbdl/Dynamics.h"
#include "rbdl/Kinematics.h"
#include "urdfreader.h"
#include "rbdl/rbdl_math.h"

namespace py = pybind11;

RigidBodyDynamics::Math::MatrixNd CalcPointJacobian(
  RigidBodyDynamics::Model& model,
  const RigidBodyDynamics::Math::VectorNd& Q,
  unsigned int body_id,
  const Eigen::Vector3d& point_position,
  bool update_kinematics)
{
  RigidBodyDynamics::Math::MatrixNd G(3, model.qdot_size);
  G.setZero();
  RigidBodyDynamics::CalcPointJacobian(model, Q, body_id, point_position, G, update_kinematics);
  return G;
}

RigidBodyDynamics::Math::MatrixNd UpdateKinematics(RigidBodyDynamics::Model& model,
  const RigidBodyDynamics::Math::VectorNd& Q,
  const RigidBodyDynamics::Math::VectorNd& QDot,
  const RigidBodyDynamics::Math::VectorNd& QDDot)
{
  RigidBodyDynamics::Math::MatrixNd G(3, model.qdot_size);
  G.setZero();
  RigidBodyDynamics::UpdateKinematics(model, Q, QDot, QDDot);
  return G;
}

void TestAlgorithm(
  RigidBodyDynamics::Model& model,
  const Eigen::VectorXd& Q  )
{
  printf("HI!");
}

void TestAlgorithm2(
  RigidBodyDynamics::Model& model,
  const Eigen::VectorXd& Q,
  const Eigen::MatrixXd& H
)
{
  printf("HI2!");
}

Eigen::Vector3d createPos(double x, double y, double z)
{
  Eigen::Vector3d pos(x, y, z);
  return pos;
}

Eigen::VectorXd createVectorXd(int size)
{
  Eigen::VectorXd v;
  v.resize(size);
  return v;
}

Eigen::MatrixXd createMatrixXd(int rows, int cols)
{
  Eigen::MatrixXd m;
  m.resize(rows, cols);
  return m;
}

Eigen::Matrix3d CalcBodyWorldOrientation2(
  RigidBodyDynamics::Model& model,
  const RigidBodyDynamics::Math::VectorNd& Q,
  const unsigned int body_id,
  bool update_kinematics) {

  return CalcBodyWorldOrientation(model, Q, body_id, update_kinematics);
}

Eigen::Vector3d CalcBodyToBaseCoordinates2(
  RigidBodyDynamics::Model& model,
  const RigidBodyDynamics::Math::VectorNd& Q,
  unsigned int body_id,
  const Eigen::Vector3d& body_point_position,
  bool update_kinematics)
{
  return CalcBodyToBaseCoordinates(model, Q, body_id, body_point_position);
}

Eigen::Vector3d CalcBaseToBodyCoordinates(
  RigidBodyDynamics::Model& model,
  const RigidBodyDynamics::Math::VectorNd& Q,
  unsigned int body_id,
  const Eigen::Vector3d& base_point_position,
  bool update_kinematics)
{
  return CalcBaseToBodyCoordinates(model, Q, body_id, base_point_position);
}

Eigen::Vector3d CalcPointVelocity(
  RigidBodyDynamics::Model& model,
  const RigidBodyDynamics::Math::VectorNd& Q,
  const RigidBodyDynamics::Math::VectorNd& QDot,
  unsigned int body_id,
  const Eigen::Vector3d& body_point_position,
  bool update_kinematics)
{
  return CalcPointVelocity(model, Q, QDot, body_id, body_point_position);
}

Eigen::Vector3d CalcPointAcceleration(
  RigidBodyDynamics::Model& model,
  const RigidBodyDynamics::Math::VectorNd& Q,
  const RigidBodyDynamics::Math::VectorNd& QDot,
  const RigidBodyDynamics::Math::VectorNd& QDDot,
  unsigned int body_id,
  const Eigen::Vector3d& body_point_position,
  bool update_kinematics)
{
  return CalcPointAcceleration(model, Q, QDot, QDDot, body_id, body_point_position);
}

Eigen::VectorXd InverseDynamics(
  RigidBodyDynamics::Model& model,
  const Eigen::VectorXd& Q,
  const Eigen::VectorXd& QDot,
  const Eigen::VectorXd& QDDot,
  std::vector<Eigen::Matrix<double, 6, 1> >& f_ext_eigen)
{
  std::vector<RigidBodyDynamics::Math::SpatialVector>* f_ext = 0;
  std::vector<RigidBodyDynamics::Math::SpatialVector> f_ext_internal;
  for (int i = 0; i < f_ext_eigen.size(); i++)
  {
    f_ext_internal.push_back(f_ext_eigen[i]);
    f_ext = &f_ext_internal;
  }

  Eigen::VectorXd Tau;
  RigidBodyDynamics::InverseDynamics(model, Q, QDot, QDDot, Tau, f_ext);
  return Tau;
}

Eigen::MatrixXd CompositeRigidBodyAlgorithm(
  RigidBodyDynamics::Model & model,
  const Eigen::VectorXd & Q,
  bool update_kinematics)
{
  Eigen::MatrixXd H(model.dof_count, model.dof_count);
  H.setZero();
  RigidBodyDynamics::CompositeRigidBodyAlgorithm(model, Q, H, update_kinematics);
  return H;
}

Eigen::VectorXd NonlinearEffects(
  RigidBodyDynamics::Model& model,
  const Eigen::VectorXd& Q,
  const Eigen::VectorXd& QDot)
{
  RigidBodyDynamics::Math::VectorNd H(model.qdot_size);
  H.setZero();
  RigidBodyDynamics::NonlinearEffects(model, Q, QDot, H);
  return H;
}

Eigen::VectorXd ForwardDynamics(
  RigidBodyDynamics::Model& model,
  const Eigen::VectorXd& Q,
  const Eigen::VectorXd& QDot,
  const Eigen::VectorXd& Tau,
  std::vector<Eigen::Matrix<double, 6, 1> >& f_ext_eigen)
{
  std::vector<RigidBodyDynamics::Math::SpatialVector>* f_ext = 0;
  std::vector<RigidBodyDynamics::Math::SpatialVector> f_ext_internal;
  for (int i = 0; i < f_ext_eigen.size(); i++)
  {
    f_ext_internal.push_back(f_ext_eigen[i]);
    f_ext = &f_ext_internal;
  }

  Eigen::VectorXd QDDot;
  QDDot.resize(QDot.size());
  RigidBodyDynamics::ForwardDynamics(model, Q, QDot, Tau, QDDot, f_ext );
  return QDDot;

}


inline RigidBodyDynamics::Math::SpatialTransform Xtrans(const Eigen::Vector3d& r) {
  return RigidBodyDynamics::Math::SpatialTransform(
    RigidBodyDynamics::Math::Matrix3d::Identity(3, 3),
    r
  );
}






PYBIND11_MODULE(pyrbdl, m) {
  m.doc() = R"pbdoc(
        rbdl bindings using pybind11
        -----------------------

        .. currentmodule:: pyrbdl

        .. autosummary::
           :toctree: _generate

    )pbdoc";



  py::class_< RigidBodyDynamics::Body>(m, "Body")
    .def(py::init<const double, const Eigen::Vector3d&, const Eigen::Vector3d&>())
    ;

  py::class_< RigidBodyDynamics::Model> (m, "Model")
    .def(py::init<>())
//    .def_readonly("I", &RigidBodyDynamics::Model::I)
//    .def_readonly("IA", &RigidBodyDynamics::Model::IA)
//    .def_readonly("Ic", &RigidBodyDynamics::Model::Ic)
//    .def_readonly("S", &RigidBodyDynamics::Model::S)
//    .def_readonly("U", &RigidBodyDynamics::Model::U)
//    .def_readonly("X_J", &RigidBodyDynamics::Model::X_J)
//    .def_readonly("X_T", &RigidBodyDynamics::Model::X_T)
//    .def_readonly("X_base", &RigidBodyDynamics::Model::X_base)
//    .def_readonly("X_lambda", &RigidBodyDynamics::Model::X_lambda)
//    .def_readonly("a", &RigidBodyDynamics::Model::a)
//    .def_readonly("c", &RigidBodyDynamics::Model::c)
//    .def_readonly("c_J", &RigidBodyDynamics::Model::c_J)
    .def_readonly("dof_count", &RigidBodyDynamics::Model::dof_count)
//    .def_readonly("f", &RigidBodyDynamics::Model::f)
//    .def_readonly("fixed_body_discriminator", &RigidBodyDynamics::Model::fixed_body_discriminator)
    .def_readonly("gravity", &RigidBodyDynamics::Model::gravity)
//    .def_readonly("hc", &RigidBodyDynamics::Model::hc)
//    .def_readonly("mBodies", &RigidBodyDynamics::Model::mBodies)
//    .def_readonly("mBodyNameMap", &RigidBodyDynamics::Model::mBodyNameMap)
//    .def_readonly("mFixedBodies", &RigidBodyDynamics::Model::mFixedBodies)
//    .def_readonly("mFixedJointCount", &RigidBodyDynamics::Model::mFixedJointCount)
//    .def_readonly("mJointUpdateOrder", &RigidBodyDynamics::Model::mJointUpdateOrder)
//    .def_readonly("mJoints", &RigidBodyDynamics::Model::mJoints)
//    .def_readonly("multdof3_w_index", &RigidBodyDynamics::Model::multdof3_w_index)
//    .def_readonly("pA", &RigidBodyDynamics::Model::pA)
//    .def_readonly("previously_added_body_id", &RigidBodyDynamics::Model::previously_added_body_id)
    .def_readonly("q_size", &RigidBodyDynamics::Model::q_size)
    .def_readonly("qdot_size", &RigidBodyDynamics::Model::qdot_size)
//    .def_readonly("v", &RigidBodyDynamics::Model::v)
//    .def_readonly("v_J", &RigidBodyDynamics::Model::v_J)
    .def("set_gravity", &RigidBodyDynamics::Model::set_gravity)
    .def("AddBody", &RigidBodyDynamics::Model::AddBody)
    //.def("AddBodySphericalJoint", &RigidBodyDynamics::Model::AddBodySphericalJoint)
    .def("AppendBody", &RigidBodyDynamics::Model::AppendBody)
    .def("AddBodyCustomJoint", &RigidBodyDynamics::Model::AddBodyCustomJoint)
    .def("GetBodyId", &RigidBodyDynamics::Model::GetBodyId)
    .def("GetBodyName", &RigidBodyDynamics::Model::GetBodyName)
    .def("IsFixedBodyIdGetBodyName", &RigidBodyDynamics::Model::IsFixedBodyId)
    .def("IsBodyId", &RigidBodyDynamics::Model::IsBodyId)
    .def("GetParentBodyId", &RigidBodyDynamics::Model::GetParentBodyId)
    .def("GetJointFrame", &RigidBodyDynamics::Model::GetJointFrame)
    .def("SetJointFrame", &RigidBodyDynamics::Model::SetJointFrame)
    //.def("GetQuaternion", &RigidBodyDynamics::Model::GetQuaternion)
    //.def("SetQuaternion", &RigidBodyDynamics::Model::SetQuaternion)
    ;

  py::class_<RigidBodyDynamics::Joint>(m, "Joint")
    .def(py::init<>())
    .def(py::init<Eigen::Matrix<double, 6, 1> >())
    .def(py::init< RigidBodyDynamics::JointType>())
    .def("validate_spatial_axis", &RigidBodyDynamics::Joint::validate_spatial_axis2)
    ;

  m.def("Xtrans", &Xtrans);

  py::class_<RigidBodyDynamics::Math::SpatialTransform>(m, "SpatialTransform")
    .def(py::init<>())
    ;


  m.def("jcalc", &RigidBodyDynamics::jcalc);

  m.def("jcalc_XJ", &RigidBodyDynamics::jcalc_XJ);
  m.def("jcalc_X_lambda_S", &RigidBodyDynamics::jcalc_X_lambda_S);
  m.def("CalcPointJacobian", &CalcPointJacobian);

  m.def("UpdateKinematics", &UpdateKinematics);

  m.def("CalcBodyToBaseCoordinates", &CalcBodyToBaseCoordinates2);
  m.def("CalcBaseToBodyCoordinates", &CalcBaseToBodyCoordinates);
  m.def("CalcPointVelocity", &CalcPointVelocity);
  m.def("CalcPointAcceleration", &CalcPointAcceleration);
  m.def("CalcBodyWorldOrientation", &CalcBodyWorldOrientation2);


  m.def("InverseDynamics", &InverseDynamics);
  m.def("NonlinearEffects", &NonlinearEffects);
  m.def("CompositeRigidBodyAlgorithm", &CompositeRigidBodyAlgorithm);


  m.def("ForwardDynamics", &ForwardDynamics);
  //todo: m.def("ForwardDynamicsLagrangian", &ForwardDynamicsLagrangian);

  m.def("TestAlgorithm", &TestAlgorithm);
  m.def("TestAlgorithm2", &TestAlgorithm2);

  m.def("createVectorXd", &createVectorXd);
  m.def("createMatrixXd", &createMatrixXd);

  m.def("createPos", &createPos);


  m.def("URDFReadFromFile", &RigidBodyDynamics::Addons::URDFReadFromFile);

  py::enum_<RigidBodyDynamics::JointType>(m, "JointType")
    .value("JointTypeUndefined", RigidBodyDynamics::JointTypeUndefined, "JointTypeUndefined")
    .value("JointTypeRevolute", RigidBodyDynamics::JointTypeRevolute, "JointTypeRevolute")
    .value("JointTypePrismatic", RigidBodyDynamics::JointTypePrismatic, "JointTypePrismatic")
    .value("JointTypeRevoluteX", RigidBodyDynamics::JointTypeRevoluteX, "JointTypeRevoluteX")
    .value("JointTypeRevoluteY", RigidBodyDynamics::JointTypeRevoluteY, "JointTypeRevoluteY")
    .value("JointTypeRevoluteZ", RigidBodyDynamics::JointTypeRevoluteZ, "JointTypeRevoluteZ")
    .value("JointTypeSpherical", RigidBodyDynamics::JointTypeSpherical, "JointTypeSpherical")
    .value("JointTypeEulerZYX", RigidBodyDynamics::JointTypeEulerZYX, "JointTypeEulerZYX")
    .value("JointTypeEulerXYZ", RigidBodyDynamics::JointTypeEulerXYZ, "JointTypeEulerXYZ")
    .value("JointTypeEulerYXZ", RigidBodyDynamics::JointTypeEulerYXZ, "JointTypeEulerYXZ")
    .value("JointTypeEulerZXY", RigidBodyDynamics::JointTypeEulerZXY, "JointTypeEulerZXY")
    .value("JointTypeTranslationXYZ", RigidBodyDynamics::JointTypeTranslationXYZ, "JointTypeTranslationXYZ")
    .value("JointTypeFloatingBase", RigidBodyDynamics::JointTypeFloatingBase, "JointTypeFloatingBase")
    .value("JointTypeFixed", RigidBodyDynamics::JointTypeFixed, "JointTypeFixed")
    .value("JointTypeHelical", RigidBodyDynamics::JointTypeHelical, "JointTypeHelical")
    .value("JointType1DoF", RigidBodyDynamics::JointType1DoF, "JointType1DoF")
    .value("JointType2DoF", RigidBodyDynamics::JointType2DoF, "JointType2DoF")
    .value("JointType3DoF", RigidBodyDynamics::JointType3DoF, "JointType3DoF")
    .value("JointType4DoF", RigidBodyDynamics::JointType4DoF, "JointType4DoF")
    .value("JointType5DoF", RigidBodyDynamics::JointType5DoF, "JointType5DoF")
    .value("JointType6DoF", RigidBodyDynamics::JointType6DoF, "JointType6DoF")
    .value("JointTypeCustom", RigidBodyDynamics::JointTypeCustom, "JointTypeCustom")
    .export_values();


#ifdef VERSION_INFO
  m.attr("__version__") = VERSION_INFO;
#else
  m.attr("__version__") = "dev";
#endif

  m.attr("RBDL_TEST") = py::int_(int(42));
}
