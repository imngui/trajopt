#pragma once
#include "typedefs.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <iterator>
#include <cmath>

namespace trajopt {

/**
Extract trajectory array from solution vector x using indices in array vars
*/
TrajArray TRAJOPT_API getTraj(const DblVec& x, const VarArray& vars);
TrajArray TRAJOPT_API getTraj(const DblVec& x, const AffArray& arr);

inline Vector3d toVector3d(const Eigen::Vector3d& v) {
  return Vector3d(v.x(), v.y(), v.z());
}
inline Vector4d toVector4d(const Eigen::Vector4d& v) {
  return Vector4d(v.x(), v.y(), v.z(), v.w());
}
Eigen::Matrix3d toRot(const Eigen::Quaterniond& q);

inline Eigen::Affine3d toEigenTransform(const Vector4d& q, const Vector3d& p) {
  Eigen::Quaterniond quat(q[0], q[1], q[2], q[3]);
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.translate(p);
  transform.rotate(quat);
  return transform;
}

inline DblVec trajToDblVec(const TrajArray& x) {
  return DblVec(x.data(), x.data() + x.rows() * x.cols());
}

inline VectorXd concat(const VectorXd& a, const VectorXd& b) {
  VectorXd out(a.size() + b.size());
  out.topRows(a.size()) = a;
  out.bottomRows(b.size()) = b;
  return out;
}

template <typename T>
vector<T> concat(const vector<T>& a, const vector<T>& b) {
  vector<T> out;
  out.insert(out.end(), a.begin(), a.end());
  out.insert(out.end(), b.begin(), b.end());
  return out;
}

template <typename T> 
vector<T> singleton(const T& x) {
  return vector<T>(1, x);
}

// Using Eigen for vectors and quaternions
using Vector3 = Eigen::Vector3d;
using Quaternion = Eigen::Quaterniond;

struct Transform {
    Vector3 trans;
    Quaternion rot;

    Transform() : trans(Vector3::Zero()), rot(Quaternion::Identity()) {}
};

enum DOF {
    DOF_X = 1,
    DOF_Y = 2,
    DOF_Z = 4,
    DOF_RotationAxis = 8,
    DOF_Rotation3D = 16,
    DOF_RotationQuat = 32
};

template<typename Iterator>
void GetTransformFromAffineDOFValues(Transform& t, Iterator itvalues, int affinedofs, const Vector3& rotationAxis, bool normalize) {
    if (affinedofs & DOF_X) {
        t.trans.x() = *itvalues++;
    }
    if (affinedofs & DOF_Y) {
        t.trans.y() = *itvalues++;
    }
    if (affinedofs & DOF_Z) {
        t.trans.z() = *itvalues++;
    }
    if (affinedofs & DOF_RotationAxis) {
        double angle = *itvalues++;
        double half_angle = angle * 0.5;
        double sin_half_angle = std::sin(half_angle);
        t.rot.w() = std::cos(half_angle);
        t.rot.vec() = rotationAxis * sin_half_angle;
    } else if (affinedofs & DOF_Rotation3D) {
        if (normalize) {
            double x = *itvalues++;
            double y = *itvalues++;
            double z = *itvalues++;
            double norm = std::sqrt(x * x + y * y + z * z);
            if (norm > 0) {
                double sin_half_norm = std::sin(0.5 * norm);
                double norm_factor = sin_half_norm / norm;
                t.rot.w() = std::cos(0.5 * norm);
                t.rot.vec() = Vector3(x, y, z) * norm_factor;
            } else {
                t.rot = Quaternion::Identity(); // identity quaternion
            }
        } else {
            t.rot = Quaternion(0, 0.5 * *itvalues++, 0.5 * *itvalues++, 0.5 * *itvalues++);
        }
    } else if (affinedofs & DOF_RotationQuat) {
        t.rot.x() = *itvalues++;
        t.rot.y() = *itvalues++;
        t.rot.z() = *itvalues++;
        t.rot.w() = *itvalues++;
        if (normalize) {
            t.rot.normalize();
        }
    }
}

void TRAJOPT_API AddVarArrays(OptProb& prob, int rows, const vector<int>& cols, const vector<string>& name_prefix, const vector<VarArray*>& newvars);

void TRAJOPT_API AddVarArray(OptProb& prob, int rows, int cols, const string& name_prefix, VarArray& newvars);

}


////////////////////////////////////////////////////////////////////////////////////////////////////

// #pragma once
// #include "typedefs.hpp"
// #include <openrave/openrave.h>

// namespace trajopt {

// /**
// Extract trajectory array from solution vector x using indices in array vars
// */
// TrajArray TRAJOPT_API getTraj(const DblVec& x, const VarArray& vars);
// TrajArray TRAJOPT_API getTraj(const DblVec& x, const AffArray& arr);



// inline Vector3d toVector3d(const OR::Vector& v) {
//   return Vector3d(v.x, v.y, v.z);
// }
// inline Vector4d toVector4d(const OR::Vector& v) {
//   return Vector4d(v.x, v.y, v.z, v.w);
// }
// Eigen::Matrix3d toRot(const OR::Vector& rq);

// inline OR::Transform toRaveTransform(const Vector4d& q, const Vector3d& p) {
//   return OR::Transform(OR::Vector(q[0], q[1], q[2], q[3]),
//                        OR::Vector(p[0], p[1], p[2]));
// }
// inline DblVec trajToDblVec(const TrajArray& x) {
//   return DblVec(x.data(), x.data()+x.rows()*x.cols());
// }

// inline VectorXd concat(const VectorXd& a, const VectorXd& b) {
//   VectorXd out(a.size()+b.size());
//   out.topRows(a.size()) = a;
//   out.middleRows(a.size(), b.size()) = b;
//   return out;
// }

// template <typename T>
// vector<T> concat(const vector<T>& a, const vector<T>& b) {
//   vector<T> out;
//   vector<int> x;
//   out.insert(out.end(), a.begin(), a.end());
//   out.insert(out.end(), b.begin(), b.end());
//   return out;
// }

// template <typename T> 
// vector<T> singleton(const T& x) {
//   return vector<T>(1,x);
// } 


// void TRAJOPT_API AddVarArrays(OptProb& prob, int rows, const vector<int>& cols, const vector<string>& name_prefix, const vector<VarArray*>& newvars);

// void TRAJOPT_API AddVarArray(OptProb& prob, int rows, int cols, const string& name_prefix, VarArray& newvars);



// }

