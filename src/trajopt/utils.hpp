#pragma once
#include "typedefs.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <iterator>
#include <cmath>
#include "trajopt/typedefs.hpp"
#include "sco/modeling.hpp"
#include "utils/basic_array.hpp"
#include "macros.h"


namespace trajopt {

// Using Eigen for vectors and quaternions
using Vector3 = Eigen::Vector3d;
using Vector4 = Eigen::Vector4d;
using VectorXd = Eigen::VectorXd;
using Quaternion = Eigen::Quaterniond;

struct Transform {
    Vector3 trans;
    Quaternion rot;

    Transform() : trans(Vector3::Zero()), rot(Quaternion::Identity()) {}
};


/**
Extract trajectory array from solution vector x using indices in array vars
*/

/// TODO: This should be imported in typedefs.hpp
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> TrajArray;
typedef std::vector<double> DblVec;
typedef std::vector<int> IntVec;

typedef util::BasicArray<sco::Var> VarArray;
typedef util::BasicArray<sco::AffExpr> AffArray;


TrajArray TRAJOPT_API getTraj(const DblVec& x, const VarArray& vars);
TrajArray TRAJOPT_API getTraj(const DblVec& x, const AffArray& arr);


inline Vector3 toVector3d(const Eigen::VectorXd& v) {
  return Vector3(v[0], v[1], v[2]);
}
inline Vector4 toVector4d(const Eigen::VectorXd& v) {
  return Vector4(v[0], v[1], v[2], v[3]);
}

inline Eigen::Vector4d toEigenVector4d(const Vector4& v) {
  return Eigen::Vector4d(v[0], v[1], v[2], v[3]);
}
inline Eigen::Vector3d toEigenVector3d(const Vector3& v) {
  return Eigen::Vector3d(v[0], v[1], v[2]);
}

Eigen::Matrix3d toRot(const Eigen::Quaterniond& q);

// inline Eigen::Affine3d toEigenTransform(const Vector4d& q, const Vector3d& p) {
//   Eigen::Quaterniond quat(q[0], q[1], q[2], q[3]);
//   Eigen::Vector3d pp = toEigenVector3d(p);
//   Eigen::Affine3d transform = Eigen::Affine3d::Identity();
//   transform.translate(pp);
//   transform.rotate(quat);
//   return transform;
// }

inline Eigen::Isometry3d toEigenTransform(const Vector4& q, const Vector3& p) {
    Eigen::Quaterniond quat(q[0], q[1], q[2], q[3]);
    Eigen::Vector3d pp = toEigenVector3d(p);
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translate(pp);
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
std::vector<T> concat(const std::vector<T>& a, const std::vector<T>& b) {
  std::vector<T> out;
  out.insert(out.end(), a.begin(), a.end());
  out.insert(out.end(), b.begin(), b.end());
  return out;
}

template <typename T> 
std::vector<T> singleton(const T& x) {
  return std::vector<T>(1, x);
}

// enum DOF {
//     DOF_X = 1,
//     DOF_Y = 2,
//     DOF_Z = 4,
//     DOF_RotationAxis = 8,
//     DOF_Rotation3D = 16,
//     DOF_RotationQuat = 32
// };

enum DOFAffine
{
    DOF_NoTransform = 0,
    DOF_X = 1,     ///< can move in the x direction
    DOF_Y = 2,     ///< can move in the y direction
    DOF_Z = 4,     ///< can move in the z direction
    DOF_XYZ=DOF_X|DOF_Y|DOF_Z,     ///< moves in xyz direction

    // DOF_RotationX fields are mutually exclusive
    DOF_RotationAxis = 8,     ///< can rotate around an axis (1 dof)
    DOF_Rotation3D = 16,     ///< can rotate freely (3 dof), the parameterization is
                             ///< theta * v, where v is the rotation axis and theta is the angle about that axis
    DOF_RotationQuat = 32,     ///< can rotate freely (4 dof), parameterization is a quaternion. In order for limits to work correctly, the quaternion is in the space of _vRotationQuatLimitStart. _vRotationQuatLimitStart is always left-multiplied before setting the transform!
    DOF_RotationMask=(DOF_RotationAxis|DOF_Rotation3D|DOF_RotationQuat), ///< mask for all bits representing 3D rotations
    DOF_Transform = (DOF_XYZ|DOF_RotationQuat), ///< translate and rotate freely in 3D space
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

void GetAffineDOFValuesFromTransform(std::vector<double>::iterator itvalues, const Transform& transform, int affinedofs, const Eigen::Vector3d& rotationAxis)
{
    if (affinedofs & DOF_X) {
        *itvalues++ = transform.trans.x();
    }
    if (affinedofs & DOF_Y) {
        *itvalues++ = transform.trans.y();
    }
    if (affinedofs & DOF_Z) {
        *itvalues++ = transform.trans.z();
    }
    if (affinedofs & DOF_RotationAxis) {
        Eigen::AngleAxisd angleAxis(transform.rot);
        double angle = angleAxis.angle();
        Eigen::Vector3d axis = angleAxis.axis();
        if (axis.dot(rotationAxis) < 0) {
            angle = -angle;
        }
        *itvalues++ = angle;
    }
    else if (affinedofs & DOF_Rotation3D) {
        Eigen::AngleAxisd angleAxis(transform.rot);
        double angle = angleAxis.angle();
        Eigen::Vector3d axis = angleAxis.axis();
        *itvalues++ = angle * axis.x();
        *itvalues++ = angle * axis.y();
        *itvalues++ = angle * axis.z();
    }
    else if (affinedofs & DOF_RotationQuat) {
        Eigen::Quaterniond quat(transform.rot);
        *itvalues++ = quat.w();
        *itvalues++ = quat.x();
        *itvalues++ = quat.y();
        *itvalues++ = quat.z();
    }
}

void TRAJOPT_API AddVarArrays(sco::OptProb& prob, int rows, const std::vector<int>& cols, const std::vector<std::string>& name_prefix, const std::vector<VarArray*>& newvars);

void TRAJOPT_API AddVarArray(sco::OptProb& prob, int rows, int cols, const std::string& name_prefix, VarArray& newvars);

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

