#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>
#include <memory>
#include <set>
#include "macros.h"
// #include "trajopt/typedefs.hpp"
#include "trajopt/utils.hpp"

namespace trajopt {

enum class ShapeType {
    Box,
    Sphere,
    Cylinder,
    TriMesh,
    Unknown
};

class TRAJOPT_API KinBodyInterface {
public:
    using Ptr = std::shared_ptr<KinBodyInterface>;

    class Link {
    public:
        using Ptr = std::shared_ptr<Link>;
        virtual ~Link() = default;

        virtual Eigen::Isometry3d GetTransform() const = 0;
        virtual int GetIndex() const = 0;
        virtual std::string GetName() const = 0;
        virtual bool HasGeometries() const = 0;
        virtual KinBodyInterface::Ptr GetParent() const = 0;
    };
    using LinkPtr = std::shared_ptr<Link>;


    class Joint {
    public:
        using Ptr = std::shared_ptr<Joint>;

        virtual ~Joint() = default;

        virtual int GetDOFIndex() const = 0;
        // virtual Eigen::Isometry3d GetTransform() const = 0;
        // virtual Eigen::VectorXd GetValues() const = 0;
        // virtual void SetValues(const Eigen::VectorXd& vals) = 0;
        // virtual std::string GetName() const = 0;
        // virtual int GetIndex() const = 0;
        // virtual int GetDOF() const = 0;
        // virtual bool IsActive() const = 0;
        // virtual bool IsFixed() const = 0;
        // virtual bool IsContinuous() const = 0;
        // virtual bool IsLimited() const = 0;
        // virtual double GetMinLimit() const = 0;
        // virtual double GetMaxLimit() const = 0;
        // virtual Eigen::VectorXd GetVelocities() const = 0;
        // virtual void SetVelocities(const Eigen::VectorXd& vals) = 0;
        // virtual Eigen::VectorXd GetAccelerations() const = 0;
        // virtual void SetAccelerations(const Eigen::VectorXd& vals) = 0;
    };
    using JointPtr = std::shared_ptr<Joint>;
    

    virtual ~KinBodyInterface() = default;

    virtual Eigen::Isometry3d GetLinkTransform(const std::string& name) const = 0;
    virtual int GetLinkIndex(const std::string& name) const = 0;
    virtual std::vector<Eigen::Isometry3d> GetAllLinkTransforms() const = 0;
    virtual std::vector<std::string> GetAllLinkNames() const = 0;
    virtual LinkPtr GetLink(const std::string& name) const = 0;
    virtual std::vector<LinkPtr> GetLinks() const = 0;
    virtual JointPtr GetJoint(const std::string& name) const = 0;
};

class TRAJOPT_API RobotInterface : public KinBodyInterface {
public:
    using Ptr = std::shared_ptr<RobotInterface>;

    class Manipulator {
    public:
        using Ptr = std::shared_ptr<Manipulator>;

        virtual ~Manipulator() = default;

        virtual std::vector<int> GetArmIndices() const = 0;
        virtual std::vector<int> GetBaseIndices() const = 0;
        virtual std::vector<int> GetArmDOFIndices() const = 0;
        virtual std::vector<int> GetBaseDOFIndices() const = 0;
        virtual std::vector<int> GetGripperIndices() const = 0;
        virtual std::vector<int> GetGripperDOFIndices() const = 0;
    };

    using ManipulatorPtr = std::shared_ptr<Manipulator>;

    virtual ~RobotInterface() = default;

    virtual bool DoesAffect(const std::vector<int>& dof_inds, int link_ind) const = 0;

    // virtual const IntVec& GetActiveDOFIndices() = 0;
    virtual const std::vector<int>& GetActiveDOFIndices() = 0;

    virtual int GetAffineDOF() = 0;

    virtual const Eigen::Vector3d GetAffineRotationAxis() = 0;

    virtual ManipulatorPtr GetManipulatorByName(const std::string& name) const = 0;

    virtual void GetAttached(std::set<KinBodyInterface::Ptr>& setAttached) const = 0;

    virtual KinBodyInterface::LinkPtr IsGrabbing(KinBodyInterface::Ptr body) const = 0;

    virtual std::vector<KinBodyInterface::Ptr> GetGrabbed(std::vector<KinBodyInterface::Ptr>& grabbed) const = 0;

    virtual void GetDOFLimits(std::vector<double>& lower, std::vector<double>& upper, const std::vector<int>& dofindices = std::vector<int>()) const = 0;

    virtual int GetActiveDOF() const = 0;

    virtual void SetActiveDOFs(const std::vector<int>& dofindices, int affine) = 0;

    virtual void GetDOFValues(std::vector<double>& v, const std::vector<int>& dofindices) const = 0;

    virtual void SetDOFValues(const std::vector<double>& values, uint32_t checklimits = false, const std::vector<int>& dofindices = std::vector<int>()) = 0;

    virtual void SetTransform(const Transform& T) = 0;

    virtual Transform GetTransform() const = 0;

    virtual void CalculateActiveJacobian(int index, const Eigen::Vector3d& offset, std::vector<double>& vjacobian) const = 0;

    virtual void ComputeJacobianAxisAngle(const int linkindex, std::vector<double>& jacobian, const std::vector<int>& dofindices = {}) const = 0;

    // virtual 
};

class TRAJOPT_API EnvironmentInterface {
public:
    using Ptr = std::shared_ptr<EnvironmentInterface>;

    virtual ~EnvironmentInterface() = default;

    virtual RobotInterface::Ptr GetRobot() const = 0;
    virtual RobotInterface::Ptr GetRobotByName(const std::string& name) const = 0;
    virtual KinBodyInterface::Ptr GetBody(const std::string& name) const = 0;
    // virtual void PlotAxes(const Eigen::Isometry3d& T, float size, std::vector<std::shared_ptr<void>>& handles) const = 0;
};

}
