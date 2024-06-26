#pragma once
#include "typedefs.hpp"
// #include "trajopt/robot_interface.hpp"
// #include "trajopt/environment_interface.hpp"
#include "trajopt/sim_utils.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "macros.h"
#include <vector>
#include <memory>
#include <string>

namespace trajopt {

class TRAJOPT_API Configuration {
public:
    virtual void SetDOFValues(const DblVec& dofs) = 0;
    virtual void GetDOFLimits(DblVec& lower, DblVec& upper) const = 0;
    virtual DblVec GetDOFValues() = 0;
    virtual int GetDOF() const = 0;
    virtual EnvironmentInterface::Ptr GetEnv() = 0;
    virtual DblMatrix PositionJacobian(int link_ind, const Eigen::Vector3d& pt) const = 0;
    virtual DblMatrix RotationJacobian(int link_ind) const = 0;
    virtual bool DoesAffect(const KinBodyInterface::Link& link) = 0;
    virtual std::vector<KinBodyInterface::Ptr> GetBodies() = 0;
    virtual std::vector<KinBodyInterface::LinkPtr> GetAffectedLinks() = 0;
    virtual void GetAffectedLinks(std::vector<KinBodyInterface::LinkPtr>& links, bool only_with_geom, std::vector<int>& link_inds) = 0;
    virtual DblVec RandomDOFValues() = 0;
    virtual ~Configuration() {}

    struct Saver {
        virtual ~Saver() {}
    };

    typedef std::shared_ptr<Saver> SaverPtr;

    struct GenericSaver : public Saver {
        DblVec dofvals;
        Configuration* parent;
        GenericSaver(Configuration* _parent) : dofvals(_parent->GetDOFValues()), parent(_parent) {}
        ~GenericSaver() {
            parent->SetDOFValues(dofvals);
        }
    };

    virtual SaverPtr Save() {
        return SaverPtr(new GenericSaver(this));
    }
};

typedef std::shared_ptr<Configuration> ConfigurationPtr;

/**
Stores a robot and the active degrees of freedom  
*/
class TRAJOPT_API RobotAndDOF : public Configuration {
public:
    RobotAndDOF(RobotInterface::Ptr _robot, const IntVec& _joint_inds, int _affinedofs = 0, const Eigen::Vector3d& _rotationaxis = Eigen::Vector3d(0, 0, 1)) :
        robot(_robot), joint_inds(_joint_inds), affinedofs(_affinedofs), rotationaxis(_rotationaxis) {}

    void SetDOFValues(const DblVec& dofs);
    void GetDOFLimits(DblVec& lower, DblVec& upper) const;
    DblVec GetDOFValues();
    int GetDOF() const;
    virtual EnvironmentInterface::Ptr GetEnv() { return env; }
    IntVec GetJointIndices() const { return joint_inds; }
    DblMatrix PositionJacobian(int link_ind, const Eigen::Vector3d& pt) const;
    DblMatrix RotationJacobian(int link_ind) const;
    RobotInterface::Ptr GetRobot() const { return robot; }
    virtual std::vector<KinBodyInterface::Ptr> GetBodies();
    bool DoesAffect(const KinBodyInterface::Link& link);
    std::vector<KinBodyInterface::LinkPtr> GetAffectedLinks();
    void GetAffectedLinks(std::vector<KinBodyInterface::LinkPtr>& links, bool only_with_geom, std::vector<int>& link_inds);
    DblVec RandomDOFValues();

    // struct RobotSaver : public Saver {
    //     RobotSaver(RobotInterface::Ptr robot) : saver(*robot) {}
    //     Eigen::Isometry3d saver;
    // };

    // SaverPtr Save() {
    //     return SaverPtr(new RobotSaver(robot));
    // }

    void SetRobotActiveDOFs();

private:
    RobotInterface::Ptr robot;
    EnvironmentInterface::Ptr env;
    IntVec joint_inds;
    int affinedofs;
    Eigen::Vector3d rotationaxis;
};

typedef std::shared_ptr<RobotAndDOF> RobotAndDOFPtr;

}


//////////////////////////////////////////////////////////////////////////////////////////////

// #pragma once
// #include "typedefs.hpp"
// #include <openrave/openrave.h>
// #include "macros.h"
// namespace trajopt {


// class TRAJOPT_API Configuration {
// public:
  
//   virtual void SetDOFValues(const DblVec& dofs) = 0;
//   virtual void GetDOFLimits(DblVec& lower, DblVec& upper) const = 0;
//   virtual DblVec GetDOFValues() = 0;
//   virtual int GetDOF() const = 0;
//   virtual OpenRAVE::EnvironmentBasePtr GetEnv() = 0;
//   virtual DblMatrix PositionJacobian(int link_ind, const OR::Vector& pt) const = 0;
//   virtual DblMatrix RotationJacobian(int link_ind) const = 0;
//   virtual bool DoesAffect(const KinBody::Link& link) = 0;
//   virtual vector<OpenRAVE::KinBodyPtr> GetBodies() = 0;
//   virtual std::vector<KinBody::LinkPtr> GetAffectedLinks() = 0;
//   virtual void GetAffectedLinks(std::vector<KinBody::LinkPtr>& links, bool only_with_geom, vector<int>& link_inds) = 0;
//   virtual DblVec RandomDOFValues() = 0;  
//   virtual ~Configuration() {}
//   struct Saver {
//     virtual ~Saver(){}
//   };
//   typedef boost::shared_ptr<Saver> SaverPtr;
//   struct GenericSaver : public Saver {
//     DblVec dofvals;
//     Configuration* parent;
//     GenericSaver(Configuration* _parent) : dofvals(_parent->GetDOFValues()), parent(_parent) {}
//     ~GenericSaver() {
//       parent->SetDOFValues(dofvals);
//     }
//   }; // inefficient
  
//   virtual SaverPtr Save() {
//     return SaverPtr(new GenericSaver(this));
//   }


// };
// typedef boost::shared_ptr<Configuration> ConfigurationPtr;

// /**
// Stores an OpenRAVE robot and the active degrees of freedom  
// */
// class TRAJOPT_API RobotAndDOF : public Configuration {
// public:
//   RobotAndDOF(OR::KinBodyPtr _robot, const IntVec& _joint_inds, int _affinedofs=0, const OR::Vector _rotationaxis=OR::Vector(0,0,1)) :
//     robot(_robot), joint_inds(_joint_inds), affinedofs(_affinedofs), rotationaxis(_rotationaxis) {}

//   void SetDOFValues(const DblVec& dofs);
//   void GetDOFLimits(DblVec& lower, DblVec& upper) const;
//   DblVec GetDOFValues();
//   int GetDOF() const;
//   virtual OpenRAVE::EnvironmentBasePtr GetEnv() {return robot->GetEnv();};  
//   IntVec GetJointIndices() const {return joint_inds;}
//   DblMatrix PositionJacobian(int link_ind, const OR::Vector& pt) const;
//   DblMatrix RotationJacobian(int link_ind) const;
//   OR::RobotBasePtr GetRobot() const {return boost::dynamic_pointer_cast<RobotBase>(robot);}
//   virtual vector<OpenRAVE::KinBodyPtr> GetBodies();  
//   bool DoesAffect(const KinBody::Link& link);
//   std::vector<KinBody::LinkPtr> GetAffectedLinks();
//   void GetAffectedLinks(std::vector<KinBody::LinkPtr>& links, bool only_with_geom, vector<int>& link_inds);
//   DblVec RandomDOFValues();

//   struct RobotSaver : public Saver {
//     OpenRAVE::KinBody::KinBodyStateSaver saver;
//     RobotSaver(OpenRAVE::KinBodyPtr robot) : saver(robot) {}
//   };
//   SaverPtr Save() {
//     return SaverPtr(new RobotSaver(robot));
//   }
//   void SetRobotActiveDOFs();
  
// private:
//   OpenRAVE::KinBodyPtr robot;
//   IntVec joint_inds;
//   int affinedofs;
//   OR::Vector rotationaxis;
// };
// typedef boost::shared_ptr<RobotAndDOF> RobotAndDOFPtr;

// }
