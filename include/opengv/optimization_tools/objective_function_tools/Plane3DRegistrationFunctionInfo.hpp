#ifndef PLANE3DREGISTRATIONFUNCTIONINFO_H
#define PLANE3DREGISTRATIONFUNCTIONINFO_H

#include <opengv/optimization_tools/objective_function_tools/ObjectiveFunctionInfo.hpp>
#include <opengv/registration/PlaneRegistrationAdapter.hpp>
#include <opengv/types.hpp>
#include <iostream>

class Plane3DRegistrationFunctionInfo : public ObjectiveFunctionInfo {
public:
  Plane3DRegistrationFunctionInfo(const opengv::registration::PlaneRegistrationAdapter & adapter);
  ~Plane3DRegistrationFunctionInfo();

  double objective_function_value(const opengv::rotation_t & rotation,
                                          const opengv::translation_t & translation);
  opengv::rotation_t rotation_gradient(const opengv::rotation_t & rotation,
							      const opengv::translation_t & translation);
  opengv::translation_t translation_gradient(const opengv::rotation_t & rotation,
  const opengv::translation_t & translation);

private:
  
  Eigen::MatrixXd Mt;
  Eigen::MatrixXd Mr; 
  Eigen::MatrixXd pp;
  Eigen::VectorXd vr;
  Eigen::VectorXd vt;
  Eigen::VectorXd vt_;
  size_t numPlanes;
  double cP;

};

#endif
