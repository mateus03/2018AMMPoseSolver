#ifndef POINT3DREGISTRATIONFUNCTIONINFO_H
#define POINT3DREGISTRATIONFUNCTIONINFO_H

#include <opengv/optimization_tools/objective_function_tools/ObjectiveFunctionInfo.hpp>
#include <opengv/registration/PointRegistrationAdapter.hpp>
#include <opengv/types.hpp>
#include <iostream>

class Point3DRegistrationFunctionInfo : public ObjectiveFunctionInfo {
public:
  Point3DRegistrationFunctionInfo(const opengv::registration::PointRegistrationAdapter & adapter);
  ~Point3DRegistrationFunctionInfo();

  double objective_function_value(const opengv::rotation_t & rotation,
                                          const opengv::translation_t & translation);
  opengv::rotation_t rotation_gradient(const opengv::rotation_t & rotation,
							      const opengv::translation_t & translation);
  opengv::translation_t translation_gradient(const opengv::rotation_t & rotation,
  const opengv::translation_t & translation);

private:
  Eigen::MatrixXd Mrt;
  Eigen::MatrixXd Mr; 
  Eigen::MatrixXd pp;
  Eigen::VectorXd vt; 
  Eigen::VectorXd vr;
  size_t numPoints;

};

#endif
