#ifndef LINE3DREGISTRATIONFUNCTIONINFO_H
#define LINE3DREGISTRATIONFUNCTIONINFO_H

#include <opengv/optimization_tools/objective_function_tools/ObjectiveFunctionInfo.hpp>
#include <opengv/registration/LineCorrRegistrationAdapter.hpp>
#include <opengv/types.hpp>
#include <iostream>

class Line3DRegistrationFunctionInfo : public ObjectiveFunctionInfo {
public:
  Line3DRegistrationFunctionInfo(const opengv::registration::LineCorrRegistrationAdapter & adapter);
  ~Line3DRegistrationFunctionInfo();

  double objective_function_value(const opengv::rotation_t & rotation,
                                          const opengv::translation_t & translation);
  opengv::rotation_t rotation_gradient(const opengv::rotation_t & rotation,
							      const opengv::translation_t & translation);
  opengv::translation_t translation_gradient(const opengv::rotation_t & rotation,
  const opengv::translation_t & translation);

private:
  Eigen::Matrix<double,3,9> Mrt;
  Eigen::Matrix<double,3,3> Mt;
  Eigen::Matrix<double,9,9> Mr; 
  Eigen::Matrix<double,1,1> pp;
  Eigen::Vector3d vt; 
  Eigen::Matrix<double,9,1> vr;
  size_t numLines;
};

#endif
