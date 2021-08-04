#ifndef LINEINTERSECTION3DREGISTRATIONFUNCTIONINFO_H
#define LINEINTERSECTION3DREGISTRATIONFUNCTIONINFO_H

#include <opengv/optimization_tools/objective_function_tools/ObjectiveFunctionInfo.hpp>
#include <opengv/registration/LineRegistrationAdapter.hpp>
#include <opengv/types.hpp>
#include <iostream>

class LineIntersection3DRegistrationFunctionInfo : public ObjectiveFunctionInfo {
public:
  LineIntersection3DRegistrationFunctionInfo(const opengv::registration::LineRegistrationAdapter & adapter);
  ~LineIntersection3DRegistrationFunctionInfo();

  double objective_function_value(const opengv::rotation_t & rotation,
                                          const opengv::translation_t & translation);
  opengv::rotation_t rotation_gradient(const opengv::rotation_t & rotation,
							      const opengv::translation_t & translation);
  opengv::translation_t translation_gradient(const opengv::rotation_t & rotation,
  const opengv::translation_t & translation);

private:
  Eigen::MatrixXd M;
  size_t numLines;

};

#endif
