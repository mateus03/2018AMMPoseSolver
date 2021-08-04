#ifndef HYBRIDREGISTRATIONFUNCTIONINFO_H
#define HYBRIDREGISTRATIONFUNCTIONINFO_H

#include <opengv/optimization_tools/objective_function_tools/ObjectiveFunctionInfo.hpp>
#include <opengv/optimization_tools/objective_function_tools/Point3DRegistrationFunctionInfo.hpp>
#include <opengv/optimization_tools/objective_function_tools/Plane3P3DRegistrationFunctionInfo.hpp>
#include <opengv/optimization_tools/objective_function_tools/Line3DRegistrationFunctionInfo.hpp>
#include <opengv/optimization_tools/objective_function_tools/LineIntersection3DRegistrationFunctionInfo.hpp>
#include <opengv/registration/PointRegistrationAdapter.hpp>
#include <opengv/registration/PlaneRegistrationAdapter.hpp>
#include <opengv/registration/LineRegistrationAdapter.hpp>
#include <opengv/registration/LineCorrRegistrationAdapter.hpp>
#include <opengv/types.hpp>
#include <iostream>

class HybridRegistrationFunctionInfo : public ObjectiveFunctionInfo {
public:
  HybridRegistrationFunctionInfo(const opengv::registration::PointRegistrationAdapter & adapterPoints, 
    const opengv::registration::PlaneRegistrationAdapter & adapterPlanes, 
    const opengv::registration::LineRegistrationAdapter & adapterLines, 
    const opengv::registration::LineCorrRegistrationAdapter & adapterLineCorr,
    const double & ptW,const double & plW,const double & liW,const double & lcW);
  ~HybridRegistrationFunctionInfo();

  double objective_function_value(const opengv::rotation_t & rotation,
                                          const opengv::translation_t & translation);
  opengv::rotation_t rotation_gradient(const opengv::rotation_t & rotation,
							      const opengv::translation_t & translation);
  opengv::translation_t translation_gradient(const opengv::rotation_t & rotation,
  const opengv::translation_t & translation);

private:
  Point3DRegistrationFunctionInfo pointFunction;
  Plane3P3DRegistrationFunctionInfo planeFunction;
  LineIntersection3DRegistrationFunctionInfo lineIntFunction;
  Line3DRegistrationFunctionInfo lineFunction;
  double pointWeight, planeWeight, lineIntWeight, lineWeight;

};

#endif
