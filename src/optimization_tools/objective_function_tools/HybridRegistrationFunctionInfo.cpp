#include <opengv/optimization_tools/objective_function_tools/HybridRegistrationFunctionInfo.hpp>

HybridRegistrationFunctionInfo::HybridRegistrationFunctionInfo(const opengv::registration::PointRegistrationAdapter & adapterPoints, 
    const opengv::registration::PlaneRegistrationAdapter & adapterPlanes, 
    const opengv::registration::LineRegistrationAdapter & adapterLines, 
    const opengv::registration::LineCorrRegistrationAdapter & adapterLineCorr,
    const double & ptW,const double & plW,const double & liW,const double & lcW):
    pointFunction(adapterPoints),planeFunction(adapterPlanes),
    lineIntFunction(adapterLines),lineFunction(adapterLineCorr){

    this->pointWeight = ptW;
    this->planeWeight = plW;
    this->lineIntWeight = liW;
    this->lineWeight = lcW;
}

HybridRegistrationFunctionInfo::~HybridRegistrationFunctionInfo(){}

double HybridRegistrationFunctionInfo::objective_function_value(const opengv::rotation_t & rotation,
                                          const opengv::translation_t & translation){
                                
  // std::cout << this->planeWeight << " cost function: " << planeFunction.objective_function_value(rotation,translation) << std::endl;
  return this->pointWeight*pointFunction.objective_function_value(rotation,translation)
         + this->planeWeight*planeFunction.objective_function_value(rotation,translation)
        //  + this->lineIntWeight*lineIntFunction.objective_function_value(rotation,translation)
         + this->lineWeight*lineFunction.objective_function_value(rotation,translation);

}

opengv::rotation_t HybridRegistrationFunctionInfo::rotation_gradient(const opengv::rotation_t & rotation,
							      const opengv::translation_t & translation){

    return this->pointWeight*pointFunction.rotation_gradient(rotation,translation)
         + this->planeWeight*planeFunction.rotation_gradient(rotation,translation)
        //  + this->lineIntWeight*lineIntFunction.rotation_gradient(rotation,translation)
         + this->lineWeight*lineFunction.rotation_gradient(rotation,translation);
}

opengv::translation_t HybridRegistrationFunctionInfo::translation_gradient(const opengv::rotation_t & rotation,
  const opengv::translation_t & translation){

    return this->pointWeight*pointFunction.translation_gradient(rotation,translation)
         + this->planeWeight*planeFunction.translation_gradient(rotation,translation)
        //  + this->lineIntWeight*lineIntFunction.translation_gradient(rotation,translation)
         + this->lineWeight*lineFunction.translation_gradient(rotation,translation);
}