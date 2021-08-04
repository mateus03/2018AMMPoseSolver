#include <opengv/registration/PointRegistrationAdapter.hpp>

opengv::registration::PointRegistrationAdapter::PointRegistrationAdapter(const points_t & points1, const points_t & points2):
    RegistrationAdapterBase(),
    _points1(points1), 
    _points2(points2){}

opengv::registration::PointRegistrationAdapter::PointRegistrationAdapter(const points_t & points1, const points_t & points2, const rotation_t & R12):
    RegistrationAdapterBase(R12),
    _points1(points1), 
    _points2(points2){}

opengv::registration::PointRegistrationAdapter::PointRegistrationAdapter(const points_t & points1, const points_t & points2, const rotation_t & R12, const translation_t & t12):
    RegistrationAdapterBase(R12,t12),
    _points1(points1), 
    _points2(points2){}
    
opengv::registration::PointRegistrationAdapter::~PointRegistrationAdapter(){}

opengv::point_t opengv::registration::PointRegistrationAdapter::getPoint1( size_t index ) const
{
  assert(index < _points1.size());
  return _points1[index];
}

opengv::point_t opengv::registration::PointRegistrationAdapter::getPoint2( size_t index ) const
{
  assert(index < _points2.size());
  return _points2[index];
}

size_t opengv::registration::PointRegistrationAdapter::getNumberCorrespondences() const{
    return _points1.size();
}