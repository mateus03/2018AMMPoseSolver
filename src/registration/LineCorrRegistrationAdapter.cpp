#include <opengv/registration/LineCorrRegistrationAdapter.hpp>

opengv::registration::LineCorrRegistrationAdapter::LineCorrRegistrationAdapter(const points_t & points1A, const points_t & points1B, const points_t & points2A, const bearingVectors_t & directionVectors2):
    RegistrationAdapterBase(),
    _points1A(points1A),
    _points1B(points1B),
    _points2A(points2A),
    _directionVectors2(directionVectors2){}

opengv::registration::LineCorrRegistrationAdapter::LineCorrRegistrationAdapter(const points_t & points1A, const points_t & points1B, const points_t & points2A, const bearingVectors_t & directionVectors2, const rotation_t & R12):
    RegistrationAdapterBase(R12),
    _points1A(points1A),
    _points1B(points1B),
    _points2A(points2A),
    _directionVectors2(directionVectors2){}

opengv::registration::LineCorrRegistrationAdapter::LineCorrRegistrationAdapter(const points_t & points1A, const points_t & points1B, const points_t & points2A, const bearingVectors_t & directionVectors2, const rotation_t & R12, const translation_t & t12):
    RegistrationAdapterBase(R12,t12),
    _points1A(points1A),
    _points1B(points1B),
    _points2A(points2A),
    _directionVectors2(directionVectors2){}

opengv::registration::LineCorrRegistrationAdapter::~LineCorrRegistrationAdapter(){}

opengv::point_t opengv::registration::LineCorrRegistrationAdapter::getPoint1A( size_t index ) const
{
  assert(index < _points1A.size());
  return _points1A[index];
}

opengv::point_t opengv::registration::LineCorrRegistrationAdapter::getPoint1B( size_t index ) const
{
  assert(index < _points1B.size());
  return _points1B[index];
}

opengv::point_t opengv::registration::LineCorrRegistrationAdapter::getPoint2A( size_t index ) const
{
  assert(index < _points2A.size());
  return _points2A[index];
}

opengv::bearingVector_t opengv::registration::LineCorrRegistrationAdapter::getDirection2( size_t index )  const
{
  assert(index < _directionVectors2.size());
  return _directionVectors2[index];
}

size_t opengv::registration::LineCorrRegistrationAdapter::getNumberCorrespondences() const{
    return _points1A.size();
}
