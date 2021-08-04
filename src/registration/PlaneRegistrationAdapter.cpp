#include <opengv/registration/PlaneRegistrationAdapter.hpp>

opengv::registration::PlaneRegistrationAdapter::PlaneRegistrationAdapter(const planes_t & planes1, const planes_t & planes2):
    RegistrationAdapterBase(),
    _planes1(planes1), 
    _planes2(planes2){}

opengv::registration::PlaneRegistrationAdapter::PlaneRegistrationAdapter(const planes_t & planes1, const planes_t & planes2, const rotation_t & R12):
    RegistrationAdapterBase(R12),
    _planes1(planes1), 
    _planes2(planes2){}

opengv::registration::PlaneRegistrationAdapter::PlaneRegistrationAdapter(const planes_t & planes1, const planes_t & planes2, const rotation_t & R12, const translation_t & t12):
    RegistrationAdapterBase(R12,t12),
    _planes1(planes1), 
    _planes2(planes2){}

opengv::registration::PlaneRegistrationAdapter::~PlaneRegistrationAdapter(){}

opengv::plane_t opengv::registration::PlaneRegistrationAdapter::getPlane1( size_t index ) const
{
  assert(index < _planes1.size());
  return _planes1[index];
}

opengv::plane_t opengv::registration::PlaneRegistrationAdapter::getPlane2( size_t index ) const
{
  assert(index < _planes2.size());
  return _planes2[index];
}

size_t opengv::registration::PlaneRegistrationAdapter::getNumberCorrespondences() const{
    return _planes1.size();
}
