#include <opengv/registration/LineRegistrationAdapter.hpp>

opengv::registration::LineRegistrationAdapter::LineRegistrationAdapter(const lines_t & lines1, const lines_t & lines2):
    RegistrationAdapterBase(),
    _lines1(lines1), 
    _lines2(lines2){}

opengv::registration::LineRegistrationAdapter::LineRegistrationAdapter(const lines_t & lines1, const lines_t & lines2, const rotation_t & R12):
    RegistrationAdapterBase(R12),
    _lines1(lines1), 
    _lines2(lines2){}

opengv::registration::LineRegistrationAdapter::LineRegistrationAdapter(const lines_t & lines1, const lines_t & lines2, const rotation_t & R12, const translation_t & t12):
    RegistrationAdapterBase(R12,t12),
    _lines1(lines1), 
    _lines2(lines2){}

opengv::registration::LineRegistrationAdapter::~LineRegistrationAdapter(){}

opengv::line_t opengv::registration::LineRegistrationAdapter::getLine1( size_t index ) const
{
  assert(index < _lines1.size());
  return _lines1[index];
}

opengv::line_t opengv::registration::LineRegistrationAdapter::getLine2( size_t index ) const
{
  assert(index < _lines2.size());
  return _lines2[index];
}

size_t opengv::registration::LineRegistrationAdapter::getNumberCorrespondences() const{
    return _lines1.size();
}
