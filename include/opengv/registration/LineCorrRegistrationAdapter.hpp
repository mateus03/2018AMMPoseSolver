#ifndef OPENGV_REGISTRATION_LINECORRREGISTRATIONADAPTER_HPP_
#define OPENGV_REGISTRATION_LINECORRREGISTRATIONADAPTER_HPP_

#include <stdlib.h>
#include <vector>
#include <opengv/types.hpp>
#include <opengv/registration/RegistrationAdapterBase.hpp>

namespace opengv{

    namespace registration{
        
        class LineCorrRegistrationAdapter : public RegistrationAdapterBase{

            protected:
                using RegistrationAdapterBase::_t12;
                using RegistrationAdapterBase::_R12;

                const points_t & _points1A, & _points1B;
                const points_t & _points2A;
                const bearingVectors_t _directionVectors2;

            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                LineCorrRegistrationAdapter(const points_t & points1A, const points_t & points1B, const points_t & points2A, const bearingVectors_t & directionVectors2);
                LineCorrRegistrationAdapter(const points_t & points1A, const points_t & points1B, const points_t & points2A, const bearingVectors_t & directionVectors2, const rotation_t & R12);
                LineCorrRegistrationAdapter(const points_t & points1A, const points_t & points1B, const points_t & points2A, const bearingVectors_t & directionVectors2, const rotation_t & R12, const translation_t & t12);

                virtual ~LineCorrRegistrationAdapter();

                point_t getPoint1A( size_t index ) const;
                point_t getPoint1B( size_t index ) const;
                point_t getPoint2A( size_t index ) const;
                bearingVector_t getDirection2( size_t index ) const;
                virtual size_t getNumberCorrespondences() const;

        };
    }

}

#endif
