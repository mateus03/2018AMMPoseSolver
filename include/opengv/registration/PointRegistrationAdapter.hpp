#ifndef OPENGV_REGISTRATION_POINTREGISTRATIONADAPTER_HPP_
#define OPENGV_REGISTRATION_POINTREGISTRATIONADAPTER_HPP_

#include <stdlib.h>
#include <vector>
#include <opengv/types.hpp>
#include <opengv/registration/RegistrationAdapterBase.hpp>

namespace opengv{

    namespace registration{
        
        class PointRegistrationAdapter : public RegistrationAdapterBase{

            protected:
                using RegistrationAdapterBase::_t12;
                using RegistrationAdapterBase::_R12;

                const points_t & _points1;
                const points_t & _points2;

            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                PointRegistrationAdapter(const points_t & points1, const points_t & points2);
                PointRegistrationAdapter(const points_t & points1, const points_t & points2, const rotation_t & R12);
                PointRegistrationAdapter(const points_t & points1, const points_t & points2, const rotation_t & R12, const translation_t & t12);

                virtual ~PointRegistrationAdapter();

                point_t getPoint1( size_t index ) const;
                point_t getPoint2( size_t index ) const;
                virtual size_t getNumberCorrespondences() const;

        };
    }

}

#endif