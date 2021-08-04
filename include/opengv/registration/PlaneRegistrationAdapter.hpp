#ifndef OPENGV_REGISTRATION_PLANEREGISTRATIONADAPTER_HPP_
#define OPENGV_REGISTRATION_PLANEREGISTRATIONADAPTER_HPP_

#include <stdlib.h>
#include <vector>
#include <opengv/types.hpp>
#include <opengv/registration/RegistrationAdapterBase.hpp>

namespace opengv{

    namespace registration{
        
        class PlaneRegistrationAdapter : public RegistrationAdapterBase{

            protected:
                using RegistrationAdapterBase::_t12;
                using RegistrationAdapterBase::_R12;

                const planes_t & _planes1;
                const planes_t & _planes2;

            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                PlaneRegistrationAdapter(const planes_t & planes1, const planes_t & planes2);
                PlaneRegistrationAdapter(const planes_t & planes1, const planes_t & planes2, const rotation_t & R12);
                PlaneRegistrationAdapter(const planes_t & planes1, const planes_t & planes2, const rotation_t & R12, const translation_t & t12);

                virtual ~PlaneRegistrationAdapter();

                plane_t getPlane1( size_t index ) const;
                plane_t getPlane2( size_t index ) const;
                virtual size_t getNumberCorrespondences() const;

        };
    }

}

#endif
