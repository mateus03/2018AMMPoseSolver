#ifndef OPENGV_REGISTRATION_REGISTRATIONADAPTERBASE_HPP_
#define OPENGV_REGISTRATION_REGISTRATIONADAPTERBASE_HPP_

#include <stdlib.h>
#include <vector>
#include <opengv/types.hpp>

namespace opengv{

    namespace registration{
        
        class RegistrationAdapterBase{

            protected:
                
                opengv::translation_t _t12;
                opengv::rotation_t _R12;

            public:

                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                RegistrationAdapterBase( ) :
                _t12(Eigen::Vector3d::Zero()),
                _R12(Eigen::Matrix3d::Identity()) {};

                RegistrationAdapterBase( const rotation_t & R12 ) :
                _t12(Eigen::Vector3d::Zero()),
                _R12(R12) {};

                RegistrationAdapterBase(const rotation_t & R12, const translation_t & t12 ) :
                _t12(t12),
                _R12(R12) {};

                virtual ~RegistrationAdapterBase() {};

                // get point with index in frame 1
                // virtual opengv::point_t getPoint1( size_t index ) const = 0;

                // // get point with index in frame 2
                // virtual opengv::point_t getPoint2( size_t index ) const = 0;

                virtual size_t getNumberCorrespondences() const = 0;

                opengv::translation_t gett12() const { return _t12; };

                void sett12(const opengv::translation_t & t12) { _t12 = t12; };

                opengv::rotation_t getR12() const { return _R12; };

                void setR12(const opengv::rotation_t & R12) { _R12 = R12; };

        };
    }

}

#endif