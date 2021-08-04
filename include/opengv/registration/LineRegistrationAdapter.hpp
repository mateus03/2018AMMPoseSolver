#ifndef OPENGV_REGISTRATION_LINEREGISTRATIONADAPTER_HPP_
#define OPENGV_REGISTRATION_LINEREGISTRATIONADAPTER_HPP_

#include <stdlib.h>
#include <vector>
#include <opengv/types.hpp>
#include <opengv/registration/RegistrationAdapterBase.hpp>

namespace opengv{

    namespace registration{
        
        class LineRegistrationAdapter : public RegistrationAdapterBase{

            protected:
                using RegistrationAdapterBase::_t12;
                using RegistrationAdapterBase::_R12;

                const lines_t & _lines1;
                const lines_t & _lines2;

            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                LineRegistrationAdapter(const lines_t & lines1, const lines_t & lines2);
                LineRegistrationAdapter(const lines_t & lines1, const lines_t & lines2, const rotation_t & R12);
                LineRegistrationAdapter(const lines_t & lines1, const lines_t & lines2, const rotation_t & R12, const translation_t & t12);

                virtual ~LineRegistrationAdapter();

                line_t getLine1( size_t index ) const;
                line_t getLine2( size_t index ) const;
                virtual size_t getNumberCorrespondences() const;

        };
    }

}

#endif
