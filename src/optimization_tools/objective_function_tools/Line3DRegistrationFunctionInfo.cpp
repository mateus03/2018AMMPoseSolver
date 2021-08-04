#include <opengv/optimization_tools/objective_function_tools/Line3DRegistrationFunctionInfo.hpp>

Line3DRegistrationFunctionInfo::Line3DRegistrationFunctionInfo(const opengv::registration::LineCorrRegistrationAdapter & adapter){

    size_t numLines = adapter.getNumberCorrespondences();
    Mrt = Eigen::Matrix<double,3,9>::Zero();
    Mt = Eigen::Matrix<double,3,3>::Zero();
    Mr =  Eigen::Matrix<double,9,9>::Zero();
    vt =  Eigen::Vector3d::Zero();
    vr =  Eigen::Matrix<double,9,1>::Zero();
    pp = Eigen::Matrix<double,1,1>::Zero();

    Eigen::Matrix3d Lx = Eigen::Matrix3d::Zero();
    opengv::point_t p1A, p1B, p2A; 
    Eigen::Vector3d d2;
    Eigen::Vector3d a, b;
    Eigen::Matrix<double,9,1> vr_i = Eigen::Matrix<double,9,1>::Zero();
    Eigen::Matrix<double,3,9> mrt_i = Eigen::Matrix<double,3,9>::Zero();
    Eigen::Matrix<double,3,9> mr1_i = Eigen::Matrix<double,3,9>::Zero();
    Eigen::Matrix<double,3,9> mr2_i = Eigen::Matrix<double,3,9>::Zero();
    for(size_t iter = 0; iter < numLines; iter++){
        p1A = adapter.getPoint1A(iter);
        p1B = adapter.getPoint1B(iter);
        p2A = adapter.getPoint2A(iter);
        d2 = adapter.getDirection2(iter);
        Lx << 0, -d2(2), d2(1), d2(2), 0, -d2(0), -d2(1), d2(0), 0;

        Eigen::MatrixXd cp = p2A.transpose()*Lx*Lx*p2A;
        pp = pp - cp;
        vt = vt + 2*Lx*Lx*p2A;
        
        b = Lx*Lx*p2A;
        a = p1A + p1B;
        vr_i << a(0)*b(0), a(0)*b(1), a(0)*b(2), a(1)*b(0), a(1)*b(1), a(1)*b(2), a(2)*b(0), a(2)*b(1), a(2)*b(2);
        vr = vr + vr_i;
        
        Eigen::MatrixXd mt_i = Lx*Lx;
        Mt = Mt - mt_i;

        mrt_i = Eigen::Matrix<double,3,9>::Zero();
        mrt_i.block(0,0,3,3) = a(0)*Lx*Lx;
        mrt_i.block(0,3,3,3) = a(1)*Lx*Lx;
        mrt_i.block(0,6,3,3) = a(2)*Lx*Lx;
        Mrt = Mrt - mrt_i;

        mr1_i.block(0,0,3,3) = p1A(0)*Lx;
        mr1_i.block(0,3,3,3) = p1A(1)*Lx;
        mr1_i.block(0,6,3,3) = p1A(2)*Lx;
        mr2_i.block(0,0,3,3) = p1B(0)*Lx;
        mr2_i.block(0,3,3,3) = p1B(1)*Lx;
        mr2_i.block(0,6,3,3) = p1B(2)*Lx;
        Mr = Mr + .5*mr1_i.transpose()*mr1_i + .5*mr2_i.transpose()*mr2_i; 
    }

    // std::cout << pp << std::endl;
    // std::cout << vt.transpose() << std::endl;
    // std::cout << vr.transpose() << std::endl;
    // std::cout << Mt << std::endl;
    // std::cout << Mrt << std::endl;
    // std::cout << Mr << std::endl;
}

Line3DRegistrationFunctionInfo::~Line3DRegistrationFunctionInfo(){}

double Line3DRegistrationFunctionInfo::objective_function_value(const opengv::rotation_t & rotation,
                                          const opengv::translation_t & translation){

    const double * p = &rotation(0);
    Eigen::Map<const Eigen::Matrix<double,1,9> > r(p, 1, 9);

    Eigen::MatrixXd e = (translation.transpose()* Mt * translation)
    + (translation.transpose() * Mrt *  r.transpose())
    + (vt.transpose() * translation)
    + (r * Mr * r.transpose() )
    + (vr.transpose() * r.transpose())
    + pp;
  return ( e(0,0));

}

opengv::rotation_t Line3DRegistrationFunctionInfo::rotation_gradient(const opengv::rotation_t & rotation,
							      const opengv::translation_t & translation){

    const double * p = &rotation(0);
    Eigen::Map<const Eigen::Matrix<double,1,9> > r(p, 1, 9);
    Eigen::MatrixXd result = (2 * Mr * r.transpose()) + ( Mrt.transpose() * translation ) + vr;
    // std::cout << Mr << std::endl;
    double * ptr = &result(0);
    Eigen::Map<Eigen::Matrix<double, 3,3> > m(ptr, 3, 3);
    return m;
}

opengv::translation_t Line3DRegistrationFunctionInfo::translation_gradient(const opengv::rotation_t & rotation,
  const opengv::translation_t & translation){

    const double * p = &rotation(0);
    Eigen::Map<const Eigen::Matrix<double,1,9> > r(p, 1, 9);
    return ( (2*Mt*translation) + (  Mrt * r.transpose() ) +  vt );
}
