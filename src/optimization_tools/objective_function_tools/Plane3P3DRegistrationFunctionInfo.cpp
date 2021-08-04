#include <opengv/optimization_tools/objective_function_tools/Plane3P3DRegistrationFunctionInfo.hpp>

Plane3P3DRegistrationFunctionInfo::Plane3P3DRegistrationFunctionInfo(const opengv::registration::PlaneRegistrationAdapter & adapter){
    numPlanes = adapter.getNumberCorrespondences();
    Mr = Eigen::Matrix<double,9,9>::Zero();
    Mt = Eigen::Matrix<double,3,3>::Zero();
    Mrt = Eigen::Matrix<double,3,9>::Zero();
    vt = Eigen::Matrix<double,3,1>::Zero();
    vr = Eigen::Matrix<double,9,1>::Zero();
    pp = Eigen::Matrix<double,1,1>::Zero();
    cP = 0;

    opengv::plane_t pi1, pi2;
    Eigen::Matrix<double,9,9> Mr_i = Eigen::Matrix<double,9,9>::Zero();
    Eigen::Matrix<double,3,9> mrt_i = Eigen::Matrix<double,3,9>::Zero();
    Eigen::Matrix<double,9,1> mr1_i = Eigen::Matrix<double,9,1>::Zero();
    Eigen::Matrix<double,9,1> mr2_i = Eigen::Matrix<double,9,1>::Zero();
    Eigen::Matrix<double,9,1> mr3_i = Eigen::Matrix<double,9,1>::Zero();
    Eigen::Matrix<double,9,1> vr_i = Eigen::Matrix<double,9,1>::Zero();
    Eigen::Matrix<double,3,3> planePoints;
    opengv::point_t pointSum, p1, p2, p3;
    opengv::bearingVector_t dirOff;
    Eigen::Matrix<double,3,3> DD;
    for(size_t iter = 0; iter < numPlanes; iter++){
        pi1 = adapter.getPlane1(iter);
        pi2 = adapter.getPlane2(iter);

        planePoints = Eigen::Matrix<double,3,3>::Zero();
        planePoints.topLeftCorner(2,3) = Eigen::Matrix<double,2,3>::Random();
        planePoints(2,0) = -(pi1(3) + pi1(0)*planePoints(0,0) + pi1(1)*planePoints(1,0))/pi1(2);
        planePoints(2,1) = -(pi1(3) + pi1(0)*planePoints(0,1) + pi1(1)*planePoints(1,1))/pi1(2);
        planePoints(2,2) = -(pi1(3) + pi1(0)*planePoints(0,2) + pi1(1)*planePoints(1,2))/pi1(2);
        pointSum = planePoints.col(0) + planePoints.col(1) + planePoints.col(2); 
        dirOff = pi2(3)*pi2.head(3);
        DD = pi2.head(3)*pi2.head(3).transpose();
        p1 = planePoints.col(0); p2 = planePoints.col(1); p3 = planePoints.col(2);

        Mt += DD;
        
        vt += 2*dirOff;

        cP = cP + pi2(3)*pi2(3);

        vr_i << pointSum(0)*dirOff(0), pointSum(0)*dirOff(1), pointSum(0)*dirOff(2), pointSum(1)*dirOff(0), pointSum(1)*dirOff(1), pointSum(1)*dirOff(2), pointSum(2)*dirOff(0), pointSum(2)*dirOff(1), pointSum(2)*dirOff(2);
        vr += 2.0/3.0*vr_i;

        mrt_i.block(0,0,3,3) = pointSum(0)*DD;
        mrt_i.block(0,3,3,3) = pointSum(1)*DD;
        mrt_i.block(0,6,3,3) = pointSum(2)*DD;
        Mrt = Mrt + 2.0/3.0*mrt_i;

        mr1_i << p1(0)*pi2(0), p1(0)*pi2(1), p1(0)*pi2(2), p1(1)*pi2(0), p1(1)*pi2(1), p1(1)*pi2(2), p1(2)*pi2(0), p1(2)*pi2(1), p1(2)*pi2(2);
        mr2_i << p2(0)*pi2(0), p2(0)*pi2(1), p2(0)*pi2(2), p2(1)*pi2(0), p2(1)*pi2(1), p2(1)*pi2(2), p2(2)*pi2(0), p2(2)*pi2(1), p2(2)*pi2(2);
        mr3_i << p3(0)*pi2(0), p3(0)*pi2(1), p3(0)*pi2(2), p3(1)*pi2(0), p3(1)*pi2(1), p3(1)*pi2(2), p3(2)*pi2(0), p3(2)*pi2(1), p3(2)*pi2(2);
        // Mr_i = ;
        Mr += 1.0/3.0*(mr1_i*mr1_i.transpose() + mr2_i*mr2_i.transpose() + mr3_i*mr3_i.transpose());

        // std::cout << "data:" << std::endl;
        // std::cout << planePoints.transpose() << std::endl;
        // std::cout << pi2.transpose() << std::endl << std::endl;
    }

    // std::cout << "matrices: " << std::endl;
    // std::cout << "cP: " << cP << std::endl;
    // std::cout << "Mt: " <<  Mt << std::endl;
    // std::cout << "vt: " <<  vt.transpose() << std::endl;
    // std::cout << "Mrt: " <<  Mrt << std::endl;
    // std::cout << "vr: " <<  vr.transpose() << std::endl;
    // std::cout << "Mr: " <<  Mr << std::endl;

}

Plane3P3DRegistrationFunctionInfo::~Plane3P3DRegistrationFunctionInfo(){}

double Plane3P3DRegistrationFunctionInfo::objective_function_value(const opengv::rotation_t & rotation,const opengv::translation_t & translation){
    const double * p = &rotation(0);
    Eigen::Map<const Eigen::Matrix<double,1,9> > r(p, 1, 9);

    // std::cout << r << std::endl;
    // std::cout << rotation << std::endl << std::endl;
    Eigen::MatrixXd c = Eigen::MatrixXd::Zero(1,1);
    c(0,0) = cP;
    Eigen::MatrixXd e = (translation.transpose() * Mt * translation)
    + (translation.transpose() * Mrt *  r.transpose())
    + (vt.transpose() * translation)
    + (r * Mr * r.transpose() )
    + (vr.transpose() * r.transpose())
    + c;
  return ( e(0,0));
}

opengv::rotation_t Plane3P3DRegistrationFunctionInfo::rotation_gradient(const opengv::rotation_t & rotation,
							      const opengv::translation_t & translation){

    const double * p = &rotation(0);
    Eigen::Map<const Eigen::Matrix<double,1,9> > r(p, 1, 9);
    Eigen::MatrixXd result = (2 * Mr * r.transpose()) + ( Mrt.transpose() * translation ) + vr;
    // std::cout << Mr << std::endl;
    double * ptr = &result(0);
    Eigen::Map<Eigen::Matrix<double, 3,3> > m(ptr, 3, 3);
    return m;
}

opengv::translation_t Plane3P3DRegistrationFunctionInfo::translation_gradient(const opengv::rotation_t & rotation,
  const opengv::translation_t & translation){

    const double * p = &rotation(0);
    Eigen::Map<const Eigen::Matrix<double,1,9> > r(p, 1, 9);
    return ( (2*Mt*translation) + (  Mrt * r.transpose() ) +  vt );
}
