#include <opengv/optimization_tools/objective_function_tools/Point3DRegistrationFunctionInfo.hpp>

Point3DRegistrationFunctionInfo::Point3DRegistrationFunctionInfo(const opengv::registration::PointRegistrationAdapter & adapter){

    numPoints = adapter.getNumberCorrespondences();
    Mrt = Eigen::MatrixXd::Zero(3,9);
    Mr =  Eigen::MatrixXd::Zero(9,9);
    vt =  Eigen::VectorXd::Zero(3,1);
    vr =  Eigen::VectorXd::Zero(9,1);
    pp = Eigen::MatrixXd::Zero(1,1);

    Eigen::Matrix3d id = Eigen::Matrix3d::Identity(3,3);

    opengv::point_t p1, p2;
    Eigen::Matrix<double,9,9> Mr_i = Eigen::Matrix<double,9,9>::Zero();
    Eigen::Matrix<double,3,9> mr_i = Eigen::Matrix<double,3,9>::Zero();
    // Eigen::Matrix<double,3,9> Mrt_i = Eigen::Matrix<double,3,9>::Zero();
    Eigen::Matrix<double,9,1> vr_i = Eigen::Matrix<double,9,1>::Zero();
    // Eigen::Matrix<double,3,1> vt_i = Eigen::Matrix<double,3,1>::Zero();
    for(size_t iter = 0; iter < numPoints; iter++){
        p1 = adapter.getPoint1(iter);
        p2 = adapter.getPoint2(iter);        

        // matrix for squared component on R
        mr_i.block(0,0,3,3) = p1(0)*id;
        mr_i.block(0,3,3,3) = p1(1)*id;
        mr_i.block(0,6,3,3) = p1(2)*id;
        Mr_i = mr_i.transpose()*mr_i;
        Mr = Mr + Mr_i;

        // vector for linear component on t
        vt = vt - 2*p2;

        // vector for linear component on R
        vr_i << p1(0)*p2(0), p1(0)*p2(1), p1(0)*p2(2), p1(1)*p2(0), p1(1)*p2(1), p1(1)*p2(2), p1(2)*p2(0), p1(2)*p2(1), p1(2)*p2(2);
        vr = vr - 2*vr_i;

        // matrix for term with cross dependency on t and R
        Mrt = Mrt + 2*mr_i;

        pp = pp + p2.transpose()*p2;
        // std::cout << p1.transpose() << " " << p2.transpose() << std::endl;
    }
    // std::cout << pp.transpose() << std::endl << std::endl;
}

Point3DRegistrationFunctionInfo::~Point3DRegistrationFunctionInfo(){}

double Point3DRegistrationFunctionInfo::objective_function_value(const opengv::rotation_t & rotation,
                                          const opengv::translation_t & translation){

    const double * p = &rotation(0);
    Eigen::Map<const Eigen::Matrix<double,1,9> > r(p, 1, 9);

    // std::cout << r << std::endl;
    // std::cout << rotation << std::endl << std::endl;

    Eigen::MatrixXd e = numPoints*(translation.transpose() * translation)
    + (translation.transpose() * Mrt *  r.transpose())
    + (vt.transpose() * translation)
    + (r * Mr * r.transpose() )
    + (vr.transpose() * r.transpose())
    + pp;
  return ( e(0,0));

}

opengv::rotation_t Point3DRegistrationFunctionInfo::rotation_gradient(const opengv::rotation_t & rotation,
							      const opengv::translation_t & translation){

    const double * p = &rotation(0);
    Eigen::Map<const Eigen::Matrix<double,1,9> > r(p, 1, 9);
    Eigen::MatrixXd result = (2 * Mr * r.transpose()) + ( Mrt.transpose() * translation ) + vr;
    // std::cout << Mr << std::endl;
    double * ptr = &result(0);
    Eigen::Map<Eigen::Matrix<double, 3,3> > m(ptr, 3, 3);
    return m;
}

opengv::translation_t Point3DRegistrationFunctionInfo::translation_gradient(const opengv::rotation_t & rotation,
  const opengv::translation_t & translation){

    const double * p = &rotation(0);
    Eigen::Map<const Eigen::Matrix<double,1,9> > r(p, 1, 9);
    return ( (2*numPoints*translation) + (  Mrt * r.transpose() ) +  vt );
}
