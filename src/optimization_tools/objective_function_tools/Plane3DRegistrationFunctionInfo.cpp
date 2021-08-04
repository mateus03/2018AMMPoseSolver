#include <opengv/optimization_tools/objective_function_tools/Plane3DRegistrationFunctionInfo.hpp>

Plane3DRegistrationFunctionInfo::Plane3DRegistrationFunctionInfo(const opengv::registration::PlaneRegistrationAdapter & adapter){
    numPlanes = adapter.getNumberCorrespondences();
    Mr =  Eigen::MatrixXd::Zero(9,9);
    Mt =  Eigen::MatrixXd::Zero(3,3);
    vt_ =  Eigen::VectorXd::Zero(3,1);
    vt =  Eigen::VectorXd::Zero(3,1);
    vr =  Eigen::VectorXd::Zero(9,1);
    pp = Eigen::MatrixXd::Zero(1,1);
    Eigen::Matrix3d id = Eigen::Matrix3d::Identity(3,3);
    cP = 0;

    opengv::plane_t pi1, pi2;
    Eigen::Matrix<double,9,9> Mr_i = Eigen::Matrix<double,9,9>::Zero();
    Eigen::Matrix<double,3,3> mt_i = Eigen::Matrix<double,3,3>::Zero();
    Eigen::Matrix<double,3,9> mr_i = Eigen::Matrix<double,3,9>::Zero();
    Eigen::Matrix<double,9,1> vr_i = Eigen::Matrix<double,9,1>::Zero();
    for(size_t iter = 0; iter < numPlanes; iter++){
        pi1 = adapter.getPlane1(iter);
        pi2 = adapter.getPlane2(iter);
    
        // matrix for squared component on R
        mr_i.block(0,0,3,3) = pi1(0)*id;
        mr_i.block(0,3,3,3) = pi1(1)*id;
        mr_i.block(0,6,3,3) = pi1(2)*id;
        Mr_i = mr_i.transpose()*mr_i;
        Mr = Mr + Mr_i;

        mt_i = pi2.head(3)*pi2.head(3).transpose();
        Mt += mt_i;

        vt = vt + 2*pi2(3)*pi2.head(3);
        vt = vt - 2*pi1(3)*pi2.head(3);

        // vector for linear component on R
        vr_i << pi1(0)*pi2(0), pi1(0)*pi2(1), pi1(0)*pi2(2), pi1(1)*pi2(0), pi1(1)*pi2(1), pi1(1)*pi2(2), pi1(2)*pi2(0), pi1(2)*pi2(1), pi1(2)*pi2(2);
        vr = vr - 2*vr_i;

        pp = pp + pi2.head(3).transpose()*pi2.head(3);
        cP = cP  + pi1(3)*pi1(3) + pi2(3)*pi2(3) - 2*pi1(3)*pi2(3);
    }
}

Plane3DRegistrationFunctionInfo::~Plane3DRegistrationFunctionInfo(){}

double Plane3DRegistrationFunctionInfo::objective_function_value(const opengv::rotation_t & rotation,const opengv::translation_t & translation){
    const double * p = &rotation(0);
    Eigen::Map<const Eigen::Matrix<double,1,9> > r(p, 1, 9);

    // std::cout << r << std::endl;
    // std::cout << rotation << std::endl << std::endl;

    Eigen::MatrixXd c = Eigen::MatrixXd::Zero(1,1);
    c(0,0) = cP;
    Eigen::MatrixXd e = (r * Mr * r.transpose() )
                        + (vr.transpose() * r.transpose())
                        + translation.transpose()*Mt*translation
                        + vt.transpose()*translation
                        + vt_.transpose()*translation
                        + pp
                        + c;
  return ( e(0,0));
}

opengv::rotation_t Plane3DRegistrationFunctionInfo::rotation_gradient(const opengv::rotation_t & rotation,
							      const opengv::translation_t & translation){

    const double * p = &rotation(0);
    Eigen::Map<const Eigen::Matrix<double,1,9> > r(p, 1, 9);
    Eigen::MatrixXd result = (2 * Mr * r.transpose()) + vr;
    double * ptr = &result(0);
    Eigen::Map<Eigen::Matrix<double, 3,3> > m(ptr, 3, 3);
    return m;
}

opengv::translation_t Plane3DRegistrationFunctionInfo::translation_gradient(const opengv::rotation_t & rotation,
  const opengv::translation_t & translation){
    
    const double * p = &rotation(0);
    Eigen::Map<const Eigen::Matrix<double,1,9> > r(p, 1, 9);
    return ( 2*Mt*translation + vt + vt_);
    // return Eigen::Matrix<double,3,1>::Zero();
}