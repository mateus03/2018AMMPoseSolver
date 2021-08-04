#include <opengv/optimization_tools/objective_function_tools/LineIntersection3DRegistrationFunctionInfo.hpp>

LineIntersection3DRegistrationFunctionInfo::
    LineIntersection3DRegistrationFunctionInfo(
        const opengv::registration::LineRegistrationAdapter & adapter){

    size_t numberIntersections = adapter.getNumberCorrespondences();
    M = Eigen::MatrixXd::Zero (numberIntersections, 18);
    Eigen::Matrix<double,6,1> l1,l2;
    for(size_t iter = 0; iter < numberIntersections; iter++){
        l1 = adapter.getLine1(iter);
        l2 = adapter.getLine2(iter);

        //Calculate the kronecker product
        double a00 = l2(0,0) * l1(0,0); double a01 = l2(0,0) * l1(1,0); double a02 = l2(0,0) * l1(2,0);//e1->First column of the essential matrix
        double a03 = l2(0,0) * l1(3,0); double a04 = l2(0,0) * l1(4,0); double a05 = l2(0,0) * l1(5,0);//r1
        double a06 = l2(1,0) * l1(0,0); double a07 = l2(1,0) * l1(1,0); double a08 = l2(1,0) * l1(2,0);//e2
        double a09 = l2(1,0) * l1(3,0); double a10 = l2(1,0) * l1(4,0); double a11 = l2(1,0) * l1(5,0);//r2
        double a12 = l2(2,0) * l1(0,0); double a13 = l2(2,0) * l1(1,0); double a14 = l2(2,0) * l1(2,0);//e3
        double a15 = l2(2,0) * l1(3,0); double a16 = l2(2,0) * l1(4,0); double a17 = l2(2,0) * l1(5,0);//r3
        double a18 = l2(3,0) * l1(0,0); double a19 = l2(3,0) * l1(1,0); double a20 = l2(3,0) * l1(2,0); //r1->First column of the rotation matrix
        double a24 = l2(4,0) * l1(0,0); double a25 = l2(4,0) * l1(1,0); double a26 = l2(4,0) * l1(2,0);//r2
        double a30 = l2(5,0) * l1(0,0); double a31 = l2(5,0) * l1(1,0); double a32 = l2(5,0) * l1(2,0);//r3
        M(iter,0) = a00; M(iter,1) = a01; M(iter,2) = a02; //e1->Column
        M(iter,3) = a06; M(iter,4) = a07; M(iter,5) = a08; //e2
        M(iter,6) = a12; M(iter,7) = a13; M(iter,8) = a14; //e3
        M(iter,9)  = a03 + a18; M(iter,10) = a04 + a19; M(iter,11) = a05 + a20; //r1->Column R
        M(iter,12) = a09 + a24; M(iter,13) = a10 + a25; M(iter,14) = a11 + a26;
        M(iter,15) = a15 + a30; M(iter,16) = a16 + a31; M(iter,17) = a17 + a32;
    }
}

LineIntersection3DRegistrationFunctionInfo::~LineIntersection3DRegistrationFunctionInfo(){}

double LineIntersection3DRegistrationFunctionInfo::objective_function_value(const opengv::rotation_t & rotation,
                                          const opengv::translation_t & translation){

    Eigen::MatrixXd v = Eigen::MatrixXd::Zero(18,1);

    v(0,0)  = translation(1,0) * rotation(2,0) - translation(2,0) * rotation(1,0);
    v(1,0)  = translation(2,0) * rotation(0,0) - translation(0,0) * rotation(2,0);
    v(2,0)  = translation(0,0) * rotation(1,0) - translation(1,0) * rotation(0,0);
    v(3,0)  = translation(1,0) * rotation(2,1) - translation(2,0) * rotation(1,1);
    v(4,0)  = translation(2,0) * rotation(0,1) - translation(0,0) * rotation(2,1);
    v(5,0)  = translation(0,0) * rotation(1,1) - translation(1,0) * rotation(0,1);
    v(6,0)  = translation(1,0) * rotation(2,2) - translation(2,0) * rotation(1,2);
    v(7,0)  = translation(2,0) * rotation(0,2) - translation(0,0) * rotation(2,2);
    v(8,0)  = translation(0,0) * rotation(1,2) - translation(1,0) * rotation(0,2);
    
    v(9,0)  = rotation(0,0);
    v(10,0) = rotation(1,0);
    v(11,0) = rotation(2,0);
    v(12,0) = rotation(0,1);
    v(13,0) = rotation(1,1);
    v(14,0) = rotation(2,1);
    v(15,0) = rotation(0,2);
    v(16,0) = rotation(1,2);
    v(17,0) = rotation(2,2);
    int n_cols = M.cols();
    int n_rows = M.rows();

    double function_value = 0;
    Eigen::MatrixXd ei = Eigen::MatrixXd::Zero(1,1);
    for(int i = 0; i < n_rows; ++i){
        ei = M.block(i,0,1,n_cols) * v;
        function_value = function_value + (ei(0,0) * ei(0,0));
    }
    //std::cout << "Is the true obj. func. called? " << function_value << std::endl;
    return function_value;
}

opengv::rotation_t LineIntersection3DRegistrationFunctionInfo::
    rotation_gradient(const opengv::rotation_t & rotation, const opengv::translation_t & translation){
    
    Eigen::MatrixXd v = Eigen::MatrixXd::Zero(18,1);

    v(0,0)  = translation(1,0) * rotation(2,0) - translation(2,0) * rotation(1,0);
    v(1,0)  = translation(2,0) * rotation(0,0) - translation(0,0) * rotation(2,0);
    v(2,0)  = translation(0,0) * rotation(1,0) - translation(1,0) * rotation(0,0);
    v(3,0)  = translation(1,0) * rotation(2,1) - translation(2,0) * rotation(1,1);
    v(4,0)  = translation(2,0) * rotation(0,1) - translation(0,0) * rotation(2,1);
    v(5,0)  = translation(0,0) * rotation(1,1) - translation(1,0) * rotation(0,1);
    v(6,0)  = translation(1,0) * rotation(2,2) - translation(2,0) * rotation(1,2);
    v(7,0)  = translation(2,0) * rotation(0,2) - translation(0,0) * rotation(2,2);
    v(8,0)  = translation(0,0) * rotation(1,2) - translation(1,0) * rotation(0,2);
    
    v(9,0)  = rotation(0,0);
    v(10,0) = rotation(1,0);
    v(11,0) = rotation(2,0);
    v(12,0) = rotation(0,1);
    v(13,0) = rotation(1,1);
    v(14,0) = rotation(2,1);
    v(15,0) = rotation(0,2);
    v(16,0) = rotation(1,2);
    v(17,0) = rotation(2,2);
    int n_cols = M.cols();
    int n_rows = M.rows();

    double function_value = 0;
    Eigen::MatrixXd ei = Eigen::MatrixXd::Zero(1,1);
    Eigen::Matrix3d grad = Eigen::Matrix3d::Zero(3,3);
    for(int i = 0; i < n_rows; ++i){
        ei = M.block(i,0,1,n_cols) * v;
        grad(0,0) = grad(0,0) + 2 * ( ei(0,0) * (M(i,1) * translation(2,0) - M(i,2) * translation(1,0) + M(i,9))  );
        grad(1,0) = grad(1,0) + 2 * ( ei(0,0) * (M(i,2) * translation(0,0) - M(i,0) * translation(2,0) + M(i,10)) );
        grad(2,0) = grad(2,0) + 2 * ( ei(0,0) * (M(i,0) * translation(1,0) - M(i,1) * translation(0,0) + M(i,11)) );
        
        grad(0,1) = grad(0,1) + 2 * ( ei(0,0) * (M(i,4) * translation(2,0) - M(i,5) * translation(1,0) + M(i,12)) );
        grad(1,1) = grad(1,1) + 2 * ( ei(0,0) * (M(i,5) * translation(0,0) - M(i,3) * translation(2,0) + M(i,13)) );
        grad(2,1) = grad(2,1) + 2 * ( ei(0,0) * (M(i,3) * translation(1,0) - M(i,4) * translation(0,0) + M(i,14)) );
        
        grad(0,2) = grad(0,2) + 2 * ( ei(0,0) * (M(i,7) * translation(2,0) - M(i,8) * translation(1,0) + M(i,15)) );
        grad(1,2) = grad(1,2) + 2 * ( ei(0,0) * (M(i,8) * translation(0,0) - M(i,6) * translation(2,0) + M(i,16)) );
        grad(2,2) = grad(2,2) + 2 * ( ei(0,0) * (M(i,6) * translation(1,0) - M(i,7) * translation(0,0) + M(i,17)) );
    
    }

    return grad;
}

opengv::translation_t LineIntersection3DRegistrationFunctionInfo::
    translation_gradient(const opengv::rotation_t & rotation, const opengv::translation_t & translation){
  
    Eigen::MatrixXd v = Eigen::MatrixXd::Zero(18,1);

    v(0,0)  = translation(1,0) * rotation(2,0) - translation(2,0) * rotation(1,0);
    v(1,0)  = translation(2,0) * rotation(0,0) - translation(0,0) * rotation(2,0);
    v(2,0)  = translation(0,0) * rotation(1,0) - translation(1,0) * rotation(0,0);
    v(3,0)  = translation(1,0) * rotation(2,1) - translation(2,0) * rotation(1,1);
    v(4,0)  = translation(2,0) * rotation(0,1) - translation(0,0) * rotation(2,1);
    v(5,0)  = translation(0,0) * rotation(1,1) - translation(1,0) * rotation(0,1);
    v(6,0)  = translation(1,0) * rotation(2,2) - translation(2,0) * rotation(1,2);
    v(7,0)  = translation(2,0) * rotation(0,2) - translation(0,0) * rotation(2,2);
    v(8,0)  = translation(0,0) * rotation(1,2) - translation(1,0) * rotation(0,2);
    

    v(9,0)  = rotation(0,0);
    v(10,0) = rotation(1,0);
    v(11,0) = rotation(2,0);
    v(12,0) = rotation(0,1);
    v(13,0) = rotation(1,1);
    v(14,0) = rotation(2,1);
    v(15,0) = rotation(0,2);
    v(16,0) = rotation(1,2);
    v(17,0) = rotation(2,2);
    int n_cols = M.cols();
    int n_rows = M.rows();

    Eigen::MatrixXd ei = Eigen::MatrixXd::Zero(1,1);
    opengv::translation_t grad = Eigen::Vector3d::Zero(3,1);
    opengv::translation_t grad_iter = Eigen::Vector3d::Zero(3,1);
    for(int i = 0; i < n_rows; ++i){
        ei = M.block(i,0,1,n_cols) * v;
        grad_iter(0,0) = M(i,2) * rotation(1,0) - M(i,1) * rotation(2,0) +
        M(i,5) * rotation(1,1) - M(i,4) * rotation(2,1) +
        M(i,8) * rotation(1,2) - M(i,7) * rotation(2,2);
        
        grad_iter(1,0) = M(i,0) * rotation(2,0) - M(i,2) * rotation(0,0) +
        M(i,3) * rotation(2,1) - M(i,5) * rotation(0,1) +
        M(i,6) * rotation(2,2) - M(i,8) * rotation(0,2);

        grad_iter(2,0) = M(i,1) * rotation(0,0) - M(i,0) * rotation(1,0) +
        M(i,4) * rotation(0,1) - M(i,3) * rotation(1,1) +
        M(i,7) * rotation(0,2) - M(i,6) * rotation(1,2);

        grad = grad + (2 * ei(0,0) * grad_iter);
    }

    return grad;
}

