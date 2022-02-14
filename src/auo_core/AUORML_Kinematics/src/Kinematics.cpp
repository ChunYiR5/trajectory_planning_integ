#include "AUORML/Kinematics/Kinematics.h"

// ================================================================================
//
// Kinematics base class
//
// ================================================================================

Kinematics::~Kinematics() { 

} // In order to fix link error

double Kinematics::fixDecimal(double value) {
    const double multiplier = pow(10.0, PRECISION);
    return ceil(value * multiplier) / multiplier;
}

double Kinematics::InPNpi(double rad){
    if(rad > M_PI) 
        rad -= 2*M_PI; 
    else if(rad < -M_PI) 
        rad += 2*M_PI;         
    else
        rad = rad; 
    return rad;
}

Matrix4d Kinematics::GetRX(double rad){
    Matrix4d RX;
    RX <<   1, 0,         0,          0,
            0, cos(rad),  -sin(rad),  0,
            0, sin(rad),  cos(rad),   0,
            0, 0,         0,          1;
    return RX;
}

Matrix4d Kinematics::GetRY(double rad){
    Matrix4d RY;
    RY <<   cos(rad),   0,  sin(rad), 0,
            0,          1,  0,        0,
            -sin(rad),  0,  cos(rad), 0,
            0,          0,  0,        1;
    return RY;
}

Matrix4d Kinematics::GetRZ(double rad){
    Matrix4d RZ;
    RZ <<   cos(rad), -sin(rad), 0, 0,
            sin(rad), cos(rad),  0, 0,
            0,        0,         1, 0,
            0,        0,         0, 1;
    return RZ;
}

Matrix4d Kinematics::GetTrans(double X, double Y, double Z){
    Matrix4d Trans;
    Trans << 1, 0, 0, X,
             0, 1, 0, Y,
             0, 0, 1, Z,
             0, 0, 0, 1;
    return Trans;
}

Matrix<double, 1, 6> Kinematics::DecompositionXYZYPR(Matrix4d H){

    double px, py, pz, yaw, pitch, roll;
    px = H(0, 3); // translation vector
    py = H(1, 3);
    pz = H(2, 3);

    if( abs(H(2,0)) != 1 ){ // If not encounter Vientiane Lock
        yaw = atan2(H(1, 0), H(0, 0));
        pitch = atan2(-H(2, 0), sqrt(pow(H(2, 1), 2) + pow(H(2, 2), 2)));
        roll = atan2(H(2, 1), H(2, 2) );
    }
	else if ( abs(H(2,0)) == 1 ){  // Encounter Vientiane Lock, and sin(pitch) = 1
		yaw = atan2( -H(1,2) , -H(0,2) );
		pitch   = -(M_PI/2);
		roll  = 0;
	}
	else if ( abs(H(2,0)) == -1 ){  // Encounter Vientiane Lock, and sin(pitch) = 1
		yaw = atan2( H(1,2) , H(0,2) );
		pitch   = (M_PI/2);
		roll  = 0;
	}
    else
        assert(false);

    Matrix<double, 1, 6> result = (Matrix<double, 1, 6>() << px, py, pz, yaw, pitch, roll).finished();
    return result; // PXYZ RZYX
}

Matrix<double, 1, 6> Kinematics::DecompositionXYZRPY(Matrix4d H){

    double px, py, pz, yaw, pitch, roll; 
    px = H(0, 3); // translation vector
    py = H(1, 3);
    pz = H(2, 3);
    
    if( abs(H(0,2)) != 1 ){ // If not encounter Vientiane Lock
        pitch = asin(H(0, 2));
        yaw = atan2(-H(0, 1), H(0, 0));
        roll = atan2(-H(1, 2), H(2, 2));
    }
	else if ( abs(H(0,2)) == 1 ){ // Encounter Vientiane Lock, and sin(pitch) = 1
		yaw = atan2( H(2,1) , H(1,1) );
		pitch   = (M_PI/2);
		roll  = 0;
	}
	else if ( abs(H(0,2)) == -1 ){ // Encounter Vientiane Lock, and sin(pitch) = -1
		yaw = atan2( -H(2,1) , H(1,1) );
		pitch   = -(M_PI/2);
		roll  = 0;
	}
    else
        assert(false);
			
    Matrix<double, 1, 6> result = (Matrix<double, 1, 6>()<< px, py, pz, roll, pitch, yaw ).finished();
    return result;
}

Matrix<double, 1, 7> Kinematics::DecompositionXYZQuaternion(Matrix4d H){

    double px, py, pz, yaw, pitch, roll;
    px = H(0, 3); // translation vector
    py = H(1, 3);
    pz = H(2, 3);

    if ( abs( H(2,0) != 1 ) ){
        yaw = atan2(H(1, 0), H(0, 0));
        pitch = atan2(-H(2, 0), sqrt(pow(H(2, 1), 2) + pow(H(2, 2), 2)));
        roll = atan2(H(2, 1), H(2, 2));
    }
	else if ( abs(H(2,0)) == 1 ){  // Encounter Vientiane Lock, and sin(pitch) = 1
		yaw = atan2( -H(1,2) , -H(0,2) );
		pitch   = -(M_PI/2);
		roll  = 0;
	}
	else if ( abs(H(2,0)) == -1 ){  // Encounter Vientiane Lock, and sin(pitch) = 1
		yaw = atan2( H(1,2) , H(0,2) );
		pitch   = (M_PI/2);
		roll  = 0;
	}

    Matrix<double, 1, 7> result = ( Matrix<double, 1, 7>() << px, py, pz, Euler2Quaternion(yaw, pitch, roll) ).finished();
    return result;
}

Matrix4d Kinematics::GetEndEffectorTransformationMatrix(Eigen::Matrix<double, 1, 6> targetPose){

    Matrix4d EndEffectorfTMatrix = MatrixXd::Zero(4, 4);
    EndEffectorfTMatrix.topLeftCorner(3, 3) = YPRToRotationMatrix(targetPose(3), targetPose(4), targetPose(5));
    EndEffectorfTMatrix(0, 3) = targetPose(0);
    EndEffectorfTMatrix(1, 3) = targetPose(1);
    EndEffectorfTMatrix(2, 3) = targetPose(2);
    EndEffectorfTMatrix(3, 3) = 1;
    return EndEffectorfTMatrix;
}

Matrix<double, 1, 6> Kinematics::GetEndEffectorPose(Eigen::Matrix<double, 1, AXIS> jointAngles) {

    auto TE = GetForwardKinematics(jointAngles);
    auto T0E = TE.block(0, 0 + 4 * (AXIS - 1), 4, 4); // ONLY solve end effector homogenous matrix
    auto poseArray = DecompositionXYZYPR(T0E);

    return poseArray;
}

Eigen::Matrix<double, 1, 6> Kinematics::GetEndEffectorVelocity(Eigen::Matrix<double, 1, AXIS> jointAngles, Eigen::Matrix<double, 1, AXIS> jointVelocity){

    auto robotJacobian = GetRobotJacobian(jointAngles);
    auto endEffectorVelocity = robotJacobian * jointVelocity.transpose();

    return endEffectorVelocity;
}

Eigen::Matrix<double, 1, 6> Kinematics::GetEndEffectorAcceleration(Eigen::Matrix<double, 1, AXIS> jointAngles, Eigen::Matrix<double, 1, AXIS> jointVelocity, Eigen::Matrix<double, 1, AXIS> jointAcceleration){

    auto robotJacobian = GetRobotJacobian(jointAngles);
    auto robotJacobianDot = GetRobotJacobianDot(jointAngles, jointVelocity);
    auto endEffectorAcceleration = robotJacobianDot * jointVelocity.transpose() + robotJacobian * jointAcceleration.transpose();

    return endEffectorAcceleration;
}

// ================================================================================
//
// IDH Kinematics
//
// ================================================================================

Matrix<double, 4, AXIS*4 > IDHKinematics::GetForwardKinematics(Matrix<double, 1, AXIS> jointAngles) {

    Matrix4d T0E = MatrixXd::Identity(4, 4);
    Matrix<double, 4, AXIS*4> TE = MatrixXd::Zero(4, AXIS*4);
    Matrix4d iJ;
    double iTheta, iD, iA, iAlpha, iBeta;

    for (int i = 0; i < AXIS; i++){
        iTheta = IDHTable(i, 0) + jointAngles(0, i);
        iD = IDHTable(i, 1);
        iA = IDHTable(i, 2);
        iAlpha = IDHTable(i, 3);
        iBeta = IDHTable(i, 4);

        iJ = GetIndividualTransMatrix(iTheta, iD, iA, iAlpha, iBeta);
        T0E = T0E * iJ;
        TE.block<4, 4>(0, 0 + i * 4) = T0E;
    }

    return TE;
}

Matrix4d IDHKinematics::GetIndividualTransMatrix(double theta, double d, double a, double alpha, double beta){
    Matrix4d J;
    J = GetRZ(theta) * GetTrans(0, 0, d) * GetTrans(a, 0, 0) * GetRX(alpha) * GetRY(beta);
    return J;
}

Vector2d IDHKinematics::SolveIKJ1(double wristX, double wristY){
    
    Vector2d J1;
    double j1, j1p;

    if (wristX == 0 && wristY == 0){
        //printf("Reach Shoulder singular !!");
        J1 << NAN , NAN;
    }
    else{
        j1 = atan2(wristY, wristX);
        j1p = atan2(wristY, wristX) + M_PI;
        j1 = InPNpi(j1);
        j1p = InPNpi(j1p);
        J1 << j1 , j1p;
    }

    return J1;
}

Vector4d IDHKinematics::SolveIKJ23(double j1, double wristX, double wristY, double wristZ){
        
    Vector4d J23;
    Vector2d J2, J3;
    double AB = IDHTable(0,2); //a1
    double P = 0;

    // determine whether if coordinate p is opposite. P is the coordinate of wrist center w.r.t joint1 frame.
    if( abs(j1) < (M_PI/2) ){
        P = wristX < 0 ? (-sqrt(wristX * wristX + wristY * wristY) - AB) : (sqrt(wristX * wristX + wristY * wristY) - AB);
    }
    else if( abs(j1) > (M_PI/2) ){
        P = wristX > 0 ? (-sqrt(wristX * wristX + wristY * wristY) - AB) : (sqrt(wristX * wristX + wristY * wristY) - AB);
    }
    else if(j1 == M_PI/2){
        P = wristX < 0 ? (-sqrt(wristX * wristX + wristY * wristY) - AB) : (sqrt(wristX * wristX + wristY * wristY) - AB);
    }
    else if(j1 == -M_PI/2){
        P = wristX > 0 ? (-sqrt(wristX * wristX + wristY * wristY) - AB) : (sqrt(wristX * wristX + wristY * wristY) - AB);
    }
    else{
        P = sqrt(wristX * wristX + wristY * wristY) - AB;
    }

    double OA = IDHTable(0,1);                  // d1
    double Q  = wristZ - OA;
    double BC = IDHTable(1, 2);                 // a2 . r1
    double CD = IDHTable(2, 2);                 // a3
    double DE = IDHTable(3, 1);                 // d4
    double BE = sqrt(P * P + Q * Q);      // p^2 + q^2
    double CE = sqrt(CD * CD + DE * DE);  // r2

    /* ------------- check singularity -------------- */
    
    if( fixDecimal( abs((BC*BC+BE*BE-CE*CE)/(2*BC*BE)) ) > 1 ){
        //printf("Robot EEF out of reachable worksapce while check J2...!");
        J2 << NAN, NAN;
    }
    else{
        /* construct the geometry triangle angle for j2 */
        double angleCBE = acos( fixDecimal((BC * BC + BE * BE - CE * CE) / (2 * BC * BE)) );
        double angleEBU = atan2(Q, P);

        double j2 = -(M_PI / 2 - (angleCBE + angleEBU));
        double j2p = -(M_PI / 2 - (angleEBU - angleCBE));
        j2 = InPNpi(j2);
        j2p = InPNpi(j2p);
        J2 << j2 , j2p;
    }

    /* construct the geometry triangle angle for j2 */
    if( fixDecimal( abs((BC*BC+CE*CE-BE*BE)/(2*BC*CE)) ) == 1  ){
        //printf("Joint singular in J3, But not raise False flag while solve inverse IDHKinematics...!");
    }

    if ( fixDecimal( abs((BC*BC+CE*CE-BE*BE)/ (2*BC*CE)) ) > 1 ){
        //printf("Robot EEF out of reachable worksapce while check J3...!");
        J3 << NAN, NAN;
    }
    else{
        double angleBCE = acos( fixDecimal((BC * BC + CE * CE - BE * BE) / (2 * BC * CE)) );
        double angleECD = acos( fixDecimal(CD/CE) );   //acosd((CD ^ 2 + CE ^ 2 - DE ^ 2) / (2 * CD * CE));

        /* 這邊的j3有考慮到軸向問題 &DH參數第三軸位置 */
        double j3 = (angleBCE + angleECD - M_PI);     //elbow up
        double j3p = (M_PI - (angleBCE - angleECD));  //elbow down

        j3 = InPNpi(j3);
        j3p = InPNpi(j3p);
        J3 << j3 , j3p;
    }

    J23 << J2, J3;
    
    return J23; //[j1 j1p j2 j2p]
}

Matrix<double, 6, 1> IDHKinematics::SolveIKJ456(double j1, double j2, double j3, double px, double py, double pz, double nx, double ny, double nz, double ox, double oy, double oz, double ax, double ay, double az){
    
    Matrix<double, 6, 1> J456;

    /* ----------------- j4 ------------------- */
    Vector4d x_eef, y_eef, z_eef;
    x_eef << nx , ny , nz , 1;
    y_eef << ox , oy , oz , 1;
    z_eef << ax , ay , az , 1;

    /* ----- since DH theta2  + = 90 ----- */
    double j2_mech = j2 - M_PI / 2;

    Vector4d x_wrist, y_wrist, z_wrist;

    x_wrist = Kinematics::GetRY(j2_mech + j3) * Kinematics::GetRZ(-j1) * x_eef;
    y_wrist = Kinematics::GetRY(j2_mech + j3) * Kinematics::GetRZ(-j1) * y_eef;
    z_wrist = Kinematics::GetRY(j2_mech + j3) * Kinematics::GetRZ(-j1) * z_eef;

    Vector2d J4, J5, J6;
    double j4, j4p, j5, j5p, j6, j6p;

    if( ( fixDecimal(abs(z_wrist(1))) == 0 && fixDecimal((abs(z_wrist(0))) == 0 ) ) || isnan(j1) || isnan(j2) || isnan(j3) ){
        //printf("Reach Wrist singular in J4!!");
        j4 = NAN;
        j4p = NAN;
        J4 << j4, j4p;
    }
    else{
        j4  = fixDecimal( atan2(z_wrist(1), z_wrist(0)) );
        j4p = fixDecimal( atan2(z_wrist(1), z_wrist(0)) + M_PI );
        
        /* 增加一個機制 若超過180度 自動變負號 */
        j4 = InPNpi(j4);
        j4p = InPNpi(j4p);
        J4 << j4, j4p;
    }

    /* ----------------- j5 ------------------- */
    Vector4d temp , ET;
    temp << 1, 0, 0, 1;
    ET = GetRZ(j4) * temp;

    double num1 = z_wrist(2);
    double num2 = ET(0) * z_wrist(0) + ET(1) * z_wrist(1) + ET(2) * z_wrist(2);

    /* ---- check singular ---- */
    
    if ( (fixDecimal(abs(num2) == 0 )) || isnan(j1) || isnan(j2) || isnan(j3) ){
        //printf("Reach Wrist singular in J5!!");
        j5 = NAN;
        j5p = NAN;
        J5 << j5, j5p;
    }
            
    else{
        // 如果有算術誤差，將其消除
        double J5temp1 = acos(fixDecimal(num1));
        double J5temp2 = acos(fixDecimal(num2));

        if(J5temp2 > M_PI/2){
            j5 = -1*J5temp1;
        }
        else {
            j5 = J5temp1;
        }

        // 軸向定義
        j5 *= -1;
        j5p = -j5;

        // 增加一個機制 若超過180度 自動變負號
        j5 = InPNpi(j5);
        j5p = InPNpi(j5p);

        J5 << j5,j5p;
    }

    /* ----------------- j6 ------------------- */

    Matrix4d T1, T2, T3, T4, T5, A5;
    T1 = GetRZ(j1 + IDHTable(0, 0)) * GetTrans(0, 0, IDHTable(0, 1)) * GetTrans(IDHTable(0, 2), 0, 0) * GetRX(IDHTable(0, 3));
    T2 = GetRZ(j2 + IDHTable(1, 0)) * GetTrans(0, 0, IDHTable(1, 1)) * GetTrans(IDHTable(1, 2), 0, 0) * GetRX(IDHTable(1, 3));
    T3 = GetRZ(j3 + IDHTable(2, 0)) * GetTrans(0, 0, IDHTable(2, 1)) * GetTrans(IDHTable(2, 2), 0, 0) * GetRX(IDHTable(2, 3));
    T4 = GetRZ(j4 + IDHTable(3, 0)) * GetTrans(0, 0, IDHTable(3, 1)) * GetTrans(IDHTable(3, 2), 0, 0) * GetRX(IDHTable(3, 3));
    T5 = GetRZ(j5 + IDHTable(4, 0)) * GetTrans(0, 0, IDHTable(4, 1)) * GetTrans(IDHTable(4, 2), 0, 0) * GetRX(IDHTable(4, 3));

    A5 = T1 * T2 * T3 * T4 * T5;
    double J6temp1cos = nx * A5(0, 0) + ny * A5(1, 0) + nz * A5(2, 0);
    double J6temp2cos = nx * A5(0, 1) + ny * A5(1, 1) + nz * A5(2, 1);

    /* ---------- check wrist singular -------- */
    
    if( isnan(j5) || isnan(j1) || isnan(j2) || isnan(j3) ){
        //printf("Reach Wrist singular in J6!!");
        j6 = NAN;
        j6p = NAN;
        J6 << j6, j6p;
    }
    else{
        // 如果有算術誤差，將其消除
        double J6temp1 = acos(fixDecimal(J6temp1cos));
        double J6temp2 = acos(fixDecimal(J6temp2cos));

        if (J6temp2 > M_PI/2){
            j6 = -1 * J6temp1;
        }
        else{
            j6 = J6temp1;
        }

        j6p = j6 + M_PI;

        // 增加一個機制 若超過180度 自動變負號
        j6 = InPNpi(j6);
        j6p = InPNpi(j6p);
        J6 << j6, j6p;
    }

    J456 << J4, J5, J6;

    return J456;
}

Matrix<double, 8, 6> IDHKinematics::SolveIndustrialIK( Eigen::Matrix<double, 1, 6> targetPose){
    /*
    *   J1 can't solve while enter shoulder singular
    *   J2、J3 can be solve while singular but not solvetable while wrist center out of length with L1 + L2
    *   J4、J5、J6
    */
    
    double px = fixDecimal(targetPose(0));
    double py = fixDecimal(targetPose(1));
    double pz = fixDecimal(targetPose(2));

    auto rotationMatrix = YPRToRotationMatrix(targetPose(3), targetPose(4), targetPose(5));
    double nx = fixDecimal(rotationMatrix(0, 0));
    double ny = fixDecimal(rotationMatrix(1, 0));
    double nz = fixDecimal(rotationMatrix(2, 0));
    double ox = fixDecimal(rotationMatrix(0, 1));
    double oy = fixDecimal(rotationMatrix(1, 1));
    double oz = fixDecimal(rotationMatrix(2, 1));
    double ax = fixDecimal(rotationMatrix(0, 2));
    double ay = fixDecimal(rotationMatrix(1, 2));
    double az = fixDecimal(rotationMatrix(2, 2));

    double wristX = px - IDHTable(5, 1) * ax;
    double wristY = py - IDHTable(5, 1) * ay;
    double wristZ = pz - IDHTable(5, 1) * az;

    Vector2d J1 = SolveIKJ1(wristX, wristY);
    Vector4d J23_01 = SolveIKJ23(J1(0), wristX, wristY, wristZ); // Output: [J2_01(0) J2_01(1) J3_01(0) J3_01(1)].T
    Vector4d J23_23 = SolveIKJ23(J1(1), wristX, wristY, wristZ); // Output: [J2_23(0) J2_23(1) J3_23(0) J3_23(1)].T

    Matrix<double, 6, 1> J456_01 = SolveIKJ456(J1(0), J23_01(0), J23_01(2), px, py, pz, nx, ny, nz, ox, oy, oz, ax, ay, az); // [J4_01(0) J4_01(1) J5_01(0) J5_01(1) J6_01(0) J6_01(1)]
    Matrix<double, 6, 1> J456_23 = SolveIKJ456(J1(0), J23_01(1), J23_01(3), px, py, pz, nx, ny, nz, ox, oy, oz, ax, ay, az); // [J4_23(0) J4_23(1) J5_23(0) J5_23(1) J6_23(0) J6_23(1)]
    Matrix<double, 6, 1> J456_45 = SolveIKJ456(J1(1), J23_23(0), J23_23(2), px, py, pz, nx, ny, nz, ox, oy, oz, ax, ay, az); // [J4_45(0) J4_45(1) J5_45(0) J5_45(1) J6_45(0) J6_45(1)]
    Matrix<double, 6, 1> J456_67 = SolveIKJ456(J1(1), J23_23(1), J23_23(3), px, py, pz, nx, ny, nz, ox, oy, oz, ax, ay, az); // [J4_67(0) J4_67(1) J5_67(0) J5_67(1) J6_67(0) J6_67(1)]


    Matrix<double, 8, 1> j1, j2, j3, j4, j5, j6;

    j1 << J1(0), J1(0), J1(0), J1(0), J1(1), J1(1), J1(1), J1(1) ;
    j2 << J23_01(0), J23_01(0), J23_01(1), J23_01(1), J23_23(0), J23_23(0), J23_23(1), J23_23(1) ;
    j3 << J23_01(2), J23_01(2), J23_01(3), J23_01(3), J23_23(2), J23_23(2), J23_23(3), J23_23(3) ;
    j4 << J456_01(0), J456_01(1), J456_23(0), J456_23(1), J456_45(0), J456_45(1), J456_67(0), J456_67(1) ;
    j5 << J456_01(2), J456_01(3), J456_23(2), J456_23(3), J456_45(2), J456_45(3), J456_67(2), J456_67(3) ;
    j6 << J456_01(4), J456_01(5), J456_23(4), J456_23(5), J456_45(4), J456_45(5), J456_67(4), J456_67(5) ;

    Matrix<double, 8, 6> theata = (Matrix<double, 8, 6>() << j1, j2, j3, j4, j5, j6).finished();
    return theata;
}

Vector2d IDHKinematics::SolveURIKJ1(double px, double py, double ax, double ay){

    double d4 = IDHTable(3,1);
    double d6 = IDHTable(5,1);
    
    double m = d6 * ay - py;
    double n = ax * d6 - px;
    
    Vector2d J1;
    // -------------- check shoulder singular --------------
    if( ( m*m+n*n-d4*d4 ) < 0 ){
        //printf("Reach Shoulder singular !!\n");
        J1 << NAN, NAN;
    }
    else{
        double j1 = fixDecimal( atan2(m,n) - atan2( -d4, ( sqrt(m*m+n*n-d4*d4) ) ) );
        double j1_flip = fixDecimal( atan2(m,n) - atan2( -d4, -1 * ( sqrt(m*m+n*n-d4*d4) ) ) );    
        j1 = InPNpi(j1);
        j1_flip = InPNpi(j1_flip);
        J1 << j1, j1_flip;
    }

    return J1;
}

Vector2d IDHKinematics::SolveURIKJ5(double j1, double ax, double ay){

    double num = -1 * sin(j1) * ax + cos(j1) * ay;
    Vector2d J5;

    if (isnan(j1))
        J5 << NAN, NAN;
    else{
        double j5 = acos(fixDecimal(num));
        double j5_p = -1 * acos(fixDecimal(num));
        j5 = InPNpi(j5);
        j5_p = InPNpi(j5_p);
        J5 << j5, j5_p;
    }
     
    return J5;
}

double IDHKinematics::SolveURIKJ6(double j1, double j5, double nx, double ny, double ox, double oy){

    double J6 = 0;

    // -------------- check shoulder singular --------------     
    if ( isnan(j1) || isnan(j5) ){
        J6 = NAN;
    }
    // -------------- check wrist singular --------------   
    else if( abs(sin(j5)) < 10e-6 ){
        printf("Reach wrist singular !!\n");
        J6 = NAN;
    }
    else{
        double m1 = -1 * sin(j1) * nx + cos(j1) * ny;
        double n1 = -1 * sin(j1) * ox + cos(j1) * oy;
        
        double num = -1 * n1 / sin(j5);
        double dom = m1 / sin(j5);
        
        double j6 = fixDecimal(atan2( num , dom ));
        j6 = InPNpi(j6);
        J6 = j6;
    }
               
    return J6;
}

Matrix<double, 6, 1> IDHKinematics::SolveURIKJ234(double j1, double j5, double j6, double px, double py, double pz, double nx, double ny, double nz, double ox, double oy, double oz, double ax, double ay, double az){

    // --------------------- cal j3 ------------------------ 
    /*
        x =  -d5*(-(nx*cos(j1) + ny*sin(j1))*sin(j6) - (ox*cos(j1) + oy*sin(j1))*cos(j6)) - d6*(ax*cos(j1) + ay*sin(j1)) + px*cos(j1) + py*sin(j1)
        y = -az*d6 - d1 - d5*(-nz*sin(j6) - oz*cos(j6)) + pz
    */

    double d1 = IDHTable(0,1);
    double d5 = IDHTable(4,1);
    double d6 = IDHTable(5,1);
    double a2 = IDHTable(1,2);
    double a3 = IDHTable(2,2);
    
    double x =  -d5*(-(nx*cos(j1) + ny*sin(j1))*sin(j6) - (ox*cos(j1) + oy*sin(j1))*cos(j6)) - d6*(ax*cos(j1) + ay*sin(j1)) + px*cos(j1) + py*sin(j1);
    double y = -az*d6 - d1 - d5*(-nz*sin(j6) - oz*cos(j6)) + pz;
    
    double num = pow(x,2) + pow(y,2) - pow(a2,2) - pow(a3,2);
    double dom = 2 * a2 * a3;
    
    Vector2d J2, J3, J4;
    // --------------------- cal j2 ------------------------ 
    // ---------- check singular ----------- 
    
    if( (num/dom) > 1 ){
        //printf("Reach elbow singular !!\n");
        J3 << NAN, NAN;
    }
    else if( isnan(j1) || isnan(j6) ){
        J3 << NAN, NAN;
    }
    else{
        double j3 = acos(fixDecimal(num/dom));
        double j3_flip = -acos(fixDecimal(num/dom)); 
        
        j3 = InPNpi(j3);
        j3_flip = InPNpi(j3_flip);

        J3 << j3, j3_flip;
    }
        
    // --------------------- cal j2 ------------------------ 
    
    double j2 = SolveURIKJ2(J3(0),a2,a3,x,y);
    double j2_flip = SolveURIKJ2(J3(1),a2,a3,x,y);

    J2 << j2, j2_flip;
    
    // --------------------- cal j4 ------------------------ #
    
    double j4 = SolveURIKJ4(j1,J2(0),J3(0),j5,j6,nx,ny,nz,ox,oy,oz,ax,ay,az);
    double j4_flip = SolveURIKJ4(j1,J2(1),J3(1),j5,j6,nx,ny,nz,ox,oy,oz,ax,ay,az);
    
    J4 << j4, j4_flip;
    
    Matrix<double, 6, 1> J234;
    J234 << J2, J3, J4;

    return J234;
}

double IDHKinematics::SolveURIKJ2(double j3, double a2, double a3, double x, double y){

    double J2 = 0;

    if (isnan(j3)){
        J2 = NAN;
    }   
    else{
        double det = pow(a3,2) * pow(cos(j3),2) + 2 * a2 * a3 * cos(j3) + pow(a2,2) + pow(a3,2) * pow(sin(j3),2);   
                
        double num = ( -a3 * sin(j3) ) * x + ( a3 * cos(j3) + a2 ) * (-y);
        double dom = ( a3 * cos(j3) + a2 ) * x + ( a3 * sin(j3) ) * (-y);
        
        double j2 = atan2(num,dom) + M_PI/2; // DH theta2 = -90
        j2 = InPNpi(fixDecimal(j2));
        
        J2 = j2;
    }
       
    return J2;
}

double IDHKinematics::SolveURIKJ4(double j1, double j2, double j3, double j5, double j6, double nx, double ny, double nz, double ox, double oy, double oz, double ax, double ay, double az){

    double J4 = 0;

    if ( isnan(j1) || isnan(j2) || isnan(j3) || isnan(j5) || isnan(j6) ){
        J4 = NAN;
    }
    else{
        double T14_12 = (nx*cos(j1) + ny*sin(j1))*sin(j6) + (ox*cos(j1) + oy*sin(j1))*cos(j6);
        double T14_11 = -(ax*cos(j1) + ay*sin(j1))*sin(j5) + ((nx*cos(j1) + ny*sin(j1))*cos(j6) - (ox*cos(j1) + oy*sin(j1))*sin(j6))*cos(j5);
    
        double j4 = atan2(-T14_12,T14_11) - j2 - j3; // DH theta4 = 90 , but theta2 = -90 => 0
        j4 = InPNpi(fixDecimal(j4));
        J4 = j4;
    }
       
    return J4; 
}


Matrix<double, 8, 6> IDHKinematics::SolveURIK( Eigen::Matrix<double, 1, 6> targetPose){

    /*
    *   Solve sequential: J1 -> J5 -> J6 -> J3 -> J2 -> J4
    */
    
    double px = fixDecimal(targetPose(0));
    double py = fixDecimal(targetPose(1));
    double pz = fixDecimal(targetPose(2));

    auto rotationMatrix = YPRToRotationMatrix(targetPose(3), targetPose(4), targetPose(5));
    double nx = fixDecimal(rotationMatrix(0, 0));
    double ny = fixDecimal(rotationMatrix(1, 0));
    double nz = fixDecimal(rotationMatrix(2, 0));
    double ox = fixDecimal(rotationMatrix(0, 1));
    double oy = fixDecimal(rotationMatrix(1, 1));
    double oz = fixDecimal(rotationMatrix(2, 1));
    double ax = fixDecimal(rotationMatrix(0, 2));
    double ay = fixDecimal(rotationMatrix(1, 2));
    double az = fixDecimal(rotationMatrix(2, 2));


    // solve joint 1
    auto J1 = SolveURIKJ1(px, py, ax, ay); // 2 solution
    
    // solve joint 5
    auto J501 = SolveURIKJ5(J1(0), ax, ay);
    auto J523 = SolveURIKJ5(J1(1), ax, ay); // 4 solution
    
    // solve joint 6
    auto J60 = SolveURIKJ6(J1(0), J501(0), nx, ny, ox, oy);
    auto J61 = SolveURIKJ6(J1(0), J501(1), nx, ny, ox, oy);
    auto J62 = SolveURIKJ6(J1(1), J523(0), nx, ny, ox, oy);
    auto J63 = SolveURIKJ6(J1(1), J523(1), nx, ny, ox, oy);
    
    // solve joint 234
    auto J234_01 = SolveURIKJ234(J1(0), J501(0), J60, px, py, pz, nx, ny, nz, ox, oy, oz, ax, ay, az);
    auto J234_23 = SolveURIKJ234(J1(0), J501(1), J61, px, py, pz, nx, ny, nz, ox, oy, oz, ax, ay, az);
    auto J234_45 = SolveURIKJ234(J1(1), J523(0), J62, px, py, pz, nx, ny, nz, ox, oy, oz, ax, ay, az);
    auto J234_67 = SolveURIKJ234(J1(1), J523(1), J63, px, py, pz, nx, ny, nz, ox, oy, oz, ax, ay, az); // 8 solution

    Matrix<double, 8, 1> j1, j2, j3, j4, j5, j6;
    j1 << J1(0), J1(0), J1(0), J1(0), J1(1), J1(1), J1(1), J1(1) ;
    j2 << J234_01(0), J234_01(1), J234_23(0), J234_23(1), J234_45(0), J234_45(1), J234_67(0), J234_67(1) ;
    j3 << J234_01(2), J234_01(3), J234_23(2), J234_23(3), J234_45(2), J234_45(3), J234_67(2), J234_67(3) ;
    j4 << J234_01(4), J234_01(5), J234_23(4), J234_23(5), J234_45(4), J234_45(5), J234_67(4), J234_67(5) ;
    j5 << J501(0), J501(0), J501(1), J501(1), J523(0), J523(0), J523(1), J523(1) ;
    j6 << J60, J60, J61, J61, J62, J62, J63, J63 ;

    Matrix<double, 8, 6> theata = (Matrix<double, 8, 6>() << j1, j2, j3, j4, j5, j6).finished();
    return theata;

}

bool IDHKinematics::GetInverseKinematics(Eigen::Matrix<double, 1, 6> targetPose, Eigen::Matrix<double, 1, AXIS> referenceJoint, Eigen::Ref<Matrix<double, 1, AXIS>> solution) {

    // call IK solver to get 8 different solution
    Matrix<double, 8, 6> solutionSet;

    if( this -> robotType == "Industrial" ){
        solutionSet = SolveIndustrialIK(targetPose);
    }
    else if( this -> robotType == "UR" ){
        solutionSet = SolveURIK(targetPose);
    }
    else{
        printf("Kinematics solver no usable robot type ! Please use ChangeRobotType() to define correct type of kinematics solver.");
        return false;
    }

    Matrix<double, 1, 6> bestFitJoint;
    // check if there exist legal solution
    bestFitJoint = FindMinDistanceJoint(solutionSet, referenceJoint);
    double array_sum = bestFitJoint.sum();

    if(isnan(array_sum)){
        for(int ijoint = 0; ijoint < AXIS; ijoint++)
            solution(ijoint) = 	NAN;
		return false;
    }
    else {
        for(int ijoint = 0; ijoint < AXIS; ijoint++)
            solution(ijoint) = 	bestFitJoint(ijoint);
        return true;
    }	
}

bool IDHKinematics::GetInverseKinematicsWithConstraint(Eigen::Matrix<double, 1, 6> targetPose, Eigen::Matrix<double, 1, AXIS> referenceJoint, Eigen::Matrix<double, 1, AXIS> posMaxBound, Eigen::Matrix<double, 1, AXIS> posMinBound, Eigen::Ref<Matrix<double, 1, AXIS>> solution) {

    // call IK solver to get 8 different solution

    Matrix<double, 8, 6> solutionSet;

    if( this -> robotType == "Industrial" ){
        solutionSet = SolveIndustrialIK(targetPose);
    }
    else if( this -> robotType == "UR" ){
        solutionSet = SolveURIK(targetPose);
    }
    else{
        printf("Kinematics solver no usable robot type ! Please use ChangeRobotType() to define correct type of kinematics solver.");
        return false;
    }

    // judge if solution exceed bound

    std::vector<Matrix<double, 1, AXIS>>  legalSolution;

    for (int ikSolNum = 0; ikSolNum < solutionSet.rows() ; ikSolNum++){

        Matrix<double, 1, 6> theta = solutionSet.row(ikSolNum);

        double array_sum = theta.sum();
        if(isnan(array_sum)){
            continue;
        }

        // IF NOT nan value, judge if exceed constraint

        for(int ijoint = 0; ijoint < AXIS; ijoint ++){
            if(theta[ijoint] > posMaxBound[ijoint] || theta[ijoint] < posMinBound[ijoint] )
                continue;
        }

        // If pass all test, push back to legalSolution

        legalSolution.push_back(theta);

    }

    // Find minimal distance solution

    bool solutionFindFlag = false;
    Matrix<double, 1, 6> _theta;
    Matrix<double, 1, 6> _solution;
    double d_min = 0;

    for (int ikSolNum = 0; ikSolNum < legalSolution.size() ; ikSolNum++){
        _theta = legalSolution[ikSolNum];

        /* ============= Judge Minimum Distance =============== */
        /* if has solution, then compute minimal distance:      */
        double distance = 0;

        for(int ijoint= 0; ijoint < AXIS; ijoint++){
            double absValue = abs(_theta(ijoint) - referenceJoint(ijoint));
            // deal with 180 -> -180
            distance += absValue > M_PI ? 2 * M_PI - absValue : absValue;
            /*
            if(absValue > M_PI ){
                distance += 2 * M_PI - absValue;
            }
            else{
                distance += absValue;
            }
            */
        }
        
        if(solutionFindFlag == false){
            _solution = _theta;
            d_min = distance;
            solutionFindFlag = true;
        }
        else{
            if(d_min > distance){
                _solution = _theta;
                d_min = distance;
            }
        }
    }

    if (solutionFindFlag == false){
        _solution.fill(NAN);
    }


    // Return final result

    double array_sum = solution.sum();

    if(!solutionFindFlag){
        for(int ijoint = 0; ijoint < AXIS; ijoint++)
            solution(ijoint) = 	NAN;
		return false;
    }
    else {
        for(int ijoint = 0; ijoint < AXIS; ijoint++)
            solution(ijoint) = 	_solution(ijoint);
        return true;
    }	
}

Matrix<double, 1, 6> IDHKinematics::FindMinDistanceJoint(Matrix<double, 8, 6> JointSet , Matrix<double, 6, 1> BenchmarkJoint){

    bool solutionFindFlag = false;
    Matrix<double, 1, 6> theta;
    Matrix<double, 1, 6> solution;
    double d_min = 0;

    for (int ikSolNum = 0; ikSolNum < JointSet.rows() ; ikSolNum++){
        theta = JointSet.row(ikSolNum);

        // First check if their is legicy solution:
        double array_sum = theta.sum();
        if(isnan(array_sum)){
            continue;
        }

        /* ============= Judge Minimum Distance =============== */
        /* if has solution, then compute minimal distance:      */
        double distance = 0;

        for(int ijoint= 0; ijoint < AXIS; ijoint++){
            double absValue = abs(theta(ijoint) - BenchmarkJoint(ijoint));
            // deal with 180 -> -180
            distance += absValue > M_PI ? 2 * M_PI - absValue : absValue;
            /*
            if(absValue > M_PI ){
                distance += 2 * M_PI - absValue;
            }
            else{
                distance += absValue;
            }
            */
        }
        
        if(solutionFindFlag == false){
            solution = theta;
            d_min = distance;
            solutionFindFlag = true;
        }
        else{
            if(d_min > distance){
                solution = theta;
                d_min = distance;
            }
        }
    }

    if (solutionFindFlag == false){
        solution.fill(NAN);
    }

    return solution;
}

Matrix<double, 6, AXIS> IDHKinematics::GetRobotJacobian(Matrix<double, 1, AXIS> jointAngles) const{

    double a2 = IDHTable(1,2);
    double d4 = IDHTable(3,1);
    double d6 = IDHTable(5,1);

    Matrix<double, 6, AXIS> geometryJacobian;

    geometryJacobian<< d6*(sin(jointAngles[4])*(cos(jointAngles[0])*sin(jointAngles[3]) + cos(jointAngles[3])*sin(jointAngles[0])*sin(jointAngles[1] + jointAngles[2])) - cos(jointAngles[4])*sin(jointAngles[0])*cos(jointAngles[1] + jointAngles[2])) + a2*sin(jointAngles[0])*sin(jointAngles[1]) - d4*sin(jointAngles[0])*cos(jointAngles[1] + jointAngles[2]), -cos(jointAngles[0])*(a2*cos(jointAngles[1]) + d4*sin(jointAngles[1] + jointAngles[2]) + d6*cos(jointAngles[4])*sin(jointAngles[1] + jointAngles[2]) + d6*cos(jointAngles[3])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])), -cos(jointAngles[0])*(d4*sin(jointAngles[1] + jointAngles[2]) + d6*cos(jointAngles[4])*sin(jointAngles[1] + jointAngles[2]) + d6*cos(jointAngles[3])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])), d6*sin(jointAngles[4])*(cos(jointAngles[3])*sin(jointAngles[0]) + cos(jointAngles[0])*cos(jointAngles[1])*sin(jointAngles[2])*sin(jointAngles[3]) + cos(jointAngles[0])*cos(jointAngles[2])*sin(jointAngles[1])*sin(jointAngles[3])), d6*cos(jointAngles[4])*sin(jointAngles[0])*sin(jointAngles[3]) - d6*cos(jointAngles[0])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2]) - d6*cos(jointAngles[0])*cos(jointAngles[3])*cos(jointAngles[4])*sin(jointAngles[1] + jointAngles[2]), 0,
                       d6*(sin(jointAngles[4])*(sin(jointAngles[0])*sin(jointAngles[3]) - cos(jointAngles[0])*cos(jointAngles[3])*sin(jointAngles[1] + jointAngles[2])) + cos(jointAngles[0])*cos(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])) - a2*cos(jointAngles[0])*sin(jointAngles[1]) + d4*cos(jointAngles[0])*cos(jointAngles[1] + jointAngles[2]), -sin(jointAngles[0])*(a2*cos(jointAngles[1]) + d4*sin(jointAngles[1] + jointAngles[2]) + d6*cos(jointAngles[4])*sin(jointAngles[1] + jointAngles[2]) + d6*cos(jointAngles[3])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])), -sin(jointAngles[0])*(d4*sin(jointAngles[1] + jointAngles[2]) + d6*cos(jointAngles[4])*sin(jointAngles[1] + jointAngles[2]) + d6*cos(jointAngles[3])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])), d6*sin(jointAngles[4])*(cos(jointAngles[1])*sin(jointAngles[0])*sin(jointAngles[2])*sin(jointAngles[3]) - cos(jointAngles[0])*cos(jointAngles[3]) + cos(jointAngles[2])*sin(jointAngles[0])*sin(jointAngles[1])*sin(jointAngles[3])), - d6*(cos(jointAngles[3])*sin(jointAngles[0]) + cos(jointAngles[0])*sin(jointAngles[3])*sin(jointAngles[1] + jointAngles[2]))*(cos(jointAngles[4])*sin(jointAngles[1] + jointAngles[2]) + cos(jointAngles[3])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])) - d6*sin(jointAngles[3])*cos(jointAngles[1] + jointAngles[2])*(sin(jointAngles[4])*(sin(jointAngles[0])*sin(jointAngles[3]) - cos(jointAngles[0])*cos(jointAngles[3])*sin(jointAngles[1] + jointAngles[2])) + cos(jointAngles[0])*cos(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])), 0,
                       0, d4*cos(jointAngles[1])*cos(jointAngles[2]) - a2*sin(jointAngles[1]) - d4*sin(jointAngles[1])*sin(jointAngles[2]) + d6*cos(jointAngles[1])*cos(jointAngles[2])*cos(jointAngles[4]) - d6*cos(jointAngles[4])*sin(jointAngles[1])*sin(jointAngles[2]) - d6*cos(jointAngles[1])*cos(jointAngles[3])*sin(jointAngles[2])*sin(jointAngles[4]) - d6*cos(jointAngles[2])*cos(jointAngles[3])*sin(jointAngles[1])*sin(jointAngles[4]), d4*cos(jointAngles[1])*cos(jointAngles[2]) - d4*sin(jointAngles[1])*sin(jointAngles[2]) + d6*cos(jointAngles[1])*cos(jointAngles[2])*cos(jointAngles[4]) - d6*cos(jointAngles[4])*sin(jointAngles[1])*sin(jointAngles[2]) - d6*cos(jointAngles[1])*cos(jointAngles[3])*sin(jointAngles[2])*sin(jointAngles[4]) - d6*cos(jointAngles[2])*cos(jointAngles[3])*sin(jointAngles[1])*sin(jointAngles[4]), -d6*sin(jointAngles[3])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2]), -d6*(cos(jointAngles[1])*sin(jointAngles[2])*sin(jointAngles[4]) + cos(jointAngles[2])*sin(jointAngles[1])*sin(jointAngles[4]) - cos(jointAngles[1])*cos(jointAngles[2])*cos(jointAngles[3])*cos(jointAngles[4]) + cos(jointAngles[3])*cos(jointAngles[4])*sin(jointAngles[1])*sin(jointAngles[2])), 0,
                       0, sin(jointAngles[0]), sin(jointAngles[0]), cos(jointAngles[0])*cos(jointAngles[1] + jointAngles[2]), cos(jointAngles[3])*sin(jointAngles[0]) + cos(jointAngles[0])*sin(jointAngles[3])*sin(jointAngles[1] + jointAngles[2]), sin(jointAngles[4])*(sin(jointAngles[0])*sin(jointAngles[3]) - cos(jointAngles[0])*cos(jointAngles[3])*sin(jointAngles[1] + jointAngles[2])) + cos(jointAngles[0])*cos(jointAngles[4])*cos(jointAngles[1] + jointAngles[2]),
                       0, -cos(jointAngles[0]), -cos(jointAngles[0]), sin(jointAngles[0])*cos(jointAngles[1] + jointAngles[2]), sin(jointAngles[0])*sin(jointAngles[3])*sin(jointAngles[1] + jointAngles[2]) - cos(jointAngles[0])*cos(jointAngles[3]), cos(jointAngles[4])*sin(jointAngles[0])*cos(jointAngles[1] + jointAngles[2]) - sin(jointAngles[4])*(cos(jointAngles[0])*sin(jointAngles[3]) + cos(jointAngles[3])*sin(jointAngles[0])*sin(jointAngles[1] + jointAngles[2])),
                       1, 0, 0, sin(jointAngles[1] + jointAngles[2]), -sin(jointAngles[3])*cos(jointAngles[1] + jointAngles[2]), cos(jointAngles[4])*sin(jointAngles[1] + jointAngles[2]) + cos(jointAngles[3])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2]);

    return geometryJacobian;
};

Matrix<double, 6, AXIS> IDHKinematics::GetRobotJacobianDot(Matrix<double, 1, AXIS> jointAngles, Matrix<double, 1, AXIS> jointVelocity) const{

    double a2 = IDHTable(1,2);
    double d4 = IDHTable(3,1);
    double d6 = IDHTable(5,1);

    Matrix<double, 6, AXIS> geometryJacobianDot;

    geometryJacobianDot << a2*cos(jointAngles[0])*sin(jointAngles[1])*jointVelocity[0] + a2*cos(jointAngles[1])*sin(jointAngles[0])*jointVelocity[1] - d4*cos(jointAngles[0])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[0] + d4*sin(jointAngles[0])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[1] + d4*sin(jointAngles[0])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[2] + d6*cos(jointAngles[0])*cos(jointAngles[3])*sin(jointAngles[4])*jointVelocity[3] + d6*cos(jointAngles[0])*cos(jointAngles[4])*sin(jointAngles[3])*jointVelocity[4] - d6*sin(jointAngles[0])*sin(jointAngles[3])*sin(jointAngles[4])*jointVelocity[0] - d6*cos(jointAngles[0])*cos(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[0] + d6*cos(jointAngles[4])*sin(jointAngles[0])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[1] + d6*cos(jointAngles[4])*sin(jointAngles[0])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[2] + d6*sin(jointAngles[0])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[4] + d6*cos(jointAngles[0])*cos(jointAngles[3])*sin(jointAngles[4])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[0] + d6*cos(jointAngles[3])*sin(jointAngles[0])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[1] + d6*cos(jointAngles[3])*sin(jointAngles[0])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[2] + d6*cos(jointAngles[3])*cos(jointAngles[4])*sin(jointAngles[0])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[4] - d6*sin(jointAngles[0])*sin(jointAngles[3])*sin(jointAngles[4])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[3], cos(jointAngles[0])*(a2*sin(jointAngles[1])*jointVelocity[1] - d4*cos(jointAngles[1] + jointAngles[2])*(jointVelocity[1] + jointVelocity[2]) + d6*sin(jointAngles[4])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[4] - d6*cos(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*(jointVelocity[1] + jointVelocity[2]) - d6*cos(jointAngles[3])*cos(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[4] + d6*sin(jointAngles[3])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[3] + d6*cos(jointAngles[3])*sin(jointAngles[4])*sin(jointAngles[1] + jointAngles[2])*(jointVelocity[1] + jointVelocity[2])) + sin(jointAngles[0])*jointVelocity[0]*(a2*cos(jointAngles[1]) + d4*sin(jointAngles[1] + jointAngles[2]) + d6*cos(jointAngles[4])*sin(jointAngles[1] + jointAngles[2]) + d6*cos(jointAngles[3])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])), sin(jointAngles[0])*jointVelocity[0]*(d4*sin(jointAngles[1] + jointAngles[2]) + d6*cos(jointAngles[4])*sin(jointAngles[1] + jointAngles[2]) + d6*cos(jointAngles[3])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])) - cos(jointAngles[0])*(d4*cos(jointAngles[1] + jointAngles[2])*(jointVelocity[1] + jointVelocity[2]) - d6*sin(jointAngles[4])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[4] + d6*cos(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*(jointVelocity[1] + jointVelocity[2]) + d6*cos(jointAngles[3])*cos(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[4] - d6*sin(jointAngles[3])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[3] - d6*cos(jointAngles[3])*sin(jointAngles[4])*sin(jointAngles[1] + jointAngles[2])*(jointVelocity[1] + jointVelocity[2])), d6*sin(jointAngles[4])*(cos(jointAngles[0])*cos(jointAngles[3])*jointVelocity[0] - sin(jointAngles[0])*sin(jointAngles[3])*jointVelocity[3] + cos(jointAngles[0])*cos(jointAngles[1])*cos(jointAngles[2])*sin(jointAngles[3])*jointVelocity[1] + cos(jointAngles[0])*cos(jointAngles[1])*cos(jointAngles[2])*sin(jointAngles[3])*jointVelocity[2] + cos(jointAngles[0])*cos(jointAngles[1])*cos(jointAngles[3])*sin(jointAngles[2])*jointVelocity[3] + cos(jointAngles[0])*cos(jointAngles[2])*cos(jointAngles[3])*sin(jointAngles[1])*jointVelocity[3] - cos(jointAngles[1])*sin(jointAngles[0])*sin(jointAngles[2])*sin(jointAngles[3])*jointVelocity[0] - cos(jointAngles[2])*sin(jointAngles[0])*sin(jointAngles[1])*sin(jointAngles[3])*jointVelocity[0] - cos(jointAngles[0])*sin(jointAngles[1])*sin(jointAngles[2])*sin(jointAngles[3])*jointVelocity[1] - cos(jointAngles[0])*sin(jointAngles[1])*sin(jointAngles[2])*sin(jointAngles[3])*jointVelocity[2]) + d6*cos(jointAngles[4])*jointVelocity[4]*(cos(jointAngles[3])*sin(jointAngles[0]) + cos(jointAngles[0])*cos(jointAngles[1])*sin(jointAngles[2])*sin(jointAngles[3]) + cos(jointAngles[0])*cos(jointAngles[2])*sin(jointAngles[1])*sin(jointAngles[3])), d6*cos(jointAngles[0])*cos(jointAngles[4])*sin(jointAngles[3])*jointVelocity[0] + d6*cos(jointAngles[3])*cos(jointAngles[4])*sin(jointAngles[0])*jointVelocity[3] - d6*sin(jointAngles[0])*sin(jointAngles[3])*sin(jointAngles[4])*jointVelocity[4] - d6*cos(jointAngles[0])*cos(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[4] + d6*sin(jointAngles[0])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[0] + d6*cos(jointAngles[0])*sin(jointAngles[4])*sin(jointAngles[1] + jointAngles[2])*(jointVelocity[1] + jointVelocity[2]) + d6*cos(jointAngles[3])*cos(jointAngles[4])*sin(jointAngles[0])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[0] + d6*cos(jointAngles[0])*cos(jointAngles[4])*sin(jointAngles[3])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[3] + d6*cos(jointAngles[0])*cos(jointAngles[3])*sin(jointAngles[4])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[4] - d6*cos(jointAngles[0])*cos(jointAngles[3])*cos(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*(jointVelocity[1] + jointVelocity[2]), 0,
                           a2*sin(jointAngles[0])*sin(jointAngles[1])*jointVelocity[0] - a2*cos(jointAngles[0])*cos(jointAngles[1])*jointVelocity[1] - d4*sin(jointAngles[0])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[0] - d4*cos(jointAngles[0])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[1] - d4*cos(jointAngles[0])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[2] + d6*cos(jointAngles[0])*sin(jointAngles[3])*sin(jointAngles[4])*jointVelocity[0] + d6*cos(jointAngles[3])*sin(jointAngles[0])*sin(jointAngles[4])*jointVelocity[3] + d6*cos(jointAngles[4])*sin(jointAngles[0])*sin(jointAngles[3])*jointVelocity[4] - d6*cos(jointAngles[4])*sin(jointAngles[0])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[0] - d6*cos(jointAngles[0])*cos(jointAngles[4])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[1] - d6*cos(jointAngles[0])*cos(jointAngles[4])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[2] - d6*cos(jointAngles[0])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[4] - d6*cos(jointAngles[0])*cos(jointAngles[3])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[1] - d6*cos(jointAngles[0])*cos(jointAngles[3])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[2] - d6*cos(jointAngles[0])*cos(jointAngles[3])*cos(jointAngles[4])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[4] + d6*cos(jointAngles[3])*sin(jointAngles[0])*sin(jointAngles[4])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[0] + d6*cos(jointAngles[0])*sin(jointAngles[3])*sin(jointAngles[4])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[3], sin(jointAngles[0])*(a2*sin(jointAngles[1])*jointVelocity[1] - d4*cos(jointAngles[1] + jointAngles[2])*(jointVelocity[1] + jointVelocity[2]) + d6*sin(jointAngles[4])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[4] - d6*cos(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*(jointVelocity[1] + jointVelocity[2]) - d6*cos(jointAngles[3])*cos(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[4] + d6*sin(jointAngles[3])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[3] + d6*cos(jointAngles[3])*sin(jointAngles[4])*sin(jointAngles[1] + jointAngles[2])*(jointVelocity[1] + jointVelocity[2])) - cos(jointAngles[0])*jointVelocity[0]*(a2*cos(jointAngles[1]) + d4*sin(jointAngles[1] + jointAngles[2]) + d6*cos(jointAngles[4])*sin(jointAngles[1] + jointAngles[2]) + d6*cos(jointAngles[3])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])), - sin(jointAngles[0])*(d4*cos(jointAngles[1] + jointAngles[2])*(jointVelocity[1] + jointVelocity[2]) - d6*sin(jointAngles[4])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[4] + d6*cos(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*(jointVelocity[1] + jointVelocity[2]) + d6*cos(jointAngles[3])*cos(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[4] - d6*sin(jointAngles[3])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[3] - d6*cos(jointAngles[3])*sin(jointAngles[4])*sin(jointAngles[1] + jointAngles[2])*(jointVelocity[1] + jointVelocity[2])) - cos(jointAngles[0])*jointVelocity[0]*(d4*sin(jointAngles[1] + jointAngles[2]) + d6*cos(jointAngles[4])*sin(jointAngles[1] + jointAngles[2]) + d6*cos(jointAngles[3])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])), d6*sin(jointAngles[4])*(cos(jointAngles[3])*sin(jointAngles[0])*jointVelocity[0] + cos(jointAngles[0])*sin(jointAngles[3])*jointVelocity[3] + cos(jointAngles[0])*cos(jointAngles[1])*sin(jointAngles[2])*sin(jointAngles[3])*jointVelocity[0] + cos(jointAngles[0])*cos(jointAngles[2])*sin(jointAngles[1])*sin(jointAngles[3])*jointVelocity[0] + cos(jointAngles[1])*cos(jointAngles[2])*sin(jointAngles[0])*sin(jointAngles[3])*jointVelocity[1] + cos(jointAngles[1])*cos(jointAngles[2])*sin(jointAngles[0])*sin(jointAngles[3])*jointVelocity[2] + cos(jointAngles[1])*cos(jointAngles[3])*sin(jointAngles[0])*sin(jointAngles[2])*jointVelocity[3] + cos(jointAngles[2])*cos(jointAngles[3])*sin(jointAngles[0])*sin(jointAngles[1])*jointVelocity[3] - sin(jointAngles[0])*sin(jointAngles[1])*sin(jointAngles[2])*sin(jointAngles[3])*jointVelocity[1] - sin(jointAngles[0])*sin(jointAngles[1])*sin(jointAngles[2])*sin(jointAngles[3])*jointVelocity[2]) + d6*cos(jointAngles[4])*jointVelocity[4]*(cos(jointAngles[1])*sin(jointAngles[0])*sin(jointAngles[2])*sin(jointAngles[3]) - cos(jointAngles[0])*cos(jointAngles[3]) + cos(jointAngles[2])*sin(jointAngles[0])*sin(jointAngles[1])*sin(jointAngles[3])), d6*cos(jointAngles[4])*sin(jointAngles[0])*sin(jointAngles[3])*jointVelocity[0] - d6*cos(jointAngles[0])*cos(jointAngles[3])*cos(jointAngles[4])*jointVelocity[3] + d6*cos(jointAngles[0])*sin(jointAngles[3])*sin(jointAngles[4])*jointVelocity[4] - d6*cos(jointAngles[0])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[0] - d6*cos(jointAngles[4])*sin(jointAngles[0])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[4] + d6*sin(jointAngles[0])*sin(jointAngles[4])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[1] + d6*sin(jointAngles[0])*sin(jointAngles[4])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[2] - d6*cos(jointAngles[0])*cos(jointAngles[3])*cos(jointAngles[4])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[0] - d6*cos(jointAngles[3])*cos(jointAngles[4])*sin(jointAngles[0])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[1] - d6*cos(jointAngles[3])*cos(jointAngles[4])*sin(jointAngles[0])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[2] + d6*cos(jointAngles[4])*sin(jointAngles[0])*sin(jointAngles[3])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[3] + d6*cos(jointAngles[3])*sin(jointAngles[0])*sin(jointAngles[4])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[4], 0,
                           0, d6*sin(jointAngles[1])*sin(jointAngles[2])*sin(jointAngles[4])*jointVelocity[4] - d4*cos(jointAngles[1])*sin(jointAngles[2])*jointVelocity[1] - d4*cos(jointAngles[2])*sin(jointAngles[1])*jointVelocity[1] - d4*cos(jointAngles[1])*sin(jointAngles[2])*jointVelocity[2] - d4*cos(jointAngles[2])*sin(jointAngles[1])*jointVelocity[2] - d6*cos(jointAngles[1])*cos(jointAngles[4])*sin(jointAngles[2])*jointVelocity[1] - d6*cos(jointAngles[2])*cos(jointAngles[4])*sin(jointAngles[1])*jointVelocity[1] - d6*cos(jointAngles[1])*cos(jointAngles[4])*sin(jointAngles[2])*jointVelocity[2] - d6*cos(jointAngles[2])*cos(jointAngles[4])*sin(jointAngles[1])*jointVelocity[2] - d6*cos(jointAngles[1])*cos(jointAngles[2])*sin(jointAngles[4])*jointVelocity[4] - a2*cos(jointAngles[1])*jointVelocity[1] - d6*cos(jointAngles[1])*cos(jointAngles[2])*cos(jointAngles[3])*sin(jointAngles[4])*jointVelocity[1] - d6*cos(jointAngles[1])*cos(jointAngles[2])*cos(jointAngles[3])*sin(jointAngles[4])*jointVelocity[2] - d6*cos(jointAngles[1])*cos(jointAngles[3])*cos(jointAngles[4])*sin(jointAngles[2])*jointVelocity[4] - d6*cos(jointAngles[2])*cos(jointAngles[3])*cos(jointAngles[4])*sin(jointAngles[1])*jointVelocity[4] + d6*cos(jointAngles[3])*sin(jointAngles[1])*sin(jointAngles[2])*sin(jointAngles[4])*jointVelocity[1] + d6*cos(jointAngles[3])*sin(jointAngles[1])*sin(jointAngles[2])*sin(jointAngles[4])*jointVelocity[2] + d6*cos(jointAngles[1])*sin(jointAngles[2])*sin(jointAngles[3])*sin(jointAngles[4])*jointVelocity[3] + d6*cos(jointAngles[2])*sin(jointAngles[1])*sin(jointAngles[3])*sin(jointAngles[4])*jointVelocity[3], d6*sin(jointAngles[1])*sin(jointAngles[2])*sin(jointAngles[4])*jointVelocity[4] - d4*cos(jointAngles[2])*sin(jointAngles[1])*jointVelocity[1] - d4*cos(jointAngles[1])*sin(jointAngles[2])*jointVelocity[2] - d4*cos(jointAngles[2])*sin(jointAngles[1])*jointVelocity[2] - d6*cos(jointAngles[1])*cos(jointAngles[4])*sin(jointAngles[2])*jointVelocity[1] - d6*cos(jointAngles[2])*cos(jointAngles[4])*sin(jointAngles[1])*jointVelocity[1] - d6*cos(jointAngles[1])*cos(jointAngles[4])*sin(jointAngles[2])*jointVelocity[2] - d6*cos(jointAngles[2])*cos(jointAngles[4])*sin(jointAngles[1])*jointVelocity[2] - d6*cos(jointAngles[1])*cos(jointAngles[2])*sin(jointAngles[4])*jointVelocity[4] - d4*cos(jointAngles[1])*sin(jointAngles[2])*jointVelocity[1] - d6*cos(jointAngles[1])*cos(jointAngles[2])*cos(jointAngles[3])*sin(jointAngles[4])*jointVelocity[1] - d6*cos(jointAngles[1])*cos(jointAngles[2])*cos(jointAngles[3])*sin(jointAngles[4])*jointVelocity[2] - d6*cos(jointAngles[1])*cos(jointAngles[3])*cos(jointAngles[4])*sin(jointAngles[2])*jointVelocity[4] - d6*cos(jointAngles[2])*cos(jointAngles[3])*cos(jointAngles[4])*sin(jointAngles[1])*jointVelocity[4] + d6*cos(jointAngles[3])*sin(jointAngles[1])*sin(jointAngles[2])*sin(jointAngles[4])*jointVelocity[1] + d6*cos(jointAngles[3])*sin(jointAngles[1])*sin(jointAngles[2])*sin(jointAngles[4])*jointVelocity[2] + d6*cos(jointAngles[1])*sin(jointAngles[2])*sin(jointAngles[3])*sin(jointAngles[4])*jointVelocity[3] + d6*cos(jointAngles[2])*sin(jointAngles[1])*sin(jointAngles[3])*sin(jointAngles[4])*jointVelocity[3], d6*sin(jointAngles[3])*sin(jointAngles[4])*sin(jointAngles[1] + jointAngles[2])*(jointVelocity[1] + jointVelocity[2]) - d6*cos(jointAngles[4])*sin(jointAngles[3])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[4] - d6*cos(jointAngles[3])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[3], -d6*(cos(jointAngles[1])*cos(jointAngles[2])*sin(jointAngles[4])*jointVelocity[1] + cos(jointAngles[1])*cos(jointAngles[2])*sin(jointAngles[4])*jointVelocity[2] + cos(jointAngles[1])*cos(jointAngles[4])*sin(jointAngles[2])*jointVelocity[4] + cos(jointAngles[2])*cos(jointAngles[4])*sin(jointAngles[1])*jointVelocity[4] - sin(jointAngles[1])*sin(jointAngles[2])*sin(jointAngles[4])*jointVelocity[1] - sin(jointAngles[1])*sin(jointAngles[2])*sin(jointAngles[4])*jointVelocity[2] + cos(jointAngles[1])*cos(jointAngles[3])*cos(jointAngles[4])*sin(jointAngles[2])*jointVelocity[1] + cos(jointAngles[2])*cos(jointAngles[3])*cos(jointAngles[4])*sin(jointAngles[1])*jointVelocity[1] + cos(jointAngles[1])*cos(jointAngles[3])*cos(jointAngles[4])*sin(jointAngles[2])*jointVelocity[2] + cos(jointAngles[2])*cos(jointAngles[3])*cos(jointAngles[4])*sin(jointAngles[1])*jointVelocity[2] + cos(jointAngles[1])*cos(jointAngles[2])*cos(jointAngles[4])*sin(jointAngles[3])*jointVelocity[3] + cos(jointAngles[1])*cos(jointAngles[2])*cos(jointAngles[3])*sin(jointAngles[4])*jointVelocity[4] - cos(jointAngles[4])*sin(jointAngles[1])*sin(jointAngles[2])*sin(jointAngles[3])*jointVelocity[3] - cos(jointAngles[3])*sin(jointAngles[1])*sin(jointAngles[2])*sin(jointAngles[4])*jointVelocity[4]), 0,
                           0, cos(jointAngles[0])*jointVelocity[0], cos(jointAngles[0])*jointVelocity[0], - sin(jointAngles[0])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[0] - cos(jointAngles[0])*sin(jointAngles[1] + jointAngles[2])*(jointVelocity[1] + jointVelocity[2]), cos(jointAngles[0])*cos(jointAngles[3])*jointVelocity[0] - sin(jointAngles[0])*sin(jointAngles[3])*jointVelocity[3] + cos(jointAngles[0])*cos(jointAngles[3])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[3] - sin(jointAngles[0])*sin(jointAngles[3])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[0] + cos(jointAngles[0])*sin(jointAngles[3])*cos(jointAngles[1] + jointAngles[2])*(jointVelocity[1] + jointVelocity[2]), sin(jointAngles[4])*(cos(jointAngles[0])*sin(jointAngles[3])*jointVelocity[0] + cos(jointAngles[3])*sin(jointAngles[0])*jointVelocity[3] + cos(jointAngles[3])*sin(jointAngles[0])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[0] + cos(jointAngles[0])*sin(jointAngles[3])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[3] - cos(jointAngles[0])*cos(jointAngles[3])*cos(jointAngles[1] + jointAngles[2])*(jointVelocity[1] + jointVelocity[2])) + cos(jointAngles[4])*(sin(jointAngles[0])*sin(jointAngles[3]) - cos(jointAngles[0])*cos(jointAngles[3])*sin(jointAngles[1] + jointAngles[2]))*jointVelocity[4] - cos(jointAngles[4])*sin(jointAngles[0])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[0] - cos(jointAngles[0])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[4] - cos(jointAngles[0])*cos(jointAngles[4])*sin(jointAngles[1] + jointAngles[2])*(jointVelocity[1] + jointVelocity[2]),
                           0, sin(jointAngles[0])*jointVelocity[0], sin(jointAngles[0])*jointVelocity[0], cos(jointAngles[0])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[0] - sin(jointAngles[0])*sin(jointAngles[1] + jointAngles[2])*(jointVelocity[1] + jointVelocity[2]), cos(jointAngles[3])*sin(jointAngles[0])*jointVelocity[0] + cos(jointAngles[0])*sin(jointAngles[3])*jointVelocity[3] + cos(jointAngles[0])*sin(jointAngles[3])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[0] + cos(jointAngles[3])*sin(jointAngles[0])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[3] + sin(jointAngles[0])*sin(jointAngles[3])*cos(jointAngles[1] + jointAngles[2])*(jointVelocity[1] + jointVelocity[2]), cos(jointAngles[0])*cos(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[0] - cos(jointAngles[4])*(cos(jointAngles[0])*sin(jointAngles[3]) + cos(jointAngles[3])*sin(jointAngles[0])*sin(jointAngles[1] + jointAngles[2]))*jointVelocity[4] - sin(jointAngles[4])*(cos(jointAngles[0])*cos(jointAngles[3])*jointVelocity[3] - sin(jointAngles[0])*sin(jointAngles[3])*jointVelocity[0] + cos(jointAngles[0])*cos(jointAngles[3])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[0] - sin(jointAngles[0])*sin(jointAngles[3])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[3] + cos(jointAngles[3])*sin(jointAngles[0])*cos(jointAngles[1] + jointAngles[2])*(jointVelocity[1] + jointVelocity[2])) - sin(jointAngles[0])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[4] - cos(jointAngles[4])*sin(jointAngles[0])*sin(jointAngles[1] + jointAngles[2])*(jointVelocity[1] + jointVelocity[2]),
                           0, 0, 0, cos(jointAngles[1] + jointAngles[2])*(jointVelocity[1] + jointVelocity[2]), sin(jointAngles[3])*sin(jointAngles[1] + jointAngles[2])*(jointVelocity[1] + jointVelocity[2]) - cos(jointAngles[3])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[3], cos(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*(jointVelocity[1] + jointVelocity[2]) - sin(jointAngles[4])*sin(jointAngles[1] + jointAngles[2])*jointVelocity[4] + cos(jointAngles[3])*cos(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[4] - sin(jointAngles[3])*sin(jointAngles[4])*cos(jointAngles[1] + jointAngles[2])*jointVelocity[3] - cos(jointAngles[3])*sin(jointAngles[4])*sin(jointAngles[1] + jointAngles[2])*(jointVelocity[1] + jointVelocity[2]);

    return geometryJacobianDot;
};

void IDHKinematics::TestKinematics(int episode, bool ifShowDebugInfo) {

    int success = 0;
    int failed = 0;
    int singular = 0;

    // 固定亂數種子
    std::random_device rd;
    std::default_random_engine generator( rd() );
    std::uniform_real_distribution<float> unif(0.0, 1.0);

    for(int iep = 0; iep < episode; iep++){

        if(ifShowDebugInfo)
            printf("\n -- Episode %d -- \n", iep+1);

        // rand from -pi ~ pi
        Matrix<double, 1, AXIS> testJoint = (Matrix<double, 1, AXIS>() << (unif(generator) * 2*M_PI) - M_PI, (unif(generator) * 2*M_PI) - M_PI, (unif(generator) * 2*M_PI) - M_PI, (unif(generator) * 2*M_PI) - M_PI, (unif(generator) * 2*M_PI) - M_PI, (unif(generator) * 2*M_PI) - M_PI ).finished();     
        
        // Compute Benchmark solution from Robot F.K.
        auto benchMarkANS = GetEndEffectorPose(testJoint);

        // ---------- Try to solve I.K Problem ------------ //
        Matrix<double, 1, 6> bestFitJoint;
        bool ret = GetInverseKinematics(benchMarkANS, testJoint, bestFitJoint);

        if(!ret){
            if(ifShowDebugInfo){
                printf("    Robot can't Reach that Pose or not choosing solve type! please try again."); 
            }						
            singular ++;
            continue;
        }
        else {
            if(ifShowDebugInfo){
                printf("    Find solution successfully.");
            }
        }	

        // Examine correctness from I.K. solution
        auto examine = GetEndEffectorPose(bestFitJoint);
    
        if(ifShowDebugInfo){
            
            printf("\nShow minimal distance angle: \n\n");
            std::cout << bestFitJoint << std::endl;
            printf("\n");

            printf("\n-------- InPut --------\n");
        
            printf("\nXYZ: \n");
            printf("X: %lf\n", benchMarkANS(0));
            printf("Y: %lf\n", benchMarkANS(1));
            printf("Z: %lf\n", benchMarkANS(2));
            
            printf("\nRYP: \n");
            printf("roll: %lf\n" ,benchMarkANS(5));
            printf("yaw: %lf\n"  ,benchMarkANS(4));
            printf("pitch: %lf\n",benchMarkANS(3));
            
            printf("\n-------- Output --------\n");
            
            printf("\nXYZ: \n");
            printf("X: %lf\n",examine(0));
            printf("Y: %lf\n",examine(1));
            printf("Z: %lf\n",examine(2));
            
            printf("\nRYP: \n");
            printf("roll: %lf\n" ,examine(5));
            printf("yaw: %lf\n"  ,examine(4));
            printf("pitch: %lf\n",examine(3));
            
            printf("\n-------- Joint Degree compare ----\n");
            
            std::cout << "TargetJoint:" << testJoint << std::endl;
            std::cout << "BestFitJoint:" << bestFitJoint << std::endl;
        }

        double justify = 0;

        for(int ijoint = 0; ijoint < AXIS; ijoint++){
            justify += abs(testJoint[ijoint] - bestFitJoint[ijoint]);
        }

        if (justify > 0.01) // solution mismatch
            failed++;
        else
            success++;
        
    } // End for iep loop
    
    // show result
    printf("----------------------\nTest Kinemtics correctness ....\n----------------------\n\n");
    printf("Total try: %d\n", episode);
    printf("Successful try without singularities: %d\n", episode-singular);
    printf("Success: %d\n", success);
    printf("Failed: %d\n", failed);
 
}

Matrix<double, AXIS, 5> IDHKinematics::GetIDHTable() const{
    return IDHTable;
}

void IDHKinematics::ChangeIDHTable(Matrix<double, AXIS, 5> IDHTable){
    MatrixXd deepCopy = IDHTable;
    this -> IDHTable = deepCopy;
}

bool IDHKinematics::ChangeRobotType(std::string robotType){

    if( robotType == "Industrial" || robotType == "INDUSTRIAL" || robotType == "industrial" ){
        this -> robotType = "Industrial";
        return true;
    }
    else if ( robotType == "UR" || robotType == "ur" ){
        this -> robotType = "UR";
        return true;
    }
    else
        return false;
}

Matrix<double, AXIS, 1> InverseKinematicsCompensation(Matrix<double, 3, 1> PoseError, Matrix<double, 3, AXIS> jacobian){

    Matrix<double, AXIS, 3> JT = jacobian.transpose(); // 6 * 3
    Matrix<double, AXIS, 1> deltaParam;
    Matrix<double, AXIS, 3> Jpseudo;
    Matrix<double, 3, 3> temp;

    temp = jacobian * JT ;
    Jpseudo = JT * temp.inverse() ;
    deltaParam = Jpseudo * PoseError; // 6*1 check size

    return deltaParam;
}

// ================================================================================
//
// MDH Kinematics
//
// ================================================================================

Matrix4d MDHKinematics::GetIndividualTransMatrix(double Theta, double D, double A, double Alpha, double Beta){
    Matrix4d J;
    J = GetRX(Alpha) * GetTrans(A, 0, 0) * GetRZ(Theta) * GetTrans(0, 0, D) * GetRY(Beta);
    return J;
}

Matrix<double, 4, AXIS*4> MDHKinematics::GetForwardKinematics(Matrix<double, 1, AXIS> jointAngles){

    Matrix4d T0E = MatrixXd::Identity(4, 4);
    Matrix<double, 4, AXIS*4> TE = MatrixXd::Zero(4, AXIS*4);
    Matrix4d iJ;
    double iTheta, iD, iA, iAlpha, iBeta;

    for (int i = 0; i < AXIS; i++)
    {
        iTheta = MDHTable(i, 0) + jointAngles(0, i);
        iD = MDHTable(i, 1);
        iA = MDHTable(i, 2);
        iAlpha = MDHTable(i, 3);
        iBeta = MDHTable(i, 4);

        iJ = GetIndividualTransMatrix(iTheta, iD, iA, iAlpha, iBeta);
        T0E = T0E * iJ;
        TE.block<4, 4>(0, 0 + i * 4) = T0E;
    }

    return TE;
}

