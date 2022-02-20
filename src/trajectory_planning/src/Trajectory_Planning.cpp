#include "Trajectory_Planning.h"
#define DEBUG_MODE_SWITCH 1

TrajectoryPlanning::TrajectoryPlanning(std::shared_ptr<IDHKinematics> _kinematicsPtr, std::shared_ptr<Constraint> _constraintPtr) : kinematicsPtr(_kinematicsPtr), constraintPtr(_constraintPtr)
{

    constraintPtr->GetHardCartVelBound(CartVelBound);
    constraintPtr->GetHardCartAccBound(CartAccBound);
    constraintPtr->GetHardCartJerkBound(CartJerkBound);
    

}

void TrajectoryPlanning::Print_instruction(void)
{
    printf("\n========================================================\n");
    printf("Please Choose What Kind of Trajectory Planning for operation:\n");
    printf("1. KeyBoard input Joint PTP position\n");
    printf("2. KeyBoard input Cartesian PTP position\n");
    printf("3. KeyBoard input Circle Trajectory Parameter\n");
    printf("Press 9 to exit the simulation program\n");
    printf("========================================================\n");
}

void TrajectoryPlanning::Initialization(int trajectory_mode){
    switch (trajectory_mode)
    {
        case TRAJECTORY_PLANNING_MODE_PTP: // 1
        case TRAJECTORY_PLANNING_MODE_LINE: // 2
        case TRAJECTORY_PLANNING_MODE_CIRCLE: // 3
        {
            printf("Initialization Done...\n");
            CarPosCmd_list.clear();
            CarVelCmd_list.clear();
            CarAccCmd_list.clear();
            JointPosCmd_list.clear();
            JointVelCmd_list.clear();
            JointAccCmd_list.clear();
            TBase_list.clear();
            time_current = 0.0;
            Function_EndFlag = false;
            currentJoint = MatrixXd::Zero(AXIS, 1);
            break;
        }

    }

}

void TrajectoryPlanning::Scurve(VectorXd joints_i, VectorXd joints_f, int mode)
{

    double CartVelBound[TASKSPACE], CartAccBound[TASKSPACE], CartJerkBound[TASKSPACE];
    constraintPtr->GetHardCartVelBound(CartVelBound);
    constraintPtr->GetHardCartAccBound(CartAccBound);
    constraintPtr->GetHardCartJerkBound(CartJerkBound);

    double v_max, a_max;
    switch (mode)
    {
    case 1:
        v_max = CartVelBound[0] * ParameterRatio;
        a_max = CartAccBound[0] * ParameterRatio;
        break;
    case 2:
        v_max = CartVelBound[3] * ParameterRatio;
        a_max = CartAccBound[3] * ParameterRatio;
        break;
    default:
        v_max = CartVelBound[0] * ParameterRatio;
        a_max = CartAccBound[0] * ParameterRatio;
        break;
    }

    printf("v_max:%f , a_max:%f \n", v_max, a_max);

    VectorXd p;
    double dist;

    p = joints_f - joints_i;
    p = p / (p.array().abs()).maxCoeff();
    dist = ((joints_f - joints_i).array().abs()).maxCoeff();

    if (dist < 2 * pow(v_max, 2) / a_max)
    {
        double degradeRatio = dist / (2 * pow(v_max, 2) / a_max);
        v_max *= (degradeRatio * 0.8);
        a_max *= (degradeRatio);
    }

    double jerk = pow(a_max, 2) * alpha / ((1 - alpha) * v_max);
    double t_top = v_max / a_max - a_max / jerk;
    double t0 = 0;
    double t1 = t0 + a_max / jerk;
    double t2 = t1 + t_top;
    double t3 = t2 + a_max / jerk;
    double t4 = dist / v_max;
    double t5 = t4 + a_max / jerk;
    double t6 = t5 + t_top;
    double t7 = t6 + a_max / jerk;

    vector<double> TBase, PosCmd, VelCmd, AccCmd;
    double time_current = 0.0;
    double iPosCmd, iVelCmd, iAccCmd;

    TBase.clear();
    PosCmd.clear();
    VelCmd.clear();
    AccCmd.clear();

    while (time_current <= t7)
    {
        if (time_current <= t1)
        {
            iAccCmd = jerk * time_current;
            iVelCmd = jerk * pow(time_current, 2) / 2;
            iPosCmd = jerk * pow(time_current, 3) / 6;
        }
        else if ((t1 < time_current) && (time_current <= t2))
        {
            iAccCmd = jerk * t1;
            iVelCmd = jerk * time_current * t1 - jerk * pow(t1, 2) / 2;
            iPosCmd = (jerk * pow(time_current, 2) * t1) / 2 - (jerk * time_current * pow(t1, 2)) / 2 + (jerk * pow(t1, 3)) / 6;
        }
        else if ((t2 < time_current) && (time_current <= t3))
        {
            iAccCmd = jerk * t1 - jerk * time_current + jerk * t2;
            iVelCmd = jerk * time_current * t1 - (jerk * pow(t1, 2)) / 2 - (jerk * pow(t2, 2)) / 2 - (jerk * pow(time_current, 2)) / 2 + jerk * time_current * t2;
            iPosCmd = jerk * (-pow(time_current, 3) / 6 + (pow(time_current, 2) * t1) / 2 + (pow(time_current, 2) * t2) / 2 - (time_current * pow(t1, 2)) / 2 - (time_current * pow(t2, 2)) / 2 + pow(t1, 3) / 6 + pow(t2, 3) / 6);
        }
        else if ((t3 < time_current) && (time_current <= t4))
        {
            iAccCmd = 0;
            iVelCmd = jerk * (t1 * t3 - pow(t2, 2) / 2 - pow(t3, 2) / 2 - pow(t1, 2) / 2 + t2 * t3);
            iPosCmd = jerk * (pow(t1, 3) / 6 - (time_current * pow(t1, 2)) / 2 - (t1 * pow(t3, 2)) / 2 + time_current * t1 * t3 + pow(t2, 3) / 6 - (time_current * pow(t2, 2)) / 2 - (t2 * pow(t3, 2)) / 2 + time_current * t2 * t3 + pow(t3, 3) / 3 - (time_current * pow(t3, 2)) / 2);
        }
        else if ((t4 < time_current) && (time_current <= t5))
        {
            iAccCmd = jerk * t4 - jerk * time_current;
            iVelCmd = jerk * (time_current * t4 - pow(t1, 2) / 2 - pow(t2, 2) / 2 - pow(t3, 2) / 2 - pow(t4, 2) / 2 - pow(time_current, 2) / 2 + t1 * t3 + t2 * t3);
            iPosCmd = jerk * (-pow(time_current, 3) / 6 + (pow(time_current, 2) * t4) / 2 - (time_current * pow(t1, 2)) / 2 + time_current * t1 * t3 - (time_current * pow(t2, 2)) / 2 + time_current * t2 * t3 - (time_current * pow(t3, 2)) / 2 - (time_current * pow(t4, 2)) / 2 + pow(t1, 3) / 6 - (t1 * pow(t3, 2)) / 2 + pow(t2, 3) / 6 - (t2 * pow(t3, 2)) / 2 + pow(t3, 3) / 3 + pow(t4, 3) / 6);
        }
        else if ((t5 < time_current) && (time_current <= t6))
        {
            iAccCmd = jerk * t4 - jerk * t5;
            iVelCmd = jerk * (pow(t5, 2) / 2 - time_current * t5 - pow(t1, 2) / 2 - pow(t2, 2) / 2 - pow(t3, 2) / 2 - pow(t4, 2) / 2 + time_current * t4 + t1 * t3 + t2 * t3);
            iPosCmd = jerk * ((pow(time_current, 2) * t4) / 2 - (pow(time_current, 2) * t5) / 2 - (time_current * pow(t1, 2)) / 2 + time_current * t1 * t3 - (time_current * pow(t2, 2)) / 2 + time_current * t2 * t3 - (time_current * pow(t3, 2)) / 2 - (time_current * pow(t4, 2)) / 2 + (time_current * pow(t5, 2)) / 2 + pow(t1, 3) / 6 - (t1 * pow(t3, 2)) / 2 + pow(t2, 3) / 6 - (t2 * pow(t3, 2)) / 2 + pow(t3, 3) / 3 + pow(t4, 3) / 6 - pow(t5, 3) / 6);
        }
        else if ((t6 < time_current) && (time_current <= t7))
        {
            iAccCmd = jerk * time_current + jerk * t4 - jerk * t5 - jerk * t6;
            iVelCmd = jerk * (pow(time_current, 2) / 2 - time_current * t5 - time_current * t6 + time_current * t4 + pow(t5, 2) / 2 + pow(t6, 2) / 2 - pow(t1, 2) / 2 - pow(t2, 2) / 2 - pow(t3, 2) / 2 - pow(t4, 2) / 2 + t1 * t3 + t2 * t3);
            iPosCmd = jerk * (pow(time_current, 3) / 6 + (pow(time_current, 2) * t4) / 2 - (pow(time_current, 2) * t5) / 2 - (pow(time_current, 2) * t6) / 2 - (time_current * pow(t1, 2)) / 2 + time_current * t1 * t3 - (time_current * pow(t2, 2)) / 2 + time_current * t2 * t3 - (time_current * pow(t3, 2)) / 2 - (time_current * pow(t4, 2)) / 2 + (time_current * pow(t5, 2)) / 2 + (time_current * pow(t6, 2)) / 2 + pow(t1, 3) / 6 - (t1 * pow(t3, 2)) / 2 + pow(t2, 3) / 6 - (t2 * pow(t3, 2)) / 2 + pow(t3, 3) / 3 + pow(t4, 3) / 6 - pow(t5, 3) / 6 - pow(t6, 3) / 6);
        }
        else
        {
            iAccCmd = 0;
            iVelCmd = 0;
            iPosCmd = 0;
        }

        TBase.push_back(time_current);
        PosCmd.push_back(iPosCmd);
        VelCmd.push_back(iVelCmd);
        AccCmd.push_back(iAccCmd);

        time_current += SAMPLINGTIME;
    }

    VectorXd t = Map<VectorXd, Unaligned>(TBase.data(), TBase.size());
    VectorXd linspace_p = Map<VectorXd, Unaligned>(PosCmd.data(), PosCmd.size());
    VectorXd linspace_v = Map<VectorXd, Unaligned>(VelCmd.data(), TBase.size());
    VectorXd linspace_a = Map<VectorXd, Unaligned>(AccCmd.data(), AccCmd.size());

    Matrix<double, Dynamic, Dynamic> pos, vel, acc;

    pos = p * (linspace_p.transpose());
    for (int ijoint = 0; ijoint < pos.rows(); ijoint++)
    {
        (pos.array()).row(ijoint) += joints_i(ijoint);
    } // pos.rows() is dim. of input
    for (int ijoint = 0; ijoint < pos.rows(); ijoint++)
    {
        pos.array().row(ijoint).tail(1) = joints_f(ijoint);
    }
    vel = p * (linspace_v.transpose());
    acc = p * (linspace_a.transpose());

    t_result = t;
    Scurve_pos = pos;
    Scurve_vel = vel;
    Scurve_acc = acc;
}

void TrajectoryPlanning::Scurve_sp(VectorXd joints_i, VectorXd joints_f, int mode)
{

    if (time_current == 0.0)
    {
        /* get v_max & a_max parameter from constraint */
        double v_max, a_max;
        switch (mode) // ParameterRatio: Ratio of boundary
        {
        case 1: // Linear
            v_max = CartVelBound[0] * ParameterRatio;
            a_max = CartAccBound[0] * ParameterRatio;
            break;
        case 2: // Angular (use in circle path)
            v_max = CartVelBound[3] * ParameterRatio;
            a_max = CartAccBound[3] * ParameterRatio;
            break;
        default: // Linear
            v_max = CartVelBound[0] * ParameterRatio;
            a_max = CartAccBound[0] * ParameterRatio;
            break;
        }

        double dist;
        p = joints_f - joints_i;
        p = p / (p.array().abs()).maxCoeff();
        dist = ((joints_f - joints_i).array().abs()).maxCoeff();

        if (dist < 2 * pow(v_max, 2) / a_max)
        {

            printf("Trajectory not length enougth for acceleration... , try to degrade max velocity\n");

            double degradeRatio = dist / (2 * pow(v_max, 2) / a_max);
            v_max *= (degradeRatio * 0.8);
            a_max *= (degradeRatio);
        }

        printf("v_max:%f , a_max:%f \n", v_max, a_max);

        jerk = pow(a_max, 2) * alpha / ((1 - alpha) * v_max);
        t_top = v_max / a_max - a_max / jerk;
        t0 = 0;
        t1 = t0 + a_max / jerk;
        t2 = t1 + t_top;
        t3 = t2 + a_max / jerk;
        t4 = dist / v_max;
        t5 = t4 + a_max / jerk;
        t6 = t5 + t_top;
        t7 = t6 + a_max / jerk;
    }

    double iPos, iVel, iAcc;

    if (time_current <= t7)
    {

        if (time_current <= t1)
        {
            iAcc = jerk * time_current;
            iVel = jerk * pow(time_current, 2) / 2;
            iPos = jerk * pow(time_current, 3) / 6;
        }
        else if ((t1 < time_current) && (time_current <= t2))
        {
            iAcc = jerk * t1;
            iVel = jerk * time_current * t1 - jerk * pow(t1, 2) / 2;
            iPos = (jerk * pow(time_current, 2) * t1) / 2 - (jerk * time_current * pow(t1, 2)) / 2 + (jerk * pow(t1, 3)) / 6;
        }
        else if ((t2 < time_current) && (time_current <= t3))
        {
            iAcc = jerk * t1 - jerk * time_current + jerk * t2;
            iVel = jerk * time_current * t1 - (jerk * pow(t1, 2)) / 2 - (jerk * pow(t2, 2)) / 2 - (jerk * pow(time_current, 2)) / 2 + jerk * time_current * t2;
            iPos = jerk * (-pow(time_current, 3) / 6 + (pow(time_current, 2) * t1) / 2 + (pow(time_current, 2) * t2) / 2 - (time_current * pow(t1, 2)) / 2 - (time_current * pow(t2, 2)) / 2 + pow(t1, 3) / 6 + pow(t2, 3) / 6);
        }
        else if ((t3 < time_current) && (time_current <= t4))
        {
            iAcc = 0;
            iVel = jerk * (t1 * t3 - pow(t2, 2) / 2 - pow(t3, 2) / 2 - pow(t1, 2) / 2 + t2 * t3);
            iPos = jerk * (pow(t1, 3) / 6 - (time_current * pow(t1, 2)) / 2 - (t1 * pow(t3, 2)) / 2 + time_current * t1 * t3 + pow(t2, 3) / 6 - (time_current * pow(t2, 2)) / 2 - (t2 * pow(t3, 2)) / 2 + time_current * t2 * t3 + pow(t3, 3) / 3 - (time_current * pow(t3, 2)) / 2);
        }
        else if ((t4 < time_current) && (time_current <= t5))
        {
            iAcc = jerk * t4 - jerk * time_current;
            iVel = jerk * (time_current * t4 - pow(t1, 2) / 2 - pow(t2, 2) / 2 - pow(t3, 2) / 2 - pow(t4, 2) / 2 - pow(time_current, 2) / 2 + t1 * t3 + t2 * t3);
            iPos = jerk * (-pow(time_current, 3) / 6 + (pow(time_current, 2) * t4) / 2 - (time_current * pow(t1, 2)) / 2 + time_current * t1 * t3 - (time_current * pow(t2, 2)) / 2 + time_current * t2 * t3 - (time_current * pow(t3, 2)) / 2 - (time_current * pow(t4, 2)) / 2 + pow(t1, 3) / 6 - (t1 * pow(t3, 2)) / 2 + pow(t2, 3) / 6 - (t2 * pow(t3, 2)) / 2 + pow(t3, 3) / 3 + pow(t4, 3) / 6);
        }
        else if ((t5 < time_current) && (time_current <= t6))
        {
            iAcc = jerk * t4 - jerk * t5;
            iVel = jerk * (pow(t5, 2) / 2 - time_current * t5 - pow(t1, 2) / 2 - pow(t2, 2) / 2 - pow(t3, 2) / 2 - pow(t4, 2) / 2 + time_current * t4 + t1 * t3 + t2 * t3);
            iPos = jerk * ((pow(time_current, 2) * t4) / 2 - (pow(time_current, 2) * t5) / 2 - (time_current * pow(t1, 2)) / 2 + time_current * t1 * t3 - (time_current * pow(t2, 2)) / 2 + time_current * t2 * t3 - (time_current * pow(t3, 2)) / 2 - (time_current * pow(t4, 2)) / 2 + (time_current * pow(t5, 2)) / 2 + pow(t1, 3) / 6 - (t1 * pow(t3, 2)) / 2 + pow(t2, 3) / 6 - (t2 * pow(t3, 2)) / 2 + pow(t3, 3) / 3 + pow(t4, 3) / 6 - pow(t5, 3) / 6);
        }
        else if ((t6 < time_current) && (time_current <= t7))
        {
            iAcc = jerk * time_current + jerk * t4 - jerk * t5 - jerk * t6;
            iVel = jerk * (pow(time_current, 2) / 2 - time_current * t5 - time_current * t6 + time_current * t4 + pow(t5, 2) / 2 + pow(t6, 2) / 2 - pow(t1, 2) / 2 - pow(t2, 2) / 2 - pow(t3, 2) / 2 - pow(t4, 2) / 2 + t1 * t3 + t2 * t3);
            iPos = jerk * (pow(time_current, 3) / 6 + (pow(time_current, 2) * t4) / 2 - (pow(time_current, 2) * t5) / 2 - (pow(time_current, 2) * t6) / 2 - (time_current * pow(t1, 2)) / 2 + time_current * t1 * t3 - (time_current * pow(t2, 2)) / 2 + time_current * t2 * t3 - (time_current * pow(t3, 2)) / 2 - (time_current * pow(t4, 2)) / 2 + (time_current * pow(t5, 2)) / 2 + (time_current * pow(t6, 2)) / 2 + pow(t1, 3) / 6 - (t1 * pow(t3, 2)) / 2 + pow(t2, 3) / 6 - (t2 * pow(t3, 2)) / 2 + pow(t3, 3) / 3 + pow(t4, 3) / 6 - pow(t5, 3) / 6 - pow(t6, 3) / 6);
        }

        iPosCmd = p * iPos;
        iVelCmd = p * iVel;
        iAccCmd = p * iAcc;

        for (int ijoint = 0; ijoint < iPosCmd.rows(); ijoint++)
        {
            iPosCmd[ijoint] += joints_i(ijoint);
        }

    } // end if Time > t7
    else
    {
        for (int ijoint = 0; ijoint < iPosCmd.rows(); ijoint++)
        {
            iPosCmd[ijoint] = joints_f(ijoint);
        }
        iVelCmd = p * 0;
        iAccCmd = p * 0;
    }

    TBase_list.push_back(time_current);

}

void TrajectoryPlanning::Circle_posture_setup(int posture_mode){
    
    switch (posture_mode)
    {
        /* ----- 1. Posture rotate along constant axis ----- */
        case 1:
        {
            printf("Circle posture Mode 1 : Posture rotate along constant axis.\n");
            printf("Please input initial posture.\n");
            
            cout << "Please input roll(x) angle (degree)....\n"; //0
            cin >> roll;
            cout << "Please input pitch(y) angle (degree)....\n"; //150
            cin >> pitch;
            cout << "Please input yaw(z) angle (degree)....\n"; //150
            cin >> yaw;

            break;
        }
        /* ----- 2. Shooting mode (Assign depression angle) ----- */
        case 2:
        {
            printf("Circle posture Mode 2 : Shooting mode (Assign depression angle).\n");

            cout << "Please input depression angle (degree)....\n";
            cin >> depression_angle;
            
            if (depression_angle > depression_angle_limit){
                depression_angle = depression_angle_limit;
            }
            else if (depression_angle < 0){
                depression_angle = 0;
            }

            break;
        }
        /* ----- 3. Constant normal plane direction posture ----- */
        case 3:
        {
            printf("Circle posture Mode 3 : Constant normal plane direction posture.\n");
            break;
        }
        /* ----- 4. Spherical Linear Interpolation ----- */
        case 4:
        {
            printf("Circle posture Mode 4 : Spherical Linear Interpolation.\n");

            cout << "Please input initial ROLL(x) angle (degree)....\n"; // 90
            cin >> roll_i;
            cout << "Please input initial PITCH(y) angle (degree)....\n"; // 150
            cin >> pitch_i;
            cout << "Please input initial YAW(z) angle (degree)....\n"; // 0
            cin >> yaw_i;
            cout << "Please input target ROLL(x) angle (degree)....\n"; // 180
            cin >> roll_f;
            cout << "Please input target PITCH(y) angle (degree)....\n"; // 180
            cin >> pitch_f;
            cout << "Please input target YAW(z) angle (degree)....\n"; // 90
            cin >> yaw_f;
            break;
        }
        default:
        {
            printf("Error input !!!\n");
            return;
        }
    }

}

Vector3d TrajectoryPlanning::Get_circle_posture(int posture_mode , double TargetRad){
    
    Vector3d iPosture;

    switch (posture_mode)
    {
        case 1:
        {
            Matrix3d iR;
            Vector3d iYPR;
            iR = RotationVector(R_initial, v_normal); // R_initial v_normal C.K.
            iYPR = RotationMatrixToYPR(iR); // C.K. < YPR !!! >
            iPosture = iYPR;
            // cout << iPosture << endl; // C.K.
            break;
        }
        case 2: // CK
        {
            Matrix3d iR;
            Vector3d iYPR;
            iR = RotationVector(R, v_normal);
            iYPR = RotationMatrixToYPR(iR);
            iPosture = iYPR;
            break;
        }
        case 3:
        {
            iPosture = YPR;
            // cout << iPosture << endl; // C.K.
            break;
        }
        case 4:
        {
            double q_t;
            q_t = iPosCmd(0) / TargetRad;
            if (time_current == 0.0){
                q_t = 1e-12;
            }
            qres = q_i.slerp(q_t, q_f);
            Vector3d eulerAngle = qres.matrix().eulerAngles(2,1,0); // YPR
            iPosture = eulerAngle;

            // if (time_current < 0.1) cout << iPosture.transpose() << endl;
            break;
        }
    }

    return iPosture;
}


/*
* @brief:
* Implement trajectory planning of 3D circle path
* 
* @param normal_vector:
* Normal vector of circle trajectory plane
*
* @param TargetRad:
* Central angle of planning circle. e.g., 2*pi is "one" turn of circles.
*
* @param posture_mode
* posture planning mode of robot end effector.
*   1. Posture rotate along constant axis
*   2. Shooting mode by assign depression angle
*   3. Constant normal plane direction posture
*   4. Spherical linear interpolation
* 
*/
void TrajectoryPlanning::Circle_Plan3D(Vector3d Center, double Radius, Vector3d normal_vector, double TargetRad, int posture_mode)
{

    if (time_current == 0)
    {

        /* Get robot initial position(xyz) */
        Vector3d initial_xyz = (kinematicsPtr->GetEndEffectorPose(current_pos)).head(3);

        /* ----- Define circle path ----- */
        v_normal = normal_vector / normal_vector.norm();
        Vector3d CP_vector = initial_xyz - Center;
        /* in eigen .dot is dot product */
        Vector3d CQ_vector = CP_vector - v_normal.dot(CP_vector) * v_normal;
        /* K is Shortest distance point from EEF in the circle path */
        Vector3d K = Center + Radius * CQ_vector / CQ_vector.norm();
        Vector3d a = K - Center;
        Vector3d a_normal = a / a.norm();
        Vector3d b_normal = v_normal.cross(a_normal);

        ab_matrix << a_normal, b_normal;

        /* ----- posture initial parameter setup ----- */
        switch (posture_mode)
        {
            /* ----- 1. Posture rotate along constant axis ----- */
            case 1:
            {
                R_initial = YPRToRotationMatrix(DEG2RAD(yaw), DEG2RAD(pitch), DEG2RAD(roll)); // check ok.
                break;
            }
            /* ----- 2. Shooting mode (Assign depression angle) ----- */
            case 2:
            {
                Matrix3d H = (Matrix3d() << -a_normal, b_normal, v_normal).finished();
                YPR = RotationMatrixToYPR(H); //CK

                Matrix3d R_m;
                double angle = 90 - depression_angle;
                angle = DEG2RAD(angle);
                R_m <<  cos(angle), 0, sin(angle),
                                 0, 1,          0,
                       -sin(angle), 0, cos(angle);
                R = YPRToRotationMatrix(YPR(0), YPR(1), YPR(2));
                R = R * R_m; //CK
                for (int i = 0; i < 9; i++){
                    if(abs(R(i))< 1e-10) R(i) = 0;
                }
                break;
            }
            /* ----- 3. Constant normal plane direction posture ----- */
            case 3:
            {
                Matrix3d H = (Matrix3d() << a_normal, b_normal, v_normal).finished();
                YPR = RotationMatrixToYPR(H);
                break;
            }
            /* ----- 4. Spherical Linear Interpolation ----- */
            case 4:
            {
                q_i = AngleAxisd(DEG2RAD(yaw_i), Vector3d::UnitZ()) * AngleAxisd(DEG2RAD(pitch_i), Vector3d::UnitY()) * AngleAxisd(DEG2RAD(roll_i), Vector3d::UnitX());
                q_f = AngleAxisd(DEG2RAD(yaw_f), Vector3d::UnitZ()) * AngleAxisd(DEG2RAD(pitch_f), Vector3d::UnitY()) * AngleAxisd(DEG2RAD(roll_f), Vector3d::UnitX());
                break;
            }
        }
    
    }

    // Get iPosCmd / iVelCmd / iAccCmd
    VectorXd Scurve_initial = (VectorXd(1) << 0).finished();
    VectorXd Scurve_target = (VectorXd(1) << TargetRad).finished();
    Scurve_sp(Scurve_initial, Scurve_target, 2); // C.K.

    
    VectorXd X(3), X_dot(3), X_dotdot(3), temp(2), temp2(2);
    temp << cos(iPosCmd(0)), sin(iPosCmd(0));
    X = Center + (Radius * ab_matrix) * temp; // C.K.

    temp << -sin(iPosCmd(0)) * iVelCmd(0), cos(iPosCmd(0)) * iVelCmd(0);
    X_dot = (Radius * ab_matrix) * temp; // C.K.

    temp << -sin(iPosCmd(0)) * iAccCmd.array(), cos(iPosCmd(0)) * iAccCmd.array();
    temp2 << cos(iPosCmd(0)) * pow(iVelCmd(0), 2), sin(iPosCmd(0)) * pow(iVelCmd(0), 2);
    X_dotdot = (Radius * ab_matrix) * temp - (Radius * ab_matrix) * temp2; // C.K.
    

    Vector3d iPosture;
    iPosture = Get_circle_posture(posture_mode , TargetRad);


    /* ----- joint trajectory -----*/
    CarPosCmd << X, iPosture; // CK.
    CarPosCmd_list.push_back(CarPosCmd);

    Matrix4d TMatrix;
    Matrix<double, 1, AXIS> XYZYPR_target;
    TMatrix = kinematicsPtr -> GetEndEffectorTransformationMatrix(CarPosCmd.transpose());
    XYZYPR_target = kinematicsPtr -> DecompositionXYZYPR(TMatrix);
    kinematicsPtr -> GetInverseKinematics(XYZYPR_target, currentJoint, JointPosCmd.transpose());
    if( isnan(JointPosCmd.array().sum()) ){
        printf("Trajectory pass through singular point ...\n");
        return;
    }
    currentJoint = JointPosCmd;
    // JointPosCmd_list.push_back(JointPosCmd);
    /* ----- joint trajectory -----*/


    /* ----- velocity trajectory -----*/
    if(time_current == 0.0){
        past_posture_pos = iPosture;
        pass_pos_diff << 0.001 , 0.001 , 0.001; // Initial pos diff.
    }

    Vector3d iPosture_modify;
    iPosture_modify = iPosture;
    for (int iaxis = 0; iaxis < 3; iaxis++)
    {
        double diff = 0;
        diff = iPosture_modify(iaxis) - past_posture_pos(iaxis);
        if(diff < -6.0){
            printf("t:%f , Posture position tauch positive limit\n" , time_current);
            iPosture_modify(iaxis) -= (diff - pass_pos_diff(iaxis) ); // diff is -
        }
        else if(diff > 6.0){
            printf("t:%f , Posture position tauch negative limit\n" , time_current);
            iPosture_modify(iaxis) -= (diff - pass_pos_diff(iaxis) ); // diff is +
        }
        pass_pos_diff(iaxis) = diff;
    }
    Vector3d iPosture_vel;
    iPosture_vel = (iPosture_modify - past_posture_pos) / SAMPLINGTIME;
    past_posture_pos = iPosture;
    CarVelCmd << X_dot, iPosture_vel; // xyz zyx
    CarVelCmd_list.push_back(CarVelCmd);

    Matrix<double, 6, AXIS> Jacobian;
    Matrix3d TfiM;
    Vector3d Omega;
    VectorXd x_dot(6);

    Jacobian = kinematicsPtr->GetRobotJacobian(JointPosCmd.transpose());
    TfiM = GetTfiMatrix(CarPosCmd(3), CarPosCmd(4), CarPosCmd(5));
    Omega = TfiM * CarVelCmd.tail(3);
    x_dot << CarVelCmd.head(3), Omega;
    JointVelCmd = Jacobian.inverse() * x_dot;
    JointVelCmd_list.push_back(JointVelCmd);
    /* ----- velocity trajectory -----*/


    /* ----- Acceleration trajectory -----*/
    if(time_current == 0.0){
        past_posture_vel = iPosture_vel;
        pass_vel_diff << 0.001 , 0.001 , 0.001; // Initial vel diff.
    }

    Vector3d iPosture_vel_modify;
    iPosture_vel_modify = iPosture_vel;
    for (int iaxis = 0; iaxis < 3; iaxis++)
    {
        double diff = 0;
        diff = iPosture_vel_modify(iaxis) - past_posture_vel(iaxis);
        if(diff < -6.0){
            printf("t:%f , Posture velocity tauch positive limit\n" , time_current);
            iPosture_vel_modify(iaxis) -= (diff - pass_vel_diff(iaxis) ); // diff is -
        }
        else if(diff > 6.0){
            printf("t:%f , Posture velocity tauch negative limit\n" , time_current);
            iPosture_vel_modify(iaxis) -= (diff - pass_vel_diff(iaxis) ); // diff is +
        }
        pass_vel_diff(iaxis) = diff;
    }
    Vector3d iPosture_acc;
    iPosture_acc = (iPosture_vel_modify - past_posture_vel) / SAMPLINGTIME;
    past_posture_vel = iPosture_vel;
    CarAccCmd << X_dotdot, iPosture_acc;
    CarAccCmd_list.push_back(CarAccCmd);

    Matrix3d TfiDotM;
    Vector3d OmegaDot;
    VectorXd x_dotdot(6);
    Matrix<double, 6, AXIS> jacobianDot;

    Jacobian = kinematicsPtr->GetRobotJacobian(JointPosCmd.transpose());
    TfiM = GetTfiMatrix(CarPosCmd(3), CarPosCmd(4), CarPosCmd(5));
    TfiDotM = GetTfiMatrixDot(CarPosCmd(3), CarPosCmd(4), CarPosCmd(5), CarVelCmd(3), CarVelCmd(4), CarVelCmd(5));
    OmegaDot = TfiM * CarAccCmd.tail(3) + TfiDotM * JointVelCmd.tail(3);
    x_dotdot << CarAccCmd.head(3), OmegaDot;
    jacobianDot = kinematicsPtr->GetRobotJacobianDot(JointPosCmd.transpose(), JointVelCmd.transpose());
    JointAccCmd = Jacobian.inverse() * (x_dotdot - (jacobianDot * JointVelCmd));
    JointAccCmd_list.push_back(JointAccCmd);
    /* ----- Acceleration trajectory -----*/


    /* ----- modify data for discontinuous joint position cmd ------*/
    if(time_current == 0.0){
        past_joint456_cmd = JointPosCmd.tail(3);
        pass_jointcmd_diff << 0.01 , 0.01 , 0.01;
        sum_joint456_diff << 0.0, 0.0, 0.0;
    }
    for (int iaxis = 0; iaxis < 3; iaxis++)
    {
        double diff = 0;
        diff = JointPosCmd(iaxis+3) - past_joint456_cmd(iaxis);
        if(diff < -6.0){
            printf("t:%f , Joint cmd tauch positive limit\n" , time_current);
            sum_joint456_diff(iaxis) += (diff - pass_jointcmd_diff(iaxis));
        }
        else if(diff > 6.0){
            printf("t:%f , Joint cmd tauch negative limit\n" , time_current);
            sum_joint456_diff(iaxis) += (diff - pass_jointcmd_diff(iaxis));
        }
    }
    past_joint456_cmd = JointPosCmd.tail(3);
    for (int iaxis = 0; iaxis < 3; iaxis++){
        JointPosCmd(iaxis+3) -= sum_joint456_diff(iaxis);
    }
    JointPosCmd_list.push_back(JointPosCmd);
    /* ----- modify data for discontinuous joint position cmd ------*/


    /* function end flag */
    if (time_current > t7){
        Function_EndFlag = true;
    }
    /* function end flag */
}

void TrajectoryPlanning::PTPLine_Plan(Matrix<double,6,1> InitialPosition , Matrix<double,6,1> TargetPosition){

    Scurve_sp(InitialPosition, TargetPosition, 1);
    CarPosCmd = iPosCmd;
    CarVelCmd = iVelCmd;
    CarAccCmd = iAccCmd;

    /* ----- joint trajectory -----*/
    CarPosCmd_list.push_back(CarPosCmd);

    Matrix4d TMatrix;
    Matrix<double, 1, AXIS> XYZYPR_target;
    TMatrix = kinematicsPtr -> GetEndEffectorTransformationMatrix(CarPosCmd.transpose());
    XYZYPR_target = kinematicsPtr -> DecompositionXYZYPR(TMatrix);
    kinematicsPtr -> GetInverseKinematics(XYZYPR_target, currentJoint, JointPosCmd.transpose());
    if( isnan(JointPosCmd.array().sum()) ){
        printf("Trajectory pass through singular point ...\n");
        return;
    }
    currentJoint = JointPosCmd;
    // JointPosCmd_list.push_back(JointPosCmd);
    /* ----- joint trajectory -----*/


    /* ----- velocity trajectory -----*/
    CarVelCmd_list.push_back(CarVelCmd);

    Matrix<double, 6, AXIS> Jacobian;
    Matrix3d TfiM;
    Vector3d Omega;
    VectorXd x_dot(6);

    Jacobian = kinematicsPtr->GetRobotJacobian(JointPosCmd.transpose());
    TfiM = GetTfiMatrix(CarPosCmd(3), CarPosCmd(4), CarPosCmd(5));
    Omega = TfiM * CarVelCmd.tail(3);
    x_dot << CarVelCmd.head(3), Omega;
    JointVelCmd = Jacobian.inverse() * x_dot;

    JointVelCmd_list.push_back(JointVelCmd);
    /* ----- velocity trajectory -----*/


    /* ----- Acceleration trajectory -----*/
    CarAccCmd_list.push_back(CarAccCmd);

    Matrix3d TfiDotM;
    Vector3d OmegaDot;
    VectorXd x_dotdot(6);
    Matrix<double, 6, AXIS> jacobianDot;

    Jacobian = kinematicsPtr->GetRobotJacobian(JointPosCmd.transpose());
    TfiM = GetTfiMatrix(CarPosCmd(3), CarPosCmd(4), CarPosCmd(5));
    TfiDotM = GetTfiMatrixDot(CarPosCmd(3), CarPosCmd(4), CarPosCmd(5), CarVelCmd(3), CarVelCmd(4), CarVelCmd(5));
    OmegaDot = TfiM * CarAccCmd.tail(3) + TfiDotM * JointVelCmd.tail(3);
    x_dotdot << CarAccCmd.head(3), OmegaDot;
    jacobianDot = kinematicsPtr->GetRobotJacobianDot(JointPosCmd.transpose(), JointVelCmd.transpose());
    JointAccCmd = Jacobian.inverse() * (x_dotdot - (jacobianDot * JointVelCmd));

    JointAccCmd_list.push_back(JointAccCmd);
    /* ----- Acceleration trajectory -----*/


    /* ----- modify data for discontinuous joint position cmd ------*/
    if(time_current == 0.0){
        past_joint456_cmd = JointPosCmd.tail(3);
        pass_jointcmd_diff << 0.01 , 0.01 , 0.01;
        sum_joint456_diff << 0.0, 0.0, 0.0;
    }
    for (int iaxis = 0; iaxis < 3; iaxis++)
    {
        double diff = 0;
        diff = JointPosCmd(iaxis+3) - past_joint456_cmd(iaxis);
        if(diff < -6.0){
            printf("t:%f , Joint cmd tauch positive limit\n" , time_current);
            sum_joint456_diff(iaxis) += (diff - pass_jointcmd_diff(iaxis));
        }
        else if(diff > 6.0){
            printf("t:%f , Joint cmd tauch negative limit\n" , time_current);
            sum_joint456_diff(iaxis) += (diff - pass_jointcmd_diff(iaxis));
        }
    }
    past_joint456_cmd = JointPosCmd.tail(3);
    for (int iaxis = 0; iaxis < 3; iaxis++){
        JointPosCmd(iaxis+3) -= sum_joint456_diff(iaxis);
    }
    JointPosCmd_list.push_back(JointPosCmd);
    /* ----- modify data for discontinuous joint position cmd ------*/


    /* function end flag */
    if (time_current > t7){
        Function_EndFlag = true;
    }
    /* function end flag */
}

void TrajectoryPlanning::PTPPlan(Matrix<double,6,1> InitialJoints , Matrix<double,6,1> TargetJoints){

    Scurve_sp(InitialJoints, TargetJoints, 1);

    JointPosCmd_list.push_back(iPosCmd);
    JointVelCmd_list.push_back(iVelCmd);
    JointAccCmd_list.push_back(iAccCmd);

    /* function end flag */
    if (time_current > t7){
        Function_EndFlag = true;
    }
    /* function end flag */

}

void TrajectoryPlanning::save_trajectory_data(){

    ofstream ofs;
    ofs.open("t.txt");
    for (auto t : TBase_list){
        ofs << t << "\n";
    }
    ofs.close();

    for (int i_RecordData = 0; i_RecordData < 6; i_RecordData++)
    {
        ofs.open(record_data_name[i_RecordData]);
        for (auto n : record_data_list[i_RecordData]){
            for (int idata = 0; idata < AXIS; idata++){
                ofs << n(idata);
                if (idata < AXIS-1) ofs << "\t";
            }
            ofs << "\n";
        }
        ofs.close();
    }

}


Matrix3d TrajectoryPlanning::RotationVector(Matrix3d RotationM, Vector3d Axis)
{
    double a,b,c,d;
    Matrix3d R;
    a = cos(iPosCmd(0)/2);
    b = sin(iPosCmd(0)/2) * Axis(0);
    c = sin(iPosCmd(0)/2) * Axis(1);
    d = sin(iPosCmd(0)/2) * Axis(2);
    R <<    1-2*c*c-2*d*d,  2*b*c-2*a*d,    2*a*c+2*b*d,
            2*b*c+2*a*d,    1-2*b*b-2*d*d,  2*c*d-2*a*b,
            2*b*d-2*a*c,    2*a*b+2*c*d,    1-2*b*b-2*c*c;
    
    return R * RotationM;
}

Matrix3d TrajectoryPlanning::GetTfiMatrix(double pit, double yaw, double roll){
    //  ZYX Euler Angler Rotate: Z = pit, Y = yaw, X = roll
    //  remapping consquence rotation back to base frame rotate
    Matrix3d T;
    T <<  0, -sin(pit)  ,cos(pit) * cos(yaw),
          0, cos(pit)   ,sin(pit) * cos(yaw),
          1, 0          ,-sin(yaw);

    return T;
}

Matrix3d TrajectoryPlanning::GetTfiMatrixDot(double pit, double yaw, double roll, double pitDot, double yawDot, double rollDot){
    // ZYX Euler Angler Rotate: Z = pit, Y = yaw, X = roll
    // remapping consquence rotation back to base frame rotate
    Matrix3d T;
    T << 0  ,-cos(pit)*pitDot   , -sin(pit)*cos(yaw)*pitDot - cos(pit)*sin(yaw)*yawDot,
         0  ,-sin(pit)*pitDot   ,  cos(pit)*cos(yaw)*pitDot - sin(pit)*sin(yaw)*yawDot,
         0  ,0                  , -cos(yaw)*yawDot;

    return T;
}