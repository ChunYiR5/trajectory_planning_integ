#include "Trajectory_Planning.h"
#include <iostream>

int main()
{

    auto robot_model_node = std::make_shared<RobotModel>();
    auto constraint_node = std::make_shared<Constraint>(robot_model_node);
    auto kinematics_node = std::make_shared<IDHKinematics>(robot_model_node->DH_Parameter, "Industrial");
    auto trajectory_planning_node = std::make_shared<TrajectoryPlanning>(kinematics_node, constraint_node);

    trajectory_planning_node->Print_instruction();

    while (true)
    {

        trajectory_planning_node->Initialization();

        printf("Waiting for use to get keyboard intput....\n");
        int trajectory_mode;
        cin >> trajectory_mode;

        switch (trajectory_mode)
        {

        case 1: // PTP mode
        {
            Matrix<double, 6, 1> InitialJoints, TargetJoints;
            InitialJoints << MatrixXd::Zero(AXIS, 1);
            TargetJoints << DEG2RAD(20), DEG2RAD(20), DEG2RAD(20), DEG2RAD(20), DEG2RAD(20), DEG2RAD(20);

            while (!trajectory_planning_node->Function_EndFlag)
            {
                trajectory_planning_node->PTPPlan(InitialJoints, TargetJoints);
                trajectory_planning_node->time_current += SAMPLINGTIME;
            }
        }
        break;
        case 2: // Line mode
        {
            Matrix<double, 6, 1> InitialPosition, TargetPosition;
            InitialPosition << 500, -200, 420, PI, PI, -DEG2RAD(30); // xyz yaw(z) pitch(y) roll(x)
            TargetPosition << 500, 200, 420, PI, PI, DEG2RAD(30); // xyz yaw(z) pitch(y) roll(x)

            while (!trajectory_planning_node->Function_EndFlag)
            {
                trajectory_planning_node->PTPPlan(InitialPosition, TargetPosition);
                trajectory_planning_node->time_current += SAMPLINGTIME;
            }
        }
        break;
        case 3: // Circle mode
        {
            Vector3d Center = (Vector3d(3) << 0, 560, 580).finished();
            double R = 80;
            Vector3d normal_vector = (Vector3d(3) << 0, 0, -1).finished();
            double TargetRad = 2 * PI;
            int posture_mode = 1;

            while (!trajectory_planning_node->Function_EndFlag)
            {
                trajectory_planning_node->Circle_Plan3D(Center, R, normal_vector, TargetRad, posture_mode);
                trajectory_planning_node->time_current += SAMPLINGTIME;
            }
        }
        break;

        case 9: // Cancel mode
            return 0;
            break;
        default:
            return 0;
            break;

        }
        trajectory_planning_node -> save_trajectory_data();
    }

    cout << "END of main" << endl;
    return 0;
}