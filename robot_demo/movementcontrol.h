#ifndef MOVEMENTCONTROL_H
#define MOVEMENTCONTROL_H

struct POSITION{
    float x;
    float y;
};

class MovementControl
{
public:
    MovementControl(float dt);
    ~MovementControl();
    void setNewPoint(float speed, POSITION pose);
    void updatePose(float pose_change, float angle_change);

private:
    //variablese
    float dt;
    POSITION irob_current_pose;
    POSITION irob_desired_pose;

    //methods
    float pidControlTranslation(float actual_pose, float desired_translation);
    float pidControlRotation(float actual_angle, float desired_angle);
    float comuteAngle();
    float comuteTranslation();
};


#endif // MOVEMENTCONTROL_H
