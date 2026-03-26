package frc.robot.lobby.subsystems.climber;

import edu.wpi.first.math.geometry.Pose2d;

public interface ClimberIO
{
    public void runUp();

    public void homeDown();

    public void stopMotor();

    public double getRotations();

    public default Pose2d getNearestPreclimbPosition(Pose2d robotPose)
    {
        return robotPose;
    }

    public default void updateSimulation()
    {
    }
}
