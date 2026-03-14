package frc.robot.lobby.subsystems.climber;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.lobby.LobbyConstants.ClimberConstants.ClimbLevels;

public interface ClimberIO
{
    public void runUp();

    public void homeDown();

    public void stopMotor();

    public double getRotations();

    public boolean atTop();

    public boolean atBottom();

    public default Pose2d getNearestPreclimbPosition(Pose2d robotPose)
    {
        return robotPose;
    }

    public default void updateSimulation()
    {
    }
}
