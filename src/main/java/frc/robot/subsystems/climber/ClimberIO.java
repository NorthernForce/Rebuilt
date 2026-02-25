package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.lobby.LobbyConstants.ClimberConstants.ClimbLevels;

public interface ClimberIO
{
    public void runUp();

    public void homeDown();

    public ClimbLevels getLevel();

    public boolean atBottom();

    public boolean atTop();

    public void stopMotor();

    public double getRotations();

    public default Pose2d getNearestPreclimbPosition(Pose2d robotPose)
    {
        return robotPose;
    }

    public default void setHookPosition(double position)
    {
    }

    public default void updateSimulation()
    {
    }
}
