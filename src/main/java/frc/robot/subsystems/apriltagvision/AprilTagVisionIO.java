package frc.robot.subsystems.apriltagvision;

import java.util.List;

public interface AprilTagVisionIO
{
    public default void setHeading(double heading, double yawRate)
    {
    }

    public List<Pose2dWithTimestamp> getPose();
}
