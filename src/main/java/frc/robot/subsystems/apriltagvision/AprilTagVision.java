package frc.robot.subsystems.apriltagvision;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagVision extends SubsystemBase
{
    public AprilTagVision()
    {

    }

    /**
     * Interface for the AprilTagVision subsystem with simulation/Limelight
     */
    public static interface AprilTagVisionIO
    {
        public List<Pose2dWithTimestamp> getPose();
    }

    /**
     * A simple record to hold the pose and timestamp of the AprilTag
     */
    public record Pose2dWithTimestamp(Pose2d pose, double timestamp) {
    }
}
