package frc.robot.zippy;

import edu.wpi.first.math.geometry.Pose2d;

public class ZippyConstants
{
    public record Pose2dWithTimestamp(Pose2d pose, double timestamp) {

    }
}
