package frc.robot.subsystems.apriltagvision;
import java.util.List;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.apriltagvision.AprilTagVision.Pose2dWithTimestamp;

public class AprilTagVisionIOLimelight implements AprilTagVision.AprilTagVisionIO
{
    private final String limelightName;

    public AprilTagVisionIOLimelight(String limelightName, int[] validIds)
    {
        this.limelightName = limelightName;
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, validIds);
    }

    @Override
    public List<Pose2dWithTimestamp> getPose()
    {
        var poseData = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (poseData.pose == null)
            return null;
        return List.of(new Pose2dWithTimestamp(poseData.pose, poseData.timestampSeconds));
    }
}
