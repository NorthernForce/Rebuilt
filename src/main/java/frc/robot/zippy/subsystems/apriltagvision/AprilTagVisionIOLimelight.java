package frc.robot.zippy.subsystems.apriltagvision;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.LimelightHelpers;
import frc.robot.zippy.ZippyConstants;

/**
 * This class is used to interface with Limelight for AprilTag detection. It
 * uses the LimelightHelpers library to get the pose of the robot relative to
 * the blue origin.
 */
public class AprilTagVisionIOLimelight implements AprilTagVisionIO
{
    /**
     * The name of the Limelight camera to use for AprilTag detection.
     */
    private final String limelightName;

    /**
     * Constructor for the AprilTagVisionIOLimelight class.
     * 
     * @param limelightName The name of the Limelight camera to use for AprilTag
     *                      detection.
     * @param validIds      An array of valid AprilTag IDs to filter.
     */
    public AprilTagVisionIOLimelight(String limelightName, int[] validIds)
    {
        this.limelightName = limelightName;
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, validIds);

    }

    /**
     * Constructor for the AprilTagVisionIOLimelight class.
     * 
     * @param limelightName The name of the Limelight camera to use for AprilTag
     *                      detection.
     */

    public AprilTagVisionIOLimelight(String limelightName)
    {
        this.limelightName = limelightName;
    }

    /**
     * Set the heading of the robot to the Limelight camera.
     * 
     * @param heading The heading of the robot in degrees.
     */
    @Override
    public void setHeading(Rotation2d heading, Rotation2d yawRate)
    {
        LimelightHelpers.SetRobotOrientation(ZippyConstants.VisionConstants.LimeLightConstants.kLimeLightName,
                heading.getDegrees(), yawRate.getDegrees(), 0, 0, 0, 0);
    }

    /**
     * Get the list of poses of the robot relative to the blue origin.
     */
    @Override
    public List<Pose2dWithTimestamp> getPoses()
    {
        var poseData = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (LimelightHelpers.validPoseEstimate(poseData))
            return List.of();
        return List.of(new Pose2dWithTimestamp(poseData.pose, poseData.timestampSeconds));
    }
}
