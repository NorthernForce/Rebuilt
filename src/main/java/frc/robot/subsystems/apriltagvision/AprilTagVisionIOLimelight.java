package frc.robot.subsystems.apriltagvision;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.estimator.PoseEstimator;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.apriltagvision.AprilTagVision.Pose2dWithTimestamp;

/**
 * This class is used to interface with Limelight for AprilTag detection. It
 * uses the LimelightHelpers library to get the pose of the robot relative to
 * the AprilTag.
 */
public class AprilTagVisionIOLimelight implements AprilTagVision.AprilTagVisionIO
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

    public void setHeading(double heading, double yawRate, double pitch, double pitchRate, double roll, double rollRate)
    {
        LimelightHelpers.SetRobotOrientation("limelight", heading, yawRate, pitch, pitchRate, roll, rollRate);

    }

    /**
     * Get the list of poses of the robot relative to the AprilTag.
     */
    @Override
    public List<Pose2dWithTimestamp> getPose()
    {
        var poseData = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (LimelightHelpers.validPoseEstimate(poseData)) // seems to be the way to check if the camera sees a tag, thus
                                                          // if the
            // pose is present.
            return List.of();
        return List.of(new Pose2dWithTimestamp(poseData.pose, poseData.timestampSeconds));
    }
}
