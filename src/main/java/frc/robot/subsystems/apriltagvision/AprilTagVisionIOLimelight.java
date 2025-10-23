package frc.robot.subsystems.apriltagvision;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.LimelightHelpers;

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
     * @param position      The position of the Limelight camera on the robot
     *                      relative to the center.
     * @param validIds      An array of valid AprilTag IDs to filter.
     */
    public AprilTagVisionIOLimelight(String limelightName, Transform3d robotToCamera, int[] validIds)
    {
        this.limelightName = limelightName;
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, validIds);
        LimelightHelpers.setCameraPose_RobotSpace(limelightName, robotToCamera.getY(), robotToCamera.getX(),
                robotToCamera.getZ(), 0, 0, 0);
    }

    /**
     * Constructor for the AprilTagVisionIOLimelight class.
     * 
     * @param limelightName The name of the Limelight camera to use for AprilTag
     *                      detection.
     * @param position      The position of the Limelight camera on the robot
     */

    public AprilTagVisionIOLimelight(String limelightName, Transform3d robotToCamera)
    {
        this.limelightName = limelightName;
        LimelightHelpers.setCameraPose_RobotSpace(limelightName, robotToCamera.getY(), robotToCamera.getX(),
                robotToCamera.getZ(), 0, 0, 0);
    }

    /**
     * Set the heading of the robot to the Limelight camera.
     * 
     * @param heading The heading of the robot in degrees.
     */
    @Override
    public void setHeading(Rotation2d heading, Rotation2d yawRate)
    {
        LimelightHelpers.SetRobotOrientation(limelightName, heading.getDegrees(), yawRate.getDegrees(), 0, 0, 0, 0);
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
