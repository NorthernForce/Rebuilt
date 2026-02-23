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
     * @param robotToCamera The position of the Limelight camera on the robot, when
     *                      viewed from above (top-down perspective) -
     *                      forwards/backwards is y, right/left is x, up/down is z
     *                      relative to the center.
     * @param validIds      An array of valid AprilTag IDs to filter.
     */
    public AprilTagVisionIOLimelight(String limelightName, Transform3d robotToCamera, int[] validIds)
    {
        this.limelightName = limelightName;
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, validIds);
        // setCameraPose_RobotSpace parameters: forward, right, up, roll, pitch, yaw
        // (all in meters/degrees)
        LimelightHelpers.setCameraPose_RobotSpace(limelightName, robotToCamera.getY(), // forward
                robotToCamera.getX(), // right
                robotToCamera.getZ(), // up
                Math.toDegrees(robotToCamera.getRotation().getX()), // roll
                Math.toDegrees(robotToCamera.getRotation().getY()), // pitch
                Math.toDegrees(robotToCamera.getRotation().getZ())); // yaw
    }

    /**
     * Constructor for the AprilTagVisionIOLimelight class.
     * 
     * @param limelightName The name of the Limelight camera to use for AprilTag
     *                      detection.
     * @param robotToCamera The position of the Limelight camera on the robot -
     *                      forwards/backwards is y, right/left is x, up/down is z
     */

    public AprilTagVisionIOLimelight(String limelightName, Transform3d robotToCamera)
    {
        this.limelightName = limelightName;

        // setCameraPose_RobotSpace parameters: forward, right, up, roll, pitch, yaw
        // (all in meters/degrees)
        LimelightHelpers.setCameraPose_RobotSpace(limelightName, robotToCamera.getY(), // forward
                robotToCamera.getX(), // right
                robotToCamera.getZ(), // up
                Math.toDegrees(robotToCamera.getRotation().getX()), // roll
                Math.toDegrees(robotToCamera.getRotation().getY()), // pitch
                Math.toDegrees(robotToCamera.getRotation().getZ())); // yaw
    }

    /**
     * Set the heading of the robot to the Limelight camera.
     * 
     * @param heading The heading of the robot (WPILib blue alliance convention)
     * @param yawRate The yaw rate of the robot
     */
    @Override
    public void setHeading(Rotation2d heading, Rotation2d yawRate)
    {
        // WPILib blue alliance: 0° = facing red alliance wall (positive X)
        // Limelight: 0° = facing red alliance wall
        // These should match, so pass heading directly
        LimelightHelpers.SetRobotOrientation(limelightName, heading.getDegrees(), yawRate.getDegrees(), 0, 0, 0, 0);
    }

    /**
     * Get the list of poses of the robot relative to the blue origin.
     */
    @Override
    public List<Pose2dWithTimestamp> getPoses()
    {
        var poseData = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (!LimelightHelpers.validPoseEstimate(poseData))
            return List.of();
        return List.of(new Pose2dWithTimestamp(poseData.pose, poseData.timestampSeconds));
    }
}
