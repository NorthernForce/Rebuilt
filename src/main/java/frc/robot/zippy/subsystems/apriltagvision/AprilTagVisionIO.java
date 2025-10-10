package frc.robot.zippy.subsystems.apriltagvision;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * This interface is used to define the methods for interfacing with AprilTag
 * detection systems.
 */

public interface AprilTagVisionIO
{
    /**
     * Set the heading of the robot to the AprilTag camera.
     * 
     * @param heading The heading of the robot in degrees.
     * @param yawRate The yaw rate of the robot in degrees.
     */
    public default void setHeading(Rotation2d heading, Rotation2d yawRate)
    {
    }

    public List<Pose2dWithTimestamp> getPoses();
}
