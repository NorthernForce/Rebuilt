package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * This class is used to store the pose of the robot relative to the AprilTag
 * and timestamp.
 */
public record Pose2dWithTimestamp(Pose2d pose, double timestamp) {

}