package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * @param pose      The pose to store
 * @param timestamp The timestamp of the recorded pose This class is used to
 *                  store the pose of the robot relative to the blue origin and
 *                  timestamp.
 */
public record Pose2dWithTimestamp(Pose2d pose, double timestamp) {

}