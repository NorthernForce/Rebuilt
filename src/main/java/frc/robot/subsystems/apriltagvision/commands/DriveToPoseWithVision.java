package frc.robot.subsystems.apriltagvision.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIO;

/**
 * A command factory that creates drive-to-pose commands with vision
 * measurement. This class provides methods to create commands that navigate to
 * a target pose while continuously updating odometry with Limelight vision
 * measurements.
 */
public class DriveToPoseWithVision
{
    private final CommandSwerveDrivetrain drive;
    private final AprilTagVisionIO vision;

    /**
     * Constructs a DriveToPoseWithVision command factory.
     *
     * @param drive  The swerve drivetrain subsystem
     * @param vision The AprilTag vision IO interface (e.g., Limelight)
     */
    public DriveToPoseWithVision(CommandSwerveDrivetrain drive, AprilTagVisionIO vision)
    {
        this.drive = drive;
        this.vision = vision;
    }

    /**
     * Creates a command that drives to the target pose while continuously updating
     * odometry with vision measurements from the Limelight.
     *
     * @param targetPose The target pose to navigate to (in field coordinates)
     * @return A command that drives to the pose while updating odometry with vision
     */
    public Command driveToPose(Pose2d targetPose)
    {
        return Commands.parallel(Commands.run(() -> updateOdometryWithVision()), drive.navigateToPose(targetPose));
    }

    /**
     * Creates a command that drives to the target pose while continuously updating
     * odometry with vision measurements. This version allows specifying a heading
     * to face while driving.
     *
     * @param targetPose    The target pose to navigate to (in field coordinates)
     * @param targetHeading The heading to face at the target
     * @return A command that drives to the pose while updating odometry with vision
     */
    public Command driveToPoseWithHeading(Pose2d targetPose, Rotation2d targetHeading)
    {
        Pose2d poseWithHeading = new Pose2d(targetPose.getTranslation(), targetHeading);
        return driveToPose(poseWithHeading);
    }

    /**
     * Updates the drivetrain odometry with vision measurements from the Limelight.
     * This method: 1. Sets the current robot heading to the Limelight for MegaTag2
     * 2. Gets valid pose estimates from the Limelight 3. Adds each valid
     * measurement to the drivetrain's pose estimator
     */
    private void updateOdometryWithVision()
    {
        var state = drive.getState();
        Rotation2d currentHeading = state.Pose.getRotation();
        Rotation2d yawRate = Rotation2d.fromRadians(state.Speeds.omegaRadiansPerSecond);

        vision.setHeading(currentHeading, yawRate);

        vision.getPoses().forEach(poseWithTimestamp ->
        {
            drive.addVisionMeasurement(poseWithTimestamp.pose(), poseWithTimestamp.timestamp());
        });
    }

    /**
     * Creates a command that only updates odometry with vision measurements without
     * driving. Useful for running alongside other drive commands or for calibration
     * purposes.
     *
     * @return A command that continuously updates odometry with vision measurements
     */
    public Command updateVisionOnly()
    {
        return Commands.run(() -> updateOdometryWithVision());
    }
}
