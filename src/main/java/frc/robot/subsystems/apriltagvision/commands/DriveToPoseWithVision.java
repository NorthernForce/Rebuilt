package frc.robot.subsystems.apriltagvision.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * A command factory that creates drive-to-pose commands.
 * 
 * NOTE: this will be expanded upon to auto align to shoot to goal.
 */
public class DriveToPoseWithVision
{
    private final CommandSwerveDrivetrain drive;

    /**
     * Constructs a DriveToPoseWithVision command factory.
     *
     * @param drive The swerve drivetrain subsystem
     */
    public DriveToPoseWithVision(CommandSwerveDrivetrain drive)
    {
        this.drive = drive;
    }

    /**
     * Creates a command that drives to the target pose.
     *
     * @param targetPose The target pose to navigate to (in field coordinates)
     * @return A command that drives to the pose
     */
    public Command driveToPose(Pose2d targetPose)
    {
        return drive.navigateToPose(targetPose);
    }

    /**
     * Creates a command that drives to the target pose with a specific heading.
     *
     * @param targetPose    The target pose to navigate to (in field coordinates)
     * @param targetHeading The heading to face at the target
     * @return A command that drives to the pose
     */
    public Command driveToPoseWithHeading(Pose2d targetPose, Rotation2d targetHeading)
    {
        Pose2d poseWithHeading = new Pose2d(targetPose.getTranslation(), targetHeading);
        return driveToPose(poseWithHeading);
    }
}
