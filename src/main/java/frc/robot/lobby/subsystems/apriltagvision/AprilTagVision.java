package frc.robot.lobby.subsystems.apriltagvision;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lobby.subsystems.CommandSwerveDrivetrain;

/**
 * Subsystem that continuously updates drivetrain odometry with AprilTag vision
 * measurements.
 */
public class AprilTagVision extends SubsystemBase
{
    private final CommandSwerveDrivetrain drive;
    private final AprilTagVisionIO vision;

    public AprilTagVision(CommandSwerveDrivetrain drive, AprilTagVisionIO vision)
    {
        this.drive = drive;
        this.vision = vision;
    }

    public void setHeading(Rotation2d heading, Rotation2d yawRate)
    {
        vision.setHeading(heading, yawRate);
    }

    public List<Pose2dWithTimestamp> getPoses()
    {
        return vision.getPoses();
    }

    @Override
    public void periodic()
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
}
