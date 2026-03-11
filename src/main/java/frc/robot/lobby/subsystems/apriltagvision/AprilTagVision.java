package frc.robot.lobby.subsystems.apriltagvision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
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
    private final AprilTagVisionIO[] visions;

    public AprilTagVision(CommandSwerveDrivetrain drive, AprilTagVisionIO... visions)
    {
        this.drive = drive;
        this.visions = visions;
    }

    public void setHeading(Rotation2d heading, Rotation2d yawRate)
    {
        for (AprilTagVisionIO vision : visions)
        {
            vision.setHeading(heading, yawRate);
        }
    }

    public List<Pose2dWithTimestamp> getPoses()
    {
        List<Pose2dWithTimestamp> poses = new ArrayList<Pose2dWithTimestamp>();
        for (AprilTagVisionIO vision : visions)
        {
            for (Pose2dWithTimestamp pose : vision.getPoses())
            {
                poses.add(pose);
            }
        }

        return poses;
    }

    @Override
    public void periodic()
    {
        var state = drive.getState();
        Rotation2d currentHeading = state.Pose.getRotation();
        Rotation2d yawRate = Rotation2d.fromRadians(state.Speeds.omegaRadiansPerSecond);

        for (AprilTagVisionIO vision : visions)
        {
            vision.setHeading(currentHeading, yawRate);
        }
    }
}
