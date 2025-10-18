package frc.robot.ralph;

import org.northernforce.util.NFRRobotContainer;
import org.photonvision.simulation.SimCameraProperties;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ralph.generated.RalphTunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIO;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOLimelight;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOPhotonVisionSim;
import frc.robot.util.AutoUtil;

public class RalphContainer implements NFRRobotContainer
{
    private final CommandSwerveDrivetrain drive;
    private final AprilTagVisionIO vision;
    private final Field2d field;

    public RalphContainer()
    {
        drive = new CommandSwerveDrivetrain(RalphTunerConstants.DrivetrainConstants,
                RalphConstants.DrivetrainConstants.kMaxSpeed, RalphConstants.DrivetrainConstants.kMaxAngularSpeed,
                RalphTunerConstants.FrontLeft, RalphTunerConstants.FrontRight, RalphTunerConstants.BackLeft,
                RalphTunerConstants.BackRight);
        if (Utils.isSimulation())
        {
            // TODO: actually get camera properties + transform
            vision = new AprilTagVisionIOPhotonVisionSim(
                    RalphConstants.VisionConstants.LimeLightConstants.kLimeLightName, new SimCameraProperties(),
                    new Transform3d());
        } else
        {
            vision = new AprilTagVisionIOLimelight(RalphConstants.VisionConstants.LimeLightConstants.kLimeLightName,
                    RalphConstants.VisionConstants.LimeLightConstants.kValidIds);
        }
        field = new Field2d();
        Shuffleboard.getTab("Developer").add(field);
        Shuffleboard.getTab("Developer").add("Reset Encoders", drive.resetEncoders());
        Shuffleboard.getTab("Developer").add("Reset Orientation", drive.resetOrientation());
        Shuffleboard.getTab("Developer").add("Drive to Blue Reef",
                drive.navigateToPose(new Pose2d(3, 4, new Rotation2d())));
        AutoUtil.buildAutos();
    }

    /**
     * gets the drive subsystem
     *
     * @return the drive subsystem
     */
    public CommandSwerveDrivetrain getDrive()
    {
        return drive;
    }

    @Override
    public void periodic()
    {
        vision.getPoses().forEach(m -> drive.addVisionMeasurement(m.pose(), m.timestamp()));
        field.setRobotPose(drive.getState().Pose);
    }

    @Override
    public void bindOI()
    {
        new RalphOI().bind(this);
    }

    @Override
    public Command getAutonomousCommand()
    {
        return AutoUtil.getSelected();
    }

}
