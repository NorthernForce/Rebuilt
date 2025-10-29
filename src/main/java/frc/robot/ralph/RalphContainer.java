package frc.robot.ralph;

import org.northernforce.util.NFRRobotContainer;
import org.photonvision.simulation.SimCameraProperties;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ralph.RalphConstants.ClimberConstants;
import frc.robot.ralph.generated.RalphTunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIO;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOLimelight;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOPhotonVisionSim;
import frc.robot.subsystems.climber.Climber;
import frc.robot.util.AutoUtil;

public class RalphContainer implements NFRRobotContainer
{
    private final CommandSwerveDrivetrain drive;
    private final AprilTagVisionIO vision;
    private final Field2d field;
    private final Climber climber = new Climber(ClimberConstants.kId, ClimberConstants.kClimbSpeed,
            ClimberConstants.kInverted, ClimberConstants.kGearRatio, ClimberConstants.kLowerLimit,
            ClimberConstants.kUpperLimit);

    public RalphContainer()
    {
        drive = new CommandSwerveDrivetrain(RalphTunerConstants.DrivetrainConstants,
                RalphConstants.DrivetrainConstants.kMaxSpeed, RalphConstants.DrivetrainConstants.kMaxAngularSpeed,
                RalphTunerConstants.FrontLeft, RalphTunerConstants.FrontRight, RalphTunerConstants.BackLeft,
                RalphTunerConstants.BackRight);
        if (Utils.isSimulation())
        {
            // TODO: get camera json config for sim
            vision = new AprilTagVisionIOPhotonVisionSim(
                    RalphConstants.VisionConstants.LimeLightConstants.kLimeLightName, new SimCameraProperties(),
                    RalphConstants.CameraConstants.kCenterCameraTransform);
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

    private void binds()
    {
        XboxController test = new XboxController(0);
        new Trigger(test::getXButtonPressed)
                .onTrue(new RunCommand(() -> climber.extend(), climber).until(climber::isAtForwardLimit));

        new Trigger(test::getXButtonPressed).onFalse(new RunCommand(() -> climber.stop(), climber));
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
