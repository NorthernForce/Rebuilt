package frc.robot.ralph;

import org.northernforce.util.NFRRobotContainer;
import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ralph.RalphConstants.InnerElevatorConstants;
import frc.robot.ralph.RalphConstants.OuterElevatorConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.elevator.SimElevator;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import com.ctre.phoenix6.Utils;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.ralph.generated.RalphTunerConstants;
import frc.robot.ralph.subsystems.shooter.ManipulatorTalonFX;
import frc.robot.ralph.subsystems.shooter.ManipulatorTalonFX.ManipulatorState;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIO;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOLimelight;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOPhotonVisionSim;
import frc.robot.util.AutoUtil;

public class RalphContainer implements NFRRobotContainer
{
    private final CommandSwerveDrivetrain drive;
    private final Superstructure superstructure;
    private final AprilTagVisionIO vision;
    private final AutoUtil autoUtil;
    private final Field2d field;
    private final ManipulatorTalonFX manipulator;

    public RalphContainer()
    {
        manipulator = new ManipulatorTalonFX(RalphConstants.ShooterConstants.kMotorId,
                RalphConstants.ShooterConstants.kIntakeSpeed, RalphConstants.ShooterConstants.kOuttakeSpeed,
                RalphConstants.ShooterConstants.kSlowOuttakeSpeed, RalphConstants.ShooterConstants.kPurgeSpeed,
                RalphConstants.ShooterConstants.kReentrySpeed, RalphConstants.ShooterConstants.kReentryTimeout,
                RalphConstants.ShooterConstants.kBruteOuttakeTimeout, RalphConstants.ShooterConstants.kMotorInverted,
                RalphConstants.ShooterConstants.kBeamBreakId, RalphConstants.ShooterConstants.kStatorCurrentLimit,
                RalphConstants.ShooterConstants.kStatorCurrentLimitEnable);
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
                    RalphConstants.CameraConstants.kFrontRightCameraTransform,
                    RalphConstants.VisionConstants.LimeLightConstants.kValidIds);
        }
        field = new Field2d();

        autoUtil = new AutoUtil(drive, RalphConstants.AutoConstants.xPid, RalphConstants.AutoConstants.yPid,
                RalphConstants.AutoConstants.rPid);
        autoUtil.bindAutoDefault("TestAuto", this::testAuto);

        Shuffleboard.getTab("Developer").add(field);
        Shuffleboard.getTab("Developer").add("Reset Encoders", drive.resetEncoders());
        Shuffleboard.getTab("Developer").add("Reset Orientation", drive.resetOrientation());
        if (RobotBase.isReal())
        {
            superstructure = new Superstructure();
        } else
        {
            superstructure = new Superstructure(
                    new SimElevator(InnerElevatorConstants.kCanID, InnerElevatorConstants.kSensorID,
                            InnerElevatorConstants.kConfig),
                    new SimElevator(OuterElevatorConstants.kCanID, OuterElevatorConstants.kSensorID,
                            OuterElevatorConstants.kConfig));
        }
        Shuffleboard.getTab("Developer").add("Drive to Blue Reef",
                drive.navigateToPose(new Pose2d(3, 4, new Rotation2d())));
    }

    public Command bruteOuttake()
    {
        return Commands.runOnce(() -> manipulator.setState(ManipulatorState.EXPULSANDO_BRUTO));
    }

    /* gets the drive subsystem
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

    public ManipulatorTalonFX getShooter()
    {
        return manipulator;
    }

    @Override
    public Command getAutonomousCommand()
    {
        return autoUtil.getSelected();
    }

    public void autonomousInit()
    {
        if (manipulator.hasCoralInSensor())
        {
            manipulator.setState(ManipulatorState.FELIZ);
        } else
        {
            manipulator.setState(ManipulatorState.HAMBRIENTO);
        }
    }

    public AutoRoutine testAuto(AutoFactory factory)
    {
        var routine = factory.newRoutine("TestAuto");

        var testPath = routine.trajectory("TestPath");
        var testPathReturn = routine.trajectory("TestPathReturn");

        routine.active().onTrue(Commands.sequence(testPath.resetOdometry(), testPath.cmd(),
                Commands.runOnce(() -> System.out.println("RETURNING")), testPathReturn.cmd()));

        return routine;
    }

    public Superstructure getSuperstructure()
    {
        return superstructure;
    }
}
