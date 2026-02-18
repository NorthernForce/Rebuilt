package frc.robot.lobby;

import org.northernforce.util.NFRRobotContainer;
import org.photonvision.simulation.SimCameraProperties;

import com.ctre.phoenix6.Utils;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.LimelightHelpers;
import frc.robot.lobby.generated.LobbyTunerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIO;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOLimelight;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOPhotonVisionSim;
import frc.robot.util.AutoUtil;

public class LobbyContainer implements NFRRobotContainer
{
    private final CommandSwerveDrivetrain drive;
    private final AprilTagVisionIO vision;
    private final Intake intake;
    private final AutoUtil autoUtil;
    private final Field2d field;

    public LobbyContainer()
    {
        drive = new CommandSwerveDrivetrain(LobbyTunerConstants.DrivetrainConstants,
                LobbyConstants.DrivetrainConstants.kMaxSpeed, LobbyConstants.DrivetrainConstants.kMaxAngularSpeed,
                LobbyTunerConstants.FrontLeft, LobbyTunerConstants.FrontRight, LobbyTunerConstants.BackLeft,
                LobbyTunerConstants.BackRight);
        drive.setVisionMeasurementStdDevs(LobbyConstants.VisionConstants.kStdDevs);
        if (Utils.isSimulation())
        {
            // TODO: get camera json config for sim
            vision = new AprilTagVisionIOPhotonVisionSim(
                    LobbyConstants.VisionConstants.LimeLightConstants.kLimeLightName, new SimCameraProperties(),
                    LobbyConstants.CameraConstants.kCenterCameraTransform);
        } else
        {
            vision = new AprilTagVisionIOLimelight(LobbyConstants.VisionConstants.LimeLightConstants.kLimeLightName,
                    LobbyConstants.CameraConstants.kFrontRightCameraTransform,
                    LobbyConstants.VisionConstants.LimeLightConstants.kValidIds);
        }
        field = new Field2d();

        intake = new Intake(new IntakeIOTalonFX(LobbyConstants.IntakeConstants.kRollerMotorId,
                LobbyConstants.IntakeConstants.kAngleMotorId, LobbyConstants.IntakeConstants.kAngleEncoderId));

        autoUtil = new AutoUtil(drive, LobbyConstants.AutoConstants.xPid, LobbyConstants.AutoConstants.yPid,
                LobbyConstants.AutoConstants.rPid);
        autoUtil.bindAutoDefault("TestAuto", this::testAuto);

        Shuffleboard.getTab("Developer").add(field);
        Shuffleboard.getTab("Developer").add("Reset Encoders", drive.resetEncoders());
        Shuffleboard.getTab("Developer").add("Reset Orientation", drive.resetOrientation());
        Shuffleboard.getTab("Developer").add("Drive to Blue Reef",
                drive.navigateToPose(new Pose2d(3, 4, new Rotation2d())));
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

    public Intake getIntake()
    {
        return intake;
    }

    @Override
    public void periodic()
    {
        vision.getPoses().forEach(m -> drive.addVisionMeasurement(m.pose(), m.timestamp()));
        field.setRobotPose(drive.getState().Pose);
        DogLog.log("BatteryVoltage", RobotController.getBatteryVoltage());
    }

    @Override
    public void bindOI()
    {
        new LobbyOI().bind(this);
    }

    @Override
    public Command getAutonomousCommand()
    {
        return autoUtil.getSelected();
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
}
