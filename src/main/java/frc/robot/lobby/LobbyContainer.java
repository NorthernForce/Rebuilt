package frc.robot.lobby;

import java.util.Optional;
import org.northernforce.util.NFRRobotContainer;
import org.photonvision.simulation.SimCameraProperties;

import com.ctre.phoenix6.Utils;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lobby.generated.LobbyTunerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.lobby.subsystems.CommandSwerveDrivetrain;
import frc.robot.lobby.subsystems.apriltagvision.AprilTagVisionIOLimelight;
import frc.robot.lobby.subsystems.apriltagvision.AprilTagVisionIOPhotonVisionSim;
import frc.robot.lobby.subsystems.spindexer.Spindexer;
import frc.robot.lobby.subsystems.spindexer.carousel.CarouselIO.CarouselConstants;
import frc.robot.lobby.subsystems.spindexer.carousel.CarouselIOTalonFX;
import frc.robot.lobby.subsystems.spindexer.carousel.CarouselIOTalonFXSim;
import frc.robot.lobby.subsystems.spindexer.flicker.FlickerIOTalonFXS;
import frc.robot.lobby.subsystems.spindexer.flicker.FlickerIOTalonFXSSim;
import frc.robot.lobby.subsystems.spindexer.flicker.FlickerParameters;
import frc.robot.lobby.subsystems.spindexer.flicker.FlickerSimParameters;
import frc.robot.lobby.subsystems.apriltagvision.*;
import frc.robot.lobby.subsystems.apriltagvision.commands.DriveToPoseWithVision;
import frc.robot.util.AutoUtil;

public class LobbyContainer implements NFRRobotContainer
{
    private final CommandSwerveDrivetrain drive;
    private final Intake intake;

    private final AprilTagVision vision;
    private final AutoUtil autoUtil;
    private final Field2d field;
    private final Spindexer spindexer;
    private final DriveToPoseWithVision driveToPoseCommand;
    private Optional<String> teamActivity = Optional.empty();

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
            vision = new AprilTagVision(drive,
                    new AprilTagVisionIOPhotonVisionSim(
                            LobbyConstants.VisionConstants.LimeLightConstants.kLimeLightName, new SimCameraProperties(),
                            LobbyConstants.CameraConstants.kBackLeftCameraTransform));
            spindexer = new Spindexer(
                    new CarouselIOTalonFXSim(new CarouselConstants(LobbyConstants.CarouselConstants.kMotorID,
                            LobbyConstants.CarouselConstants.kSpeed, LobbyConstants.CarouselConstants.kGearRatio,
                            LobbyConstants.CarouselConstants.kInverted)),
                    new FlickerIOTalonFXSSim(new FlickerSimParameters(LobbyConstants.FlickerConstants.kMotorId,
                            LobbyConstants.FlickerConstants.kRampSpeed, LobbyConstants.FlickerConstants.kGearRatio,
                            LobbyConstants.FlickerConstants.kV, LobbyConstants.FlickerConstants.kP,
                            LobbyConstants.FlickerConstants.kI, LobbyConstants.FlickerConstants.kD,
                            LobbyConstants.FlickerConstants.kErrorTolerance, LobbyConstants.FlickerConstants.kSimRpm,
                            LobbyConstants.FlickerConstants.kSimMoi)));
        } else
        {
            vision = new AprilTagVision(drive,
                    new AprilTagVisionIOLimelight(LobbyConstants.VisionConstants.LimeLightConstants.kLimeLightName,
                            LobbyConstants.CameraConstants.kBackLeftCameraTransform,
                            LobbyConstants.VisionConstants.LimeLightConstants.kValidIds));
            spindexer = new Spindexer(
                    new CarouselIOTalonFX(new CarouselConstants(LobbyConstants.CarouselConstants.kMotorID,
                            LobbyConstants.CarouselConstants.kSpeed, LobbyConstants.CarouselConstants.kGearRatio,
                            LobbyConstants.CarouselConstants.kInverted)),
                    new FlickerIOTalonFXS(new FlickerParameters(LobbyConstants.FlickerConstants.kMotorId,
                            LobbyConstants.FlickerConstants.kRampSpeed, LobbyConstants.FlickerConstants.kGearRatio,
                            LobbyConstants.FlickerConstants.kV, LobbyConstants.FlickerConstants.kP,
                            LobbyConstants.FlickerConstants.kI, LobbyConstants.FlickerConstants.kD,
                            LobbyConstants.FlickerConstants.kErrorTolerance)));
        }

        intake = new Intake(new IntakeIOTalonFX(LobbyConstants.IntakeConstants.kRollerMotorId,
                LobbyConstants.IntakeConstants.kAngleMotorId, LobbyConstants.IntakeConstants.kAngleEncoderId));

        field = new Field2d();
        driveToPoseCommand = new DriveToPoseWithVision(drive);
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

    public Spindexer getSpindexer()
    {
        return spindexer;
    }

    @Override
    public void periodic()
    {
        var state = drive.getState();
        Rotation2d currentHeading = state.Pose.getRotation();
        Rotation2d yawRate = Rotation2d.fromRadians(state.Speeds.omegaRadiansPerSecond);
        vision.setHeading(currentHeading, yawRate);

        var visionPoses = vision.getPoses();
        DogLog.log("Vision/PoseCount", visionPoses.size());
        visionPoses.forEach(m ->
        {
            DogLog.log("Vision/VisionPose", m.pose());
            DogLog.log("Vision/Timestamp", m.timestamp());
            drive.addVisionMeasurement(m.pose(), m.timestamp());
        });
        field.setRobotPose(drive.getState().Pose);
        DogLog.log("BatteryVoltage", RobotController.getBatteryVoltage());

        if (DriverStation.getGameSpecificMessage().equals("R"))
        {
            teamActivity = Optional
                    .of((DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? "active" : "inactive");
        } else if (DriverStation.getGameSpecificMessage().equals("B"))
        {
            teamActivity = Optional
                    .of((DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? "active" : "inactive");
        }

        DogLog.log("GameData/StartingActivity", teamActivity.orElse("unknown"));
        if (!teamActivity.orElse("unknown").equals("unknown"))

            if (DriverStation.getMatchTime() > 130)
            {
                DogLog.log("GameData/GameShift", "active");

            } else if (DriverStation.getMatchTime() > 105)
            {
                DogLog.log("GameData/GameShift", teamActivity.get().equals("inactive") ? "inactive" : "active");
            } else if (DriverStation.getMatchTime() > 80)
            {
                DogLog.log("GameData/GameShift", teamActivity.get());
            } else if (DriverStation.getMatchTime() > 55)
            {
                DogLog.log("GameData/GameShift", teamActivity.get().equals("inactive") ? "inactive" : "active");
            } else if (DriverStation.getMatchTime() > 30)
            {
                DogLog.log("GameData/GameShift", teamActivity.get());
            } else
            {
                DogLog.log("GameData/GameShift", teamActivity.get().equals("inactive") ? "inactive" : "active");
            }

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

    public Command driveToPose(Pose2d pose)
    {
        return driveToPoseCommand.driveToPose(pose);
    }

    public void resetOdometry(Pose2d pose)
    {
        drive.resetPose(pose);
    }
}
