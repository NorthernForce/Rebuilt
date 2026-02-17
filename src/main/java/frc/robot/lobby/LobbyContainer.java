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
import frc.robot.lobby.generated.LobbyTunerConstants;
import frc.robot.lobby.subsystems.CommandSwerveDrivetrain;
import frc.robot.lobby.subsystems.apriltagvision.AprilTagVisionIO;
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
import frc.robot.util.AutoUtil;

public class LobbyContainer implements NFRRobotContainer
{
    private final CommandSwerveDrivetrain drive;
    private final AprilTagVisionIO vision;
    private final AutoUtil autoUtil;
    private final Field2d field;
    private final Spindexer spindexer;

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
            spindexer = new Spindexer(
                    new CarouselIOTalonFXSim(new CarouselConstants(LobbyConstants.CarouselConstants.kMotorID,
                            LobbyConstants.CarouselConstants.kSpeed, LobbyConstants.CarouselConstants.kGearRatio,
                            LobbyConstants.CarouselConstants.kV, LobbyConstants.CarouselConstants.kA,
                            LobbyConstants.CarouselConstants.kP, LobbyConstants.CarouselConstants.kI,
                            LobbyConstants.CarouselConstants.kD, LobbyConstants.CarouselConstants.kErrorTolerance,
                            LobbyConstants.CarouselConstants.kInverted)),
                    new FlickerIOTalonFXSSim(new FlickerSimParameters(LobbyConstants.FlickerConstants.kMotorId,
                            LobbyConstants.FlickerConstants.kRampSpeed, LobbyConstants.FlickerConstants.kGearRatio,
                            LobbyConstants.FlickerConstants.kV, LobbyConstants.FlickerConstants.kP,
                            LobbyConstants.FlickerConstants.kI, LobbyConstants.FlickerConstants.kD,
                            LobbyConstants.FlickerConstants.kErrorTolerance, LobbyConstants.FlickerConstants.kSimRpm,
                            LobbyConstants.FlickerConstants.kSimMoi)));
        } else
        {
            vision = new AprilTagVisionIOLimelight(LobbyConstants.VisionConstants.LimeLightConstants.kLimeLightName,
                    LobbyConstants.CameraConstants.kFrontRightCameraTransform,
                    LobbyConstants.VisionConstants.LimeLightConstants.kValidIds);
            spindexer = new Spindexer(
                    new CarouselIOTalonFX(new CarouselConstants(LobbyConstants.CarouselConstants.kMotorID,
                            LobbyConstants.CarouselConstants.kSpeed, LobbyConstants.CarouselConstants.kGearRatio,
                            LobbyConstants.CarouselConstants.kV, LobbyConstants.CarouselConstants.kA,
                            LobbyConstants.CarouselConstants.kP, LobbyConstants.CarouselConstants.kI,
                            LobbyConstants.CarouselConstants.kD, LobbyConstants.CarouselConstants.kErrorTolerance,
                            LobbyConstants.CarouselConstants.kInverted)),
                    new FlickerIOTalonFXS(new FlickerParameters(LobbyConstants.FlickerConstants.kMotorId,
                            LobbyConstants.FlickerConstants.kRampSpeed, LobbyConstants.FlickerConstants.kGearRatio,
                            LobbyConstants.FlickerConstants.kV, LobbyConstants.FlickerConstants.kP,
                            LobbyConstants.FlickerConstants.kI, LobbyConstants.FlickerConstants.kD,
                            LobbyConstants.FlickerConstants.kErrorTolerance)));
        }
        field = new Field2d();

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

    public Spindexer getSpindexer()
    {
        return spindexer;
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
