package frc.robot.lobby;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import java.util.Optional;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.commands.PathPlannerAuto;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.lobby.generated.LobbyTunerConstants;
import frc.robot.lobby.subsystems.CommandSwerveDrivetrain;
import frc.robot.lobby.subsystems.apriltagvision.*;
import frc.robot.lobby.subsystems.apriltagvision.commands.CloseDriveToPoseRequest;
import frc.robot.lobby.subsystems.apriltagvision.commands.DriveToPoseWithVision;
import frc.robot.lobby.subsystems.intake.Intake;
import frc.robot.lobby.subsystems.intake.IntakeIOTalonFX;
import frc.robot.lobby.subsystems.spindexer.Spindexer;
import frc.robot.lobby.subsystems.spindexer.Spindexer.SpindexerParameters;
import frc.robot.lobby.subsystems.spindexer.carousel.CarouselIO.CarouselConstants;
import frc.robot.lobby.subsystems.spindexer.carousel.CarouselIOTalonFX;
import frc.robot.lobby.subsystems.spindexer.carousel.CarouselIOTalonFXSim;
import frc.robot.lobby.subsystems.spindexer.flicker.FlickerIOTalonFXS;
import frc.robot.lobby.subsystems.spindexer.flicker.FlickerIOTalonFXSSim;
import frc.robot.lobby.subsystems.spindexer.flicker.FlickerParameters;
import frc.robot.lobby.subsystems.spindexer.flicker.FlickerSimParameters;
import frc.robot.lobby.subsystems.turret.Turret;
import frc.robot.lobby.subsystems.turret.Turret.TurretConstants;
import frc.robot.lobby.subsystems.turret.hood.Hood;
import frc.robot.lobby.subsystems.turret.hood.HoodIOServo;
import frc.robot.lobby.subsystems.turret.shooter.Shooter;
import frc.robot.lobby.subsystems.turret.shooter.ShooterIOTalonFX;
import frc.robot.lobby.subsystems.turret.shooter.ShooterIOTalonFXSim;
import frc.robot.lobby.subsystems.turret.suzie.Suzie;
import frc.robot.lobby.subsystems.turret.suzie.SuzieIOTalonFXS;
import frc.robot.lobby.subsystems.turret.suzie.SuzieIOTalonFXSSim;
import frc.robot.lobby.subsystems.turret.hood.HoodIOServoSim;
import frc.robot.util.AutoUtil;
import frc.robot.util.InterpolatedTargetingCalculator;
import frc.robot.util.TestTargetingCalculator;
import frc.robot.util.TrigHoodTargetingCalculator;
import org.northernforce.util.NFRRobotContainer;
import org.photonvision.simulation.SimCameraProperties;

public class LobbyContainer implements NFRRobotContainer
{
    private final CommandSwerveDrivetrain drive;
    private final Intake intake;

    private final AprilTagVision vision;
    private final AutoUtil autoUtil;
    private final Field2d field;
    private final Turret turret;
    private final Spindexer spindexer;
    private final DriveToPoseWithVision driveToPoseCommand;
    private Optional<String> teamActivity = Optional.empty();
    private final PowerDistribution powerDistributionHub = new PowerDistribution(LobbyConstants.PDHConstants.kPDHPort,
            LobbyConstants.PDHConstants.kModuleType);
        private final StatusSignal<Current> flDriveCurrent;
        private final StatusSignal<Current> flSteerCurrent;
        private final StatusSignal<Current> frDriveCurrent;
        private final StatusSignal<Current> frSteerCurrent;
        private final StatusSignal<Current> blDriveCurrent;
        private final StatusSignal<Current> blSteerCurrent;
        private final StatusSignal<Current> brDriveCurrent;
        private final StatusSignal<Current> brSteerCurrent;

    private InterpolatingDoubleTreeMap treeMap;

    public LobbyContainer()
    {

        drive = new CommandSwerveDrivetrain(LobbyTunerConstants.DrivetrainConstants,
                LobbyConstants.DrivetrainConstants.kMaxSpeed, LobbyConstants.DrivetrainConstants.kMaxAngularSpeed,
                LobbyTunerConstants.FrontLeft, LobbyTunerConstants.FrontRight, LobbyTunerConstants.BackLeft,
                LobbyTunerConstants.BackRight);
        flDriveCurrent = drive.getModules()[0].getDriveMotor().getSupplyCurrent();
        flSteerCurrent = drive.getModules()[0].getSteerMotor().getSupplyCurrent();
        frDriveCurrent = drive.getModules()[1].getDriveMotor().getSupplyCurrent();
        frSteerCurrent = drive.getModules()[1].getSteerMotor().getSupplyCurrent();
        blDriveCurrent = drive.getModules()[2].getDriveMotor().getSupplyCurrent();
        blSteerCurrent = drive.getModules()[2].getSteerMotor().getSupplyCurrent();
        brDriveCurrent = drive.getModules()[3].getDriveMotor().getSupplyCurrent();
        brSteerCurrent = drive.getModules()[3].getSteerMotor().getSupplyCurrent();
        drive.resetPose(new Pose2d(3, 3, new Rotation2d()));

        drive.setVisionMeasurementStdDevs(LobbyConstants.VisionConstants.kStdDevs);
        if (Utils.isSimulation())
        {
            // TODO: get camera json config for sim
            vision = new AprilTagVision(drive,
                    new AprilTagVisionIOPhotonVisionSim(
                            LobbyConstants.VisionConstants.LimeLightConstants.kLeftLimeLightName,
                            new SimCameraProperties(), LobbyConstants.CameraConstants.kLeftCameraTransform));
            turret = new Turret(new TurretConstants(LobbyConstants.Turret.offset),
                    new Suzie(new SuzieIOTalonFXSSim(LobbyConstants.Turret.Suzie.kMinionConstants)),
                    new Hood(new HoodIOServoSim(LobbyConstants.Turret.Hood.kServoConstants)),
                    new Shooter(new ShooterIOTalonFXSim(LobbyConstants.Turret.Shooter.kKrakenConstants)),
                    new TrigHoodTargetingCalculator(), new TrigHoodTargetingCalculator());
            spindexer = new Spindexer(
                    new CarouselIOTalonFXSim(new CarouselConstants(LobbyConstants.CarouselConstants.kMotorID,
                            LobbyConstants.CarouselConstants.kSpeed, LobbyConstants.CarouselConstants.kGearRatio,
                            LobbyConstants.CarouselConstants.kInverted,
                            LobbyConstants.CarouselConstants.kJamCurrentThreshold,
                            LobbyConstants.CarouselConstants.kJamTimeout,
                            LobbyConstants.CarouselConstants.kDejamSpeed)),
                    new FlickerIOTalonFXSSim(new FlickerSimParameters(LobbyConstants.FlickerConstants.kMotorId,
                            LobbyConstants.FlickerConstants.kRampSpeed, LobbyConstants.FlickerConstants.kGearRatio,
                            LobbyConstants.FlickerConstants.kV, LobbyConstants.FlickerConstants.kP,
                            LobbyConstants.FlickerConstants.kI, LobbyConstants.FlickerConstants.kD,
                            LobbyConstants.FlickerConstants.kErrorTolerance, LobbyConstants.FlickerConstants.kSimRpm,
                            LobbyConstants.FlickerConstants.kSimMoi,
                            LobbyConstants.FlickerConstants.kJamCurrentThreshold,
                            LobbyConstants.FlickerConstants.kJamTimeout, LobbyConstants.FlickerConstants.kDejamSpeed)),
                    new SpindexerParameters(LobbyConstants.SpindexerConstants.kDeJamTimeout));
        } else
        {
            vision = new AprilTagVision(drive,
                    new AprilTagVisionIOLimelight(LobbyConstants.VisionConstants.LimeLightConstants.kLeftLimeLightName,
                            LobbyConstants.CameraConstants.kLeftCameraTransform,
                            LobbyConstants.VisionConstants.LimeLightConstants.kValidIds),
                    new AprilTagVisionIOLimelight(LobbyConstants.VisionConstants.LimeLightConstants.kFrontLimeLightName,
                            LobbyConstants.CameraConstants.kFrontCameraTransform,
                            LobbyConstants.VisionConstants.LimeLightConstants.kValidIds));
            turret = new Turret(new TurretConstants(LobbyConstants.Turret.offset),
                    new Suzie(new SuzieIOTalonFXS(LobbyConstants.Turret.Suzie.kMinionConstants)),
                    new Hood(new HoodIOServo(LobbyConstants.Turret.Hood.kServoConstants)),
                    new Shooter(new ShooterIOTalonFX(LobbyConstants.Turret.Shooter.kKrakenConstants)),
                    new TestTargetingCalculator(), new InterpolatedTargetingCalculator(TargetingData.SHOOTER_DATA));
            spindexer = new Spindexer(
                    new CarouselIOTalonFX(new CarouselConstants(LobbyConstants.CarouselConstants.kMotorID,
                            LobbyConstants.CarouselConstants.kSpeed, LobbyConstants.CarouselConstants.kGearRatio,
                            LobbyConstants.CarouselConstants.kInverted,
                            LobbyConstants.CarouselConstants.kJamCurrentThreshold,
                            LobbyConstants.CarouselConstants.kJamTimeout,
                            LobbyConstants.CarouselConstants.kDejamSpeed)),
                    new FlickerIOTalonFXS(new FlickerParameters(LobbyConstants.FlickerConstants.kMotorId,
                            LobbyConstants.FlickerConstants.kRampSpeed, LobbyConstants.FlickerConstants.kGearRatio,
                            LobbyConstants.FlickerConstants.kV, LobbyConstants.FlickerConstants.kP,
                            LobbyConstants.FlickerConstants.kI, LobbyConstants.FlickerConstants.kD,
                            LobbyConstants.FlickerConstants.kErrorTolerance,
                            LobbyConstants.FlickerConstants.kJamCurrentThreshold,
                            LobbyConstants.FlickerConstants.kJamTimeout, LobbyConstants.FlickerConstants.kDejamSpeed)),
                    new SpindexerParameters(LobbyConstants.SpindexerConstants.kDeJamTimeout));
        }

        intake = new Intake(new IntakeIOTalonFX(LobbyConstants.IntakeConstants.kIOParameters),
                LobbyConstants.IntakeConstants.kParameters);

        field = new Field2d();
        driveToPoseCommand = new DriveToPoseWithVision(drive);
        autoUtil = new AutoUtil(drive, LobbyConstants.AutoConstants.xPid, LobbyConstants.AutoConstants.yPid,
                LobbyConstants.AutoConstants.rPid);
        autoUtil.bindAutoDefault("TestAuto", this::testAuto);
        autoUtil.bindAuto("ShmallAuto", new PathPlannerAuto("ShmallAuto"));

        treeMap = new InterpolatingDoubleTreeMap();
        treeMap.put(0.0, 0.0);
        treeMap.put(1.0, 10.0);
        treeMap.put(2.0, 30.0);

        Shuffleboard.getTab("Developer").add(field);
        Shuffleboard.getTab("Developer").add("Reset Encoders", drive.resetEncoders());
        Shuffleboard.getTab("Developer").add("Reset Orientation", drive.resetOrientation());

        Shuffleboard.getTab("SysId").add("Turntable Quasistatic Forward",
                turret.getSuzie().getSysIdQuasistaticForward());
        Shuffleboard.getTab("SysId").add("Turntable Quasistatic Reverse",
                turret.getSuzie().getSysIdQuasistaticReverse());
        Shuffleboard.getTab("SysId").add("Turntable Dynamic Forward", turret.getSuzie().getSysIdDynamicForward());
        Shuffleboard.getTab("SysId").add("Turntable Dynamic Reverse", turret.getSuzie().getSysIdDynamicReverse());
        Shuffleboard.getTab("Developer").add("Drive to Blue Reef",
                drive.navigateToPose(new Pose2d(3, 4, new Rotation2d())));

        // Intake Arm SysId buttons
        Shuffleboard.getTab("SysId").add("Arm Quasistatic Fwd",
                intake.sysIdArmQuasistatic(SysIdRoutine.Direction.kForward));
        Shuffleboard.getTab("SysId").add("Arm Quasistatic Rev",
                intake.sysIdArmQuasistatic(SysIdRoutine.Direction.kReverse));
        Shuffleboard.getTab("SysId").add("Arm Dynamic Fwd", intake.sysIdArmDynamic(SysIdRoutine.Direction.kForward));
        Shuffleboard.getTab("SysId").add("Arm Dynamic Rev", intake.sysIdArmDynamic(SysIdRoutine.Direction.kReverse));
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

    public Turret getTurret()
    {
        return turret;
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
        StatusSignal.refreshAll(flDriveCurrent, flSteerCurrent, frDriveCurrent, frSteerCurrent, blDriveCurrent, blSteerCurrent, brDriveCurrent, brSteerCurrent);
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
        DogLog.log("Drive/Pose", drive.getState().Pose);
        DogLog.log("Turret/Position", new Pose2d(
                getTurret().calculateFieldRelativeShooterPosition(getDrive().getState().Pose),
                new Rotation2d(turret.getSuzieAngleRobotRelative().plus(drive.getPose().getRotation().getMeasure()))));
        DogLog.log("Turret/Target Direction",
                getTurret().calculateFieldRelativeShooterPosition(drive.getPose())
                        .plus(new Translation2d(
                                Math.cos(getTurret().getSuzie().getTargetAngle().in(Radians)
                                        + getDrive().getState().Pose.getRotation().getRadians()),
                                Math.sin(getTurret().getSuzie().getTargetAngle().in(Radians)
                                        + getDrive().getState().Pose.getRotation().getRadians()))));
        DogLog.log("Turret/Direction",
                getTurret().calculateFieldRelativeShooterPosition(drive.getState().Pose)
                        .plus(new Translation2d(
                                Math.cos(getTurret().getSuzieAngleRobotRelative().in(Radians)
                                        + getDrive().getState().Pose.getRotation().getRadians()),
                                Math.sin(getTurret().getSuzieAngleRobotRelative().in(Radians)
                                        + getDrive().getState().Pose.getRotation().getRadians()))));
        DogLog.log("Turret/Distance", getTurret().calculateShooterDistanceToHub(drive.getPose()));

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
        DogLog.log("CurrentDraw/General/Voltage", powerDistributionHub.getVoltage());
        DogLog.log("CurrentDraw/General/TotalCurrent", powerDistributionHub.getTotalCurrent());
        int i = 0;
        for (double current : powerDistributionHub.getAllCurrents())
        {
            DogLog.log("CurrentDraw/General/Channel" + i, current);
            i++;
        }
        // NOTE: Per-subsystem current logging below uses CAN IDs as PDH channel numbers.
        // These will only be correct if the motor's CAN ID matches its PDH port wiring.
        // Use the generic channel logs above for accurate per-channel current data.
        DogLog.log("CurrentDraw/Turret/Shooter/LeftMotor", turret.getShooter().getMotor1Current());
        DogLog.log("CurrentDraw/Turret/Shooter/RightMotor", turret.getShooter().getMotor2Current());

        DogLog.log("CurrentDraw/Turret/Suzie", turret.getSuzie().getCurrent());
        DogLog.log("CurrentDraw/Intake/Rollers", intake.getRollerCurrent());
        DogLog.log("CurrentDraw/Intake/Angling", intake.getAnglingCurrent());
        // DogLog.log("CurrentDraw/Turret/Hood",
        // turret.getHood().getCurrent(powerDistributionHub));
        DogLog.log("CurrentDraw/Spindexer/Feeder", spindexer.getFlicker().getCurrent());
        DogLog.log("CurrentDraw/Spindexer/Carousel", spindexer.getCarousel().getCurrent());
        
        DogLog.log("CurrentDraw/DriveTrain/FrontLeft/Drive", flDriveCurrent.getValue().in(Amps));
        DogLog.log("CurrentDraw/DriveTrain/FrontLeft/Steer", flSteerCurrent.getValue().in(Amps));
        DogLog.log("CurrentDraw/DriveTrain/FrontRight/Drive", frDriveCurrent.getValue().in(Amps));
        DogLog.log("CurrentDraw/DriveTrain/FrontRight/Steer", frSteerCurrent.getValue().in(Amps));
        DogLog.log("CurrentDraw/DriveTrain/BackLeft/Drive", blDriveCurrent.getValue().in(Amps));
        DogLog.log("CurrentDraw/DriveTrain/BackLeft/Steer", blSteerCurrent.getValue().in(Amps));
        DogLog.log("CurrentDraw/DriveTrain/BackRight/Drive", brDriveCurrent.getValue().in(Amps));
        DogLog.log("CurrentDraw/DriveTrain/BackRight/Steer", brSteerCurrent.getValue().in(Amps));
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

    public Command roughDriveToPose(Pose2d pose)
    {
        return driveToPoseCommand.driveToPose(pose);
    }

    public Command driveToPose(Pose2d pose)
    {
        return Commands.sequence(roughDriveToPose(pose), closeDriveToPose(pose));
    }

    public Command closeDriveToPose(Pose2d pose)
    {
        CloseDriveToPoseRequest request = new CloseDriveToPoseRequest(pose,
                LobbyConstants.DrivetrainConstants.kCloseDriveTP, LobbyConstants.DrivetrainConstants.kCloseDriveTI,
                LobbyConstants.DrivetrainConstants.kCloseDriveTD, LobbyConstants.DrivetrainConstants.kCloseDriveRP,
                LobbyConstants.DrivetrainConstants.kCloseDriveRI, LobbyConstants.DrivetrainConstants.kCloseDriveRD,
                LobbyConstants.DrivetrainConstants.kPPMaxVelocity);
        return Commands.runOnce(() -> request.reset(drive.getPose()))
                .andThen(drive.applyRequest(() -> request).until(request::isFinished));
    }

    public void resetOdometry(Pose2d pose)
    {
        drive.resetPose(pose);
    }
}
