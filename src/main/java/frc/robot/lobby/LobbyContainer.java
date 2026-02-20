package frc.robot.lobby;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;

import java.util.Optional;
import org.northernforce.util.NFRRobotContainer;
import org.photonvision.simulation.SimCameraProperties;

import com.ctre.phoenix6.Utils;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.lobby.generated.LobbyTunerConstants;
import frc.robot.lobby.subsystems.CommandSwerveDrivetrain;
import frc.robot.lobby.subsystems.apriltagvision.*;
import frc.robot.subsystems.turret.Turret.TurretConstants;
import frc.robot.subsystems.turret.hood.Hood;
import frc.robot.subsystems.turret.hood.HoodIO.HoodConstants;
import frc.robot.subsystems.turret.hood.HoodIOServo;
import frc.robot.subsystems.turret.hood.HoodIOServoSim;
import frc.robot.subsystems.turret.hood.HoodIOTalonFXS;
import frc.robot.subsystems.turret.shooter.Shooter;
import frc.robot.subsystems.turret.shooter.ShooterIO.ShooterConstants;
import frc.robot.subsystems.turret.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.turret.shooter.ShooterIOTalonFXSim;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.suzie.Suzie;
import frc.robot.subsystems.turret.suzie.SuzieIO.SuzieConstants;
import frc.robot.subsystems.turret.suzie.SuzieIOTalonFXS;
import frc.robot.subsystems.turret.suzie.SuzieIOTalonFXSSim;
import frc.robot.lobby.subsystems.apriltagvision.commands.DriveToPoseWithVision;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.lobby.subsystems.CommandSwerveDrivetrain;
import frc.robot.lobby.subsystems.CommandSwerveDrivetrain;
import frc.robot.lobby.subsystems.spindexer.Spindexer;
import frc.robot.lobby.subsystems.spindexer.Spindexer.SpindexerParameters;
import frc.robot.lobby.subsystems.spindexer.carousel.CarouselIO.CarouselConstants;
import frc.robot.lobby.subsystems.spindexer.carousel.CarouselIOTalonFX;
import frc.robot.lobby.subsystems.spindexer.carousel.CarouselIOTalonFXSim;
import frc.robot.lobby.subsystems.spindexer.flicker.FlickerIOTalonFXS;
import frc.robot.lobby.subsystems.spindexer.flicker.FlickerIOTalonFXSSim;
import frc.robot.lobby.subsystems.spindexer.flicker.FlickerParameters;
import frc.robot.lobby.subsystems.spindexer.flicker.FlickerSimParameters;
import frc.robot.lobby.subsystems.apriltagvision.*;
import frc.robot.lobby.subsystems.CommandSwerveDrivetrain;
import frc.robot.lobby.subsystems.apriltagvision.*;
import frc.robot.lobby.subsystems.apriltagvision.commands.DriveToPoseWithVision;
import frc.robot.util.AutoUtil;
import frc.robot.util.InterpolatedTargetingCalculator;
import frc.robot.util.TrigHoodTargetingCalculator;

public class LobbyContainer implements NFRRobotContainer
{
    private final CommandSwerveDrivetrain drive;
    private final Intake intake;

    private final AprilTagVision vision;
    private final AutoUtil autoUtil;
    private final Field2d field;
    private final Turret turret;
    private final GenericEntry flickerSpeedEntry;
    private final GenericEntry indexerSpeedEntry;
    private final GenericEntry shooterSpeedEntry;
    private final GenericEntry shooterDutyCycleEntry;
    private final GenericEntry shooterKPEntry;
    private final GenericEntry shooterKIEntry;
    private final GenericEntry shooterKDEntry;
    private final GenericEntry shooterKVEntry;
    private final GenericEntry shooterKAEntry;
    private final Spindexer spindexer;
    private final GenericEntry hoodAngleEntry;
    private final DriveToPoseWithVision driveToPoseCommand;
    private Optional<String> teamActivity = Optional.empty();

    public LobbyContainer()
    {

        drive = new CommandSwerveDrivetrain(LobbyTunerConstants.DrivetrainConstants,
                LobbyConstants.DrivetrainConstants.kMaxSpeed, LobbyConstants.DrivetrainConstants.kMaxAngularSpeed,
                LobbyTunerConstants.FrontLeft, LobbyTunerConstants.FrontRight, LobbyTunerConstants.BackLeft,
                LobbyTunerConstants.BackRight);
        drive.resetPose(new Pose2d(3, 3, new Rotation2d()));

        // Create list of all 4 trench positions
        List<Translation2d> allTrenchPositions = List.of(FieldConstants.kBlueTrench1, FieldConstants.kBlueTrench2,
                FieldConstants.kRedTrench1, FieldConstants.kRedTrench2);

        drive.setVisionMeasurementStdDevs(LobbyConstants.VisionConstants.kStdDevs);
        if (Utils.isSimulation())
        {
            // TODO: get camera json config for sim
            vision = new AprilTagVision(drive,
                    new AprilTagVisionIOPhotonVisionSim(
                            LobbyConstants.VisionConstants.LimeLightConstants.kLimeLightName, new SimCameraProperties(),
                            LobbyConstants.CameraConstants.kBackLeftCameraTransform));
            turret = new Turret(new TurretConstants(LobbyConstants.Turret.offset),
                    new Suzie(new SuzieIOTalonFXSSim(new SuzieConstants(LobbyConstants.Turret.Suzie.kMotorID,
                            LobbyConstants.Turret.Suzie.kEncoderID, LobbyConstants.Turret.Suzie.kS,
                            LobbyConstants.Turret.Suzie.kV, LobbyConstants.Turret.Suzie.kA,
                            LobbyConstants.Turret.Suzie.kP, LobbyConstants.Turret.Suzie.kI,
                            LobbyConstants.Turret.Suzie.kD, LobbyConstants.Turret.Suzie.kG,
                            LobbyConstants.Turret.Suzie.kCruiseVelocity, LobbyConstants.Turret.Suzie.kAcceleration,
                            LobbyConstants.Turret.Suzie.kJerk, LobbyConstants.Turret.Suzie.kGearRatio,
                            LobbyConstants.Turret.Suzie.kInverted, LobbyConstants.Turret.Suzie.kLowerSoftLimit,
                            LobbyConstants.Turret.Suzie.kUpperSoftLimit, LobbyConstants.Turret.Suzie.kErrorTolerance,
                            LobbyConstants.Turret.Suzie.kMotorArrangement,
                            LobbyConstants.Turret.Suzie.kEncoderDIOPin))),
                    new Hood(new HoodIOServoSim(new HoodConstants(LobbyConstants.Turret.Hood.kMotorID,
                            LobbyConstants.Turret.Hood.kEncoderID, LobbyConstants.Turret.Hood.kS,
                            LobbyConstants.Turret.Hood.kV, LobbyConstants.Turret.Hood.kA, LobbyConstants.Turret.Hood.kP,
                            LobbyConstants.Turret.Hood.kI, LobbyConstants.Turret.Hood.kD, LobbyConstants.Turret.Hood.kG,
                            LobbyConstants.Turret.Hood.kCruiseVelocity, LobbyConstants.Turret.Hood.kAcceleration,
                            LobbyConstants.Turret.Hood.kJerk, LobbyConstants.Turret.Hood.kGearRatio,
                            LobbyConstants.Turret.Hood.kInverted, LobbyConstants.Turret.Hood.kLowerSoftLimit,
                            LobbyConstants.Turret.Hood.kUpperSoftLimit, LobbyConstants.Turret.Hood.kErrorTolerance,
                            LobbyConstants.Turret.Hood.kMotorArrangement, LobbyConstants.Turret.Hood.kDangerZone,
                            allTrenchPositions, LobbyConstants.Turret.Hood.kMechanismLowerAngle,
                            LobbyConstants.Turret.Hood.kMechanismUpperAngle))),
                    new Shooter(new ShooterIOTalonFXSim(new ShooterConstants(LobbyConstants.Turret.Shooter.kMotor1ID,
                            LobbyConstants.Turret.Shooter.kMotor2ID, LobbyConstants.Turret.Shooter.kS,
                            LobbyConstants.Turret.Shooter.kV, LobbyConstants.Turret.Shooter.kA,
                            LobbyConstants.Turret.Shooter.kP, LobbyConstants.Turret.Shooter.kI,
                            LobbyConstants.Turret.Shooter.kD, LobbyConstants.Turret.Shooter.kG,
                            LobbyConstants.Turret.Shooter.kCruiseVelocity, LobbyConstants.Turret.Shooter.kAcceleration,
                            LobbyConstants.Turret.Shooter.kJerk, LobbyConstants.Turret.Shooter.kMotor1Inverted,
                            LobbyConstants.Turret.Shooter.kMotor2Inverted,
                            LobbyConstants.Turret.Shooter.kErrorTolerance))),
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
                    new AprilTagVisionIOLimelight(LobbyConstants.VisionConstants.LimeLightConstants.kLimeLightName,
                            LobbyConstants.CameraConstants.kBackLeftCameraTransform,
                            LobbyConstants.VisionConstants.LimeLightConstants.kValidIds));
            turret = new Turret(new TurretConstants(LobbyConstants.Turret.offset),
                    new Suzie(new SuzieIOTalonFXS(new SuzieConstants(LobbyConstants.Turret.Suzie.kMotorID,
                            LobbyConstants.Turret.Suzie.kEncoderID, LobbyConstants.Turret.Suzie.kS,
                            LobbyConstants.Turret.Suzie.kV, LobbyConstants.Turret.Suzie.kA,
                            LobbyConstants.Turret.Suzie.kP, LobbyConstants.Turret.Suzie.kI,
                            LobbyConstants.Turret.Suzie.kD, LobbyConstants.Turret.Suzie.kG,
                            LobbyConstants.Turret.Suzie.kCruiseVelocity, LobbyConstants.Turret.Suzie.kAcceleration,
                            LobbyConstants.Turret.Suzie.kJerk, LobbyConstants.Turret.Suzie.kGearRatio,
                            LobbyConstants.Turret.Suzie.kInverted, LobbyConstants.Turret.Suzie.kLowerSoftLimit,
                            LobbyConstants.Turret.Suzie.kUpperSoftLimit, LobbyConstants.Turret.Suzie.kErrorTolerance,
                            LobbyConstants.Turret.Suzie.kMotorArrangement,
                            LobbyConstants.Turret.Suzie.kEncoderDIOPin))),
                    new Hood(new HoodIOServo(new HoodConstants(LobbyConstants.Turret.Hood.kServoID,
                            LobbyConstants.Turret.Hood.kEncoderID, LobbyConstants.Turret.Hood.kS,
                            LobbyConstants.Turret.Hood.kV, LobbyConstants.Turret.Hood.kA, LobbyConstants.Turret.Hood.kP,
                            LobbyConstants.Turret.Hood.kI, LobbyConstants.Turret.Hood.kD, LobbyConstants.Turret.Hood.kG,
                            LobbyConstants.Turret.Hood.kCruiseVelocity, LobbyConstants.Turret.Hood.kAcceleration,
                            LobbyConstants.Turret.Hood.kJerk, LobbyConstants.Turret.Hood.kGearRatio,
                            LobbyConstants.Turret.Hood.kInverted, LobbyConstants.Turret.Hood.kLowerServoLimit,
                            LobbyConstants.Turret.Hood.kUpperServoLimit, LobbyConstants.Turret.Hood.kErrorTolerance,
                            LobbyConstants.Turret.Hood.kMotorArrangement, LobbyConstants.Turret.Hood.kDangerZone,
                            allTrenchPositions, LobbyConstants.Turret.Hood.kMechanismLowerAngle,
                            LobbyConstants.Turret.Hood.kMechanismUpperAngle))),
                    new Shooter(new ShooterIOTalonFX(new ShooterConstants(LobbyConstants.Turret.Shooter.kMotor1ID,
                            LobbyConstants.Turret.Shooter.kMotor2ID, LobbyConstants.Turret.Shooter.kS,
                            LobbyConstants.Turret.Shooter.kV, LobbyConstants.Turret.Shooter.kA,
                            LobbyConstants.Turret.Shooter.kP, LobbyConstants.Turret.Shooter.kI,
                            LobbyConstants.Turret.Shooter.kD, LobbyConstants.Turret.Shooter.kG,
                            LobbyConstants.Turret.Shooter.kCruiseVelocity, LobbyConstants.Turret.Shooter.kAcceleration,
                            LobbyConstants.Turret.Shooter.kJerk, LobbyConstants.Turret.Shooter.kMotor1Inverted,
                            LobbyConstants.Turret.Shooter.kMotor2Inverted,
                            LobbyConstants.Turret.Shooter.kErrorTolerance))),
                    new InterpolatedTargetingCalculator(LobbyConstants.Turret.Hood.kTargetingDataFilepath),
                    new InterpolatedTargetingCalculator(LobbyConstants.Turret.Hood.kTargetingDataFilepath));
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

        intake = new Intake(new IntakeIOTalonFX(LobbyConstants.IntakeConstants.kRollerMotorId,
                LobbyConstants.IntakeConstants.kAngleMotorId, LobbyConstants.IntakeConstants.kAngleEncoderId,
                LobbyConstants.IntakeConstants.kDownAngle, LobbyConstants.IntakeConstants.kMiddleAngle,
                LobbyConstants.IntakeConstants.kStowedAngle));

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
        indexerSpeedEntry = Shuffleboard.getTab("Tuning")
                .add("Indexer Speed", (double) spindexer.getCarousel().getTargetPower()).getEntry();
        flickerSpeedEntry = Shuffleboard.getTab("Tuning")
                .add("Flicker Speed", (double) spindexer.getFlicker().getTargetPower()).getEntry();
        shooterSpeedEntry = Shuffleboard.getTab("Tuning")
                .add("Shooter Velocity", (double) turret.getShooter().getIO().getTargetSpeed().in(RotationsPerSecond))
                .getEntry();

        hoodAngleEntry = Shuffleboard.getTab("Tuning").add("Hood Angle (degrees)", 0.0).getEntry();

        shooterDutyCycleEntry = Shuffleboard.getTab("Tuning").add("Shooter Duty Cycle", 0).getEntry();
        shooterKPEntry = Shuffleboard.getTab("Tuning").add("Shooter kP", 0).getEntry();
        shooterKIEntry = Shuffleboard.getTab("Tuning").add("Shooter kI", 0).getEntry();
        shooterKDEntry = Shuffleboard.getTab("Tuning").add("Shooter kD", 0).getEntry();
        shooterKVEntry = Shuffleboard.getTab("Tuning").add("Shooter kV", 0).getEntry();
        shooterKAEntry = Shuffleboard.getTab("Tuning").add("Shooter kA", 0).getEntry();

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
        DogLog.log("Turret/Position",
                new Pose2d(turret.calculateFieldRelativeShooterPosition(getDrive().getState().Pose), new Rotation2d()));
        DogLog.log("Turret/Direction",
                getTurret().calculateFieldRelativeShooterPosition(drive.getState().Pose)
                        .plus(new Translation2d(
                                Math.cos(getTurret().getSuzie().getIO().getAngle().in(Radians)
                                        + getDrive().getState().Pose.getRotation().getRadians()),
                                Math.sin(getTurret().getSuzie().getIO().getAngle().in(Radians)
                                        + getDrive().getState().Pose.getRotation().getRadians()))));

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
        double indexerSpeed = indexerSpeedEntry.getDouble(spindexer.getCarousel().getPower());
        spindexer.getCarousel().setPower(indexerSpeed);

        double flickerSpeed = flickerSpeedEntry.getDouble(spindexer.getFlicker().getPower());
        spindexer.getFlicker().setPower(flickerSpeed);

        double shooterSpeed = shooterSpeedEntry
                .getDouble(turret.getShooter().getIO().getSpeed().in(RotationsPerSecond));
        turret.getShooter().getIO().setPotentialSpeed(RotationsPerSecond.of(shooterSpeed));

        // double shooterDuty = shooterDutyCycleEntry.getDouble(0);
        // turret.getShooter().setPotentialDutyCycle(shooterDuty);

        double shooterKP = shooterKPEntry.getDouble(0);
        double shooterKI = shooterKIEntry.getDouble(0);
        double shooterKD = shooterKDEntry.getDouble(0);
        double shooterKV = shooterKVEntry.getDouble(0);
        double shooterKA = shooterKAEntry.getDouble(0);
        turret.getHood().getIO().setTargetMechanismAngle(Degrees.of(hoodAngleEntry.getDouble(0.0)));
        turret.getShooter().getIO().setPID(shooterKP, shooterKI, shooterKD, shooterKV, shooterKA);

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
