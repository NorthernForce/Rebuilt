package frc.robot.lobby;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.northernforce.util.NFRRobotContainer;
import org.photonvision.simulation.SimCameraProperties;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;

import com.ctre.phoenix6.StatusSignal;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.lobby.autos.SimpleAuto;
import frc.robot.lobby.generated.LobbyTunerConstants;
import frc.robot.lobby.subsystems.CommandSwerveDrivetrain;
import frc.robot.lobby.subsystems.apriltagvision.*;
import frc.robot.lobby.subsystems.apriltagvision.commands.CloseDriveToPoseRequest;
import frc.robot.lobby.subsystems.apriltagvision.commands.DriveToPoseWithVision;
import frc.robot.lobby.subsystems.climber.Climber;
import frc.robot.lobby.subsystems.climber.ClimberIOTalonFX;
import frc.robot.lobby.subsystems.climber.ClimberIOTalonFXSim;
import frc.robot.lobby.subsystems.intake.Intake;
import frc.robot.lobby.subsystems.intake.Intake.PumpIntake;
import frc.robot.lobby.subsystems.intake.IntakeIOTalonFX;
import frc.robot.lobby.subsystems.nfrdashboard.Dashboard;
import frc.robot.lobby.subsystems.nfrdashboard.Dashboard.DashboardSystem;
import frc.robot.lobby.subsystems.spindexer.Spindexer;
import frc.robot.lobby.subsystems.spindexer.Spindexer.SpindexerParameters;
import frc.robot.lobby.subsystems.spindexer.carousel.CarouselIO.CarouselConstants;
import frc.robot.lobby.subsystems.spindexer.carousel.CarouselIOTalonFX;
import frc.robot.lobby.subsystems.spindexer.carousel.CarouselIOTalonFXSim;
import frc.robot.lobby.subsystems.spindexer.commands.RunSpindexer;
import frc.robot.lobby.subsystems.spindexer.flicker.FlickerIOTalonFXS;
import frc.robot.lobby.subsystems.spindexer.flicker.FlickerIOTalonFXSSim;
import frc.robot.lobby.subsystems.spindexer.flicker.FlickerParameters;
import frc.robot.lobby.subsystems.spindexer.flicker.FlickerSimParameters;
import frc.robot.lobby.subsystems.turret.Turret;
import frc.robot.lobby.subsystems.turret.Turret.TurretConstants;
import frc.robot.lobby.subsystems.turret.commands.PrepTurretCommand;
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
import frc.robot.util.TrigHoodTargetingCalculator;

public class LobbyContainer implements NFRRobotContainer
{
    private final Dashboard dashboard;
    private final CommandSwerveDrivetrain drive;
    private final Intake intake;
    private final AprilTagVision vision;
    private final AutoUtil autoUtil;
    private final Field2d field;
    private final Climber climber;
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
    private LinearVelocity maxTranslationalVelocity = MetersPerSecond.of(4.0);
    private AngularVelocity maxAngularVelocity = RotationsPerSecond.of(1.5);

    public LobbyContainer()
    {
        dashboard = new Dashboard();

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
            climber = new Climber(new ClimberIOTalonFXSim(LobbyConstants.ClimberConstants.kClimberParameters));

            // TODO: get camera json config for sim
            vision = new AprilTagVision(drive,
                    new AprilTagVisionIOPhotonVisionSim(
                            LobbyConstants.VisionConstants.LimeLightConstants.kLeftLimeLightName,
                            new SimCameraProperties(), LobbyConstants.CameraConstants.kLeftCameraTransform));
            turret = new Turret(new TurretConstants(LobbyConstants.Turret.offset),
                    new Suzie(new SuzieIOTalonFXSSim(LobbyConstants.Turret.Suzie.kMinionConstants)),
                    new Hood(new HoodIOServoSim(LobbyConstants.Turret.Hood.kServoConstants)),
                    new Shooter(new ShooterIOTalonFXSim(LobbyConstants.Turret.Shooter.kKrakenSimConstants)),
                    new TrigHoodTargetingCalculator(), new TrigHoodTargetingCalculator(),
                    new TrigHoodTargetingCalculator());
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
            climber = new Climber(new ClimberIOTalonFX(LobbyConstants.ClimberConstants.kClimberParameters));

            vision = new AprilTagVision(drive,
                    new AprilTagVisionIOLimelight(LobbyConstants.VisionConstants.LimeLightConstants.kLeftLimeLightName,
                            LobbyConstants.CameraConstants.kLeftCameraTransform,
                            LobbyConstants.VisionConstants.LimeLightConstants.kValidIds),
                    new AprilTagVisionIOLimelight(LobbyConstants.VisionConstants.LimeLightConstants.kFrontLimeLightName,
                            LobbyConstants.CameraConstants.kFrontCameraTransform,
                            LobbyConstants.VisionConstants.LimeLightConstants.kValidIds),
                    new AprilTagVisionIOLimelight(LobbyConstants.VisionConstants.LimeLightConstants.kBackLimeLightName,
                            LobbyConstants.CameraConstants.kBackCameraTransform,
                            LobbyConstants.VisionConstants.LimeLightConstants.kValidIds));
            turret = new Turret(new TurretConstants(LobbyConstants.Turret.offset),
                    new Suzie(new SuzieIOTalonFXS(LobbyConstants.Turret.Suzie.kMinionConstants)),
                    new Hood(new HoodIOServo(LobbyConstants.Turret.Hood.kServoConstants)),
                    new Shooter(new ShooterIOTalonFX(LobbyConstants.Turret.Shooter.kKrakenConstants)),
                    new InterpolatedTargetingCalculator(TargetingData.HOOD_DATA),
                    new InterpolatedTargetingCalculator(TargetingData.SHOOTER_DATA),
                    new InterpolatedTargetingCalculator(TargetingData.TOF_DATA));
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

        DogLog.tunable("Drive/MaxTranslationalVelocity", 4.0, (newV) ->
        {
            setMaxTranslationalVelocity(MetersPerSecond.of(newV));
        });
        DogLog.tunable("Drive/MaxAngularVelocity", 1.5, (newV) ->
        {
            setMaxAngularVelocity(RotationsPerSecond.of(newV));
        });

        field = new Field2d();
        driveToPoseCommand = new DriveToPoseWithVision(drive);
        NamedCommands.registerCommand("Shoot",
                Commands.waitUntil(() -> turret.getSuzie().isAtTargetAngle() && turret.getShooter().isAtTargetSpeed())
                        .andThen(new RunSpindexer(getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime,
                                LobbyConstants.SpindexerConstants.kPostDeJamTime, () -> turret.isAtTargetPose()))
                        .alongWith(new PrepTurretCommand(this, false)));
        NamedCommands.registerCommand("ShootAndPump",
                Commands.waitUntil(() -> turret.getSuzie().isAtTargetAngle() && turret.getShooter().isAtTargetSpeed())
                        .andThen(new RunSpindexer(getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime,
                                LobbyConstants.SpindexerConstants.kPostDeJamTime, () -> turret.isAtTargetPose()))
                        .alongWith(new PrepTurretCommand(this, false)).alongWith(intake.pump()));
        NamedCommands.registerCommand("ShootWithPrediction",
                Commands.waitUntil(() -> turret.getSuzie().isAtTargetAngle() && turret.getShooter().isAtTargetSpeed())
                        .andThen(new RunSpindexer(getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime,
                                LobbyConstants.SpindexerConstants.kPostDeJamTime, () -> turret.isAtTargetPose()))
                        .alongWith(new PrepTurretCommand(this, true)));
        NamedCommands.registerCommand("Intake", intake.intakeMoving());
        NamedCommands.registerCommand("StopShoot",
                Commands.runOnce(() -> turret.getShooter().stop(), turret.getShooter()));
        NamedCommands.registerCommand("StopIntake", intake.stopIntake().andThen(intake.getRunToIntakeAngleCommand()));
        NamedCommands.registerCommand("RunUpClimber", climber.runUp());
        NamedCommands.registerCommand("RunDownClimber", climber.runDown());
        autoUtil = new AutoUtil(drive, LobbyConstants.AutoConstants.xPid, LobbyConstants.AutoConstants.yPid,
                LobbyConstants.AutoConstants.rPid);
        autoUtil.bindAutoDefault("DO NOTHING",
                Commands.runOnce(() -> resetOdometry(new Pose2d(drive.getPose().getTranslation(),
                        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                ? new Rotation2d(Degrees.of(0))
                                : new Rotation2d(Degrees.of(180))))));
        autoUtil.bindAuto("S1-DEPOT", new PathPlannerAuto("S1-DEPOT"));
        autoUtil.bindAuto("S2-DEPOT", new PathPlannerAuto("S2-DEPOT"));
        autoUtil.bindAuto("S1-SHOOT", new PathPlannerAuto("S1-SHOOT"));
        autoUtil.bindAuto("S3-SHOOT", new PathPlannerAuto("S3-SHOOT"));
        autoUtil.bindAuto("S1-SHOOT-DEPOT", new PathPlannerAuto("S1-SHOOT-DEPOT"));
        autoUtil.bindAuto("S1-SIMPLE", new SimpleAuto(this, new PathPlannerAuto("S1-SHOOT").getStartingPose()));
        autoUtil.bindAuto("S3-SIMPLE", new SimpleAuto(this, new PathPlannerAuto("S3-SHOOT").getStartingPose()));

        Shuffleboard.getTab("Developer").add(field);
        Shuffleboard.getTab("Developer").add("Reset Encoders", drive.resetEncoders());
        Shuffleboard.getTab("Developer").add("Reset Orientation", drive.resetOrientation());

        Shuffleboard.getTab("SysId").add("Turntable Quasistatic Forward",
                turret.getSuzie().getSysIdQuasistaticForward());
        Shuffleboard.getTab("SysId").add("Turntable Quasistatic Reverse",
                turret.getSuzie().getSysIdQuasistaticReverse());
        Shuffleboard.getTab("SysId").add("Turntable Dynamic Forward", turret.getSuzie().getSysIdDynamicForward());
        Shuffleboard.getTab("SysId").add("Turntable Dynamic Reverse", turret.getSuzie().getSysIdDynamicReverse());

        Shuffleboard.getTab("SysId").add("Shooter Quasistatic Forward",
                turret.getShooter().getSysIdQuasistaticForward());
        Shuffleboard.getTab("SysId").add("Shooter Quasistatic Reverse",
                turret.getShooter().getSysIdQuasistaticReverse());
        Shuffleboard.getTab("SysId").add("Shooter Dynamic Forward", turret.getShooter().getSysIdDynamicForward());
        Shuffleboard.getTab("SysId").add("Shooter Dynamic Reverse", turret.getShooter().getSysIdDynamicReverse());

        Shuffleboard.getTab("Developer").add("Drive to Blue Reef",
                drive.navigateToPose(new Pose2d(3, 4, new Rotation2d())));

        // Intake Arm SysId buttons
        Shuffleboard.getTab("SysId").add("Arm Quasistatic Fwd",
                intake.sysIdArmQuasistatic(SysIdRoutine.Direction.kForward));
        Shuffleboard.getTab("SysId").add("Arm Quasistatic Rev",
                intake.sysIdArmQuasistatic(SysIdRoutine.Direction.kReverse));
        Shuffleboard.getTab("SysId").add("Arm Dynamic Fwd", intake.sysIdArmDynamic(SysIdRoutine.Direction.kForward));
        Shuffleboard.getTab("SysId").add("Arm Dynamic Rev", intake.sysIdArmDynamic(SysIdRoutine.Direction.kReverse));
        dashboard.putCommand("Reset Turret", Commands.runOnce(() -> turret.getSuzie().resetEncoders()));
        dashboard.putCommand("Reset Orientation", drive.resetOrientation());
        dashboard.putLimelightStream(LobbyConstants.VisionConstants.LimeLightConstants.kLeftLimeLightName);
        dashboard.putLimelightStream(LobbyConstants.VisionConstants.LimeLightConstants.kFrontLimeLightName);
        dashboard.putCommand("Reset Suzie Encoders", Commands.runOnce(() ->
        {
            turret.getSuzie().resetEncoders();
        }));
        DashboardSystem turntableSystem = dashboard.putSystem("Turntable");
        turntableSystem.withCommand("Turntable SysId Quasistatic Forward",
                turret.getSuzie().getSysIdQuasistaticForward());
        turntableSystem.withCommand("Turntable SysId Quasistatic Reverse",
                turret.getSuzie().getSysIdQuasistaticReverse());
        turntableSystem.withCommand("Turntable SysId Dynamic Forward", turret.getSuzie().getSysIdDynamicForward());
        turntableSystem.withCommand("Turntable SysId Dynamic Reverse", turret.getSuzie().getSysIdDynamicReverse());
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

    public Command driveToPreClimbPosition()
    {
        return Commands.defer(() ->
        {
            Pose2d target = climber.getClosestClimbPose(drive.getPose());
            DogLog.log("Auto/DrivingToPreClimbPosition", target);
            return driveToPose(target);
        }, java.util.Set.of(drive));
    }

    public Command driveToClimbPost()
    {
        return Commands.deadline(Commands.waitSeconds(0.5),
                drive.applyRequest(() -> new SwerveRequest.ApplyRobotSpeeds().withSpeeds(
                        new ChassisSpeeds(MetersPerSecond.of(0), MetersPerSecond.of(0.05), DegreesPerSecond.of(0)))));
    }

    @Override
    public void periodic()
    {
        StatusSignal.refreshAll(flDriveCurrent, flSteerCurrent, frDriveCurrent, frSteerCurrent, blDriveCurrent,
                blSteerCurrent, brDriveCurrent, brSteerCurrent);
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
        field.setRobotPose(drive.getPose());
        DogLog.log("BatteryVoltage", RobotController.getBatteryVoltage());
        DogLog.log("Drive/Pose", drive.getPose());
        DogLog.log("Turret/Position", new Pose2d(getTurret().calculateFieldRelativeShooterPosition(drive.getPose()),
                new Rotation2d(turret.getSuzieAngleRobotRelative().plus(drive.getPose().getRotation().getMeasure()))));
        DogLog.log("Turret/Target Position",
                new Pose2d(getTurret().calculateFieldRelativeShooterPosition(drive.getPose()), new Rotation2d(
                        turret.getSuzieTargetAngleRobotRelative().plus(drive.getPose().getRotation().getMeasure()))));
        DogLog.log("Turret/Target Direction",
                getTurret().calculateFieldRelativeShooterPosition(drive.getPose())
                        .plus(new Translation2d(
                                Math.cos(getTurret().getSuzie().getTargetAngle().in(Radians)
                                        + getDrive().getPose().getRotation().getRadians()),
                                Math.sin(getTurret().getSuzie().getTargetAngle().in(Radians)
                                        + getDrive().getPose().getRotation().getRadians()))));
        DogLog.log("Turret/Direction",
                getTurret().calculateFieldRelativeShooterPosition(drive.getPose())
                        .plus(new Translation2d(
                                Math.cos(getTurret().getSuzieAngleRobotRelative().in(Radians)
                                        + getDrive().getPose().getRotation().getRadians()),
                                Math.sin(getTurret().getSuzieAngleRobotRelative().in(Radians)
                                        + getDrive().getPose().getRotation().getRadians()))));
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

        DogLog.log("Velocity", drive.getVelocity());
        // DogLog.log("Drive/Predicted Pose", predictPose());
        DogLog.log("CurrentDraw/General/Voltage", powerDistributionHub.getVoltage());
        DogLog.log("CurrentDraw/General/TotalCurrent", powerDistributionHub.getTotalCurrent());
        DogLog.log("CurrentDraw/PDH/Feeder", powerDistributionHub.getCurrent(8));
        DogLog.log("CurrentDraw/PDH/Suzie", powerDistributionHub.getCurrent(9));
        DogLog.log("CurrentDraw/PDH/LeftShooterMotor", powerDistributionHub.getCurrent(7));
        DogLog.log("CurrentDraw/PDH/RightShooterMotor", powerDistributionHub.getCurrent(6));
        DogLog.log("CurrentDraw/PDH/Suzie", powerDistributionHub.getCurrent(9));
        DogLog.log("CurrentDraw/PDH/Carousel", powerDistributionHub.getCurrent(4));
        DogLog.log("CurrentDraw/Turret/Shooter/LeftMotor", turret.getShooter().getMotor1Current());
        DogLog.log("CurrentDraw/Turret/Shooter/RightMotor", turret.getShooter().getMotor2Current());
        DogLog.log("Turret/Shooter/Speed", turret.getShooter().getSpeed());

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

    // public Pose2d predictPose()
    // {
    // Pose2d pose = drive.predictSeconds(Seconds.of(timePredict.getAsDouble()),
    // amtPoseCaptureFrames.getAsDouble());
    // DogLog.log("PredictedPose", pose);
    // return pose;
    // }

    public void setMaxTranslationalVelocity(LinearVelocity v)
    {
        maxTranslationalVelocity = v;
    }

    public void setMaxAngularVelocity(AngularVelocity v)
    {
        maxAngularVelocity = v;
    }

    public Translation2d predictTurretPose(DoubleSupplier translationXInputSupplier,
            DoubleSupplier translationYInputSupplier, DoubleSupplier rotationInputSupplier)
    {
        return turret.calculateFieldRelativeShooterPosition(drive.getPose()).plus(turret.updateFromTOF(drive.getPose(),
                new Translation2d(
                        translationYInputSupplier.getAsDouble() * maxTranslationalVelocity.in(MetersPerSecond),
                        translationXInputSupplier.getAsDouble() * maxTranslationalVelocity.in(MetersPerSecond))))
                .plus(new Translation2d(
                        LobbyConstants.Turret.offsetDistance.in(Meters)
                                * -Math.sin(drive.getPose().getRotation().getRadians()
                                        + LobbyConstants.Turret.offsetAngle.in(Radians))
                                * rotationInputSupplier.getAsDouble() * maxAngularVelocity.in(RadiansPerSecond),
                        LobbyConstants.Turret.offsetDistance.in(Meters)
                                * Math.cos(drive.getPose().getRotation().getRadians()
                                        + LobbyConstants.Turret.offsetAngle.in(Radians))
                                * rotationInputSupplier.getAsDouble() * maxAngularVelocity.in(RadiansPerSecond)));
    }

    @Override
    public Command getAutonomousCommand()
    {
        return autoUtil.getSelected();
    }

    public Climber getClimber()
    {
        return climber;
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
        DogLog.log("Auto/DrivingToPose", pose);
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