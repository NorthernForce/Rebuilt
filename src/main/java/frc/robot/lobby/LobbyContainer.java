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
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIO;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOLimelight;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOPhotonVisionSim;
import frc.robot.subsystems.turret.Turret.TurretConstants;
import frc.robot.subsystems.turret.hood.HoodIO.HoodConstants;
import frc.robot.subsystems.turret.hood.HoodIOTalonFXS;
import frc.robot.subsystems.turret.shooter.ShooterIO.ShooterConstants;
import frc.robot.subsystems.turret.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.suzie.SuzieIO.SuzieConstants;
import frc.robot.subsystems.turret.suzie.SuzieIOTalonFXS;
import frc.robot.util.AutoUtil;

public class LobbyContainer implements NFRRobotContainer {
    private final CommandSwerveDrivetrain drive;
    private final AprilTagVisionIO vision;
    private final AutoUtil autoUtil;
    private final Field2d field;
    private final Turret turret;

    public LobbyContainer() {
        drive = new CommandSwerveDrivetrain(LobbyTunerConstants.DrivetrainConstants,
                LobbyConstants.DrivetrainConstants.kMaxSpeed, LobbyConstants.DrivetrainConstants.kMaxAngularSpeed,
                LobbyTunerConstants.FrontLeft, LobbyTunerConstants.FrontRight, LobbyTunerConstants.BackLeft,
                LobbyTunerConstants.BackRight);
        if (Utils.isSimulation()) {
            // TODO: get camera json config for sim
            vision = new AprilTagVisionIOPhotonVisionSim(
                    LobbyConstants.VisionConstants.LimeLightConstants.kLimeLightName, new SimCameraProperties(),
                    LobbyConstants.CameraConstants.kCenterCameraTransform);
        } else {
            vision = new AprilTagVisionIOLimelight(LobbyConstants.VisionConstants.LimeLightConstants.kLimeLightName,
                    LobbyConstants.CameraConstants.kFrontRightCameraTransform,
                    LobbyConstants.VisionConstants.LimeLightConstants.kValidIds);
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

        turret = new Turret(
                new TurretConstants(LobbyConstants.Turret.offset),
                new SuzieIOTalonFXS(new SuzieConstants(LobbyConstants.Turret.Suzie.kMotorID,
                        LobbyConstants.Turret.Suzie.kEncoderID, LobbyConstants.Turret.Suzie.kS,
                        LobbyConstants.Turret.Suzie.kV, LobbyConstants.Turret.Suzie.kA, LobbyConstants.Turret.Suzie.kP,
                        LobbyConstants.Turret.Suzie.kI, LobbyConstants.Turret.Suzie.kD, LobbyConstants.Turret.Suzie.kG,
                        LobbyConstants.Turret.Suzie.kCruiseVelocity, LobbyConstants.Turret.Suzie.kAcceleration,
                        LobbyConstants.Turret.Suzie.kJerk, LobbyConstants.Turret.Suzie.kGearRatio,
                        LobbyConstants.Turret.Suzie.kInverted, LobbyConstants.Turret.Suzie.kLowerSoftLimit,
                        LobbyConstants.Turret.Suzie.kUpperSoftLimit, LobbyConstants.Turret.Suzie.kErrorTolerance,
                        LobbyConstants.Turret.Suzie.kMotorArrangement)),
                new HoodIOTalonFXS(new HoodConstants(LobbyConstants.Turret.Hood.kMotorID,
                        LobbyConstants.Turret.Hood.kEncoderID, LobbyConstants.Turret.Hood.kS,
                        LobbyConstants.Turret.Hood.kV, LobbyConstants.Turret.Hood.kA, LobbyConstants.Turret.Hood.kP,
                        LobbyConstants.Turret.Hood.kI, LobbyConstants.Turret.Hood.kD, LobbyConstants.Turret.Hood.kG,
                        LobbyConstants.Turret.Hood.kCruiseVelocity, LobbyConstants.Turret.Hood.kAcceleration,
                        LobbyConstants.Turret.Hood.kJerk, LobbyConstants.Turret.Hood.kGearRatio,
                        LobbyConstants.Turret.Hood.kInverted, LobbyConstants.Turret.Hood.kLowerSoftLimit,
                        LobbyConstants.Turret.Hood.kUpperSoftLimit, LobbyConstants.Turret.Hood.kErrorTolerance,
                        LobbyConstants.Turret.Hood.kMotorArrangement)),
                new ShooterIOTalonFX(new ShooterConstants(LobbyConstants.Turret.Shooter.kMotor1ID,
                        LobbyConstants.Turret.Shooter.kMotor2ID, LobbyConstants.Turret.Shooter.kS,
                        LobbyConstants.Turret.Shooter.kV, LobbyConstants.Turret.Shooter.kA,
                        LobbyConstants.Turret.Shooter.kP, LobbyConstants.Turret.Shooter.kI,
                        LobbyConstants.Turret.Shooter.kD, LobbyConstants.Turret.Shooter.kG,
                        LobbyConstants.Turret.Shooter.kCruiseVelocity, LobbyConstants.Turret.Shooter.kAcceleration,
                        LobbyConstants.Turret.Shooter.kJerk, LobbyConstants.Turret.Shooter.kMotor1Inverted,
                        LobbyConstants.Turret.Shooter.kMotor2Inverted, LobbyConstants.Turret.Shooter.kErrorTolerance)));
    }

    /**
     * gets the drive subsystem
     *
     * @return the drive subsystem
     */
    public CommandSwerveDrivetrain getDrive() {
        return drive;
    }

    public Turret getTurret() {
        return turret;
    }

    @Override
    public void periodic() {
        vision.getPoses().forEach(m -> drive.addVisionMeasurement(m.pose(), m.timestamp()));
        field.setRobotPose(drive.getState().Pose);
        DogLog.log("BatteryVoltage", RobotController.getBatteryVoltage());
    }

    @Override
    public void bindOI() {
        new LobbyOI().bind(this);
    }

    @Override
    public Command getAutonomousCommand() {
        return autoUtil.getSelected();
    }

    public AutoRoutine testAuto(AutoFactory factory) {
        var routine = factory.newRoutine("TestAuto");

        var testPath = routine.trajectory("TestPath");
        var testPathReturn = routine.trajectory("TestPathReturn");

        routine.active().onTrue(Commands.sequence(testPath.resetOdometry(), testPath.cmd(),
                Commands.runOnce(() -> System.out.println("RETURNING")), testPathReturn.cmd()));

        return routine;
    }
}
