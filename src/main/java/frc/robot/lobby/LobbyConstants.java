package frc.robot.lobby;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import java.util.List;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.FieldConstants;
import frc.robot.lobby.generated.LobbyTunerConstants;
import frc.robot.lobby.subsystems.turret.hood.HoodIO.HoodConstants;
import frc.robot.lobby.subsystems.turret.shooter.ShooterIO.ShooterConstants;
import frc.robot.lobby.subsystems.turret.suzie.SuzieIO.SuzieConstants;

public class LobbyConstants
{
    public class DrivetrainConstants
    {
        public static final LinearVelocity kMaxSpeed = LobbyTunerConstants.kSpeedAt12Volts;
        public static final AngularVelocity kMaxAngularSpeed = RotationsPerSecond.of(3.0);
        public static final double kPPDriveTP = 2.9;
        public static final double kPPDriveTI = 0.0;
        public static final double kPPDriveTD = 0.0;
        public static final PIDConstants kPPDrivePID = new PIDConstants(kPPDriveTP, kPPDriveTI, kPPDriveTD);

        public static final double kPPDriveRP = 5.0;
        public static final double kPPDriveRI = 0.0;
        public static final double kPPDriveRD = 0.0;
        public static final PIDConstants kPPDriveRPID = new PIDConstants(kPPDriveRP, kPPDriveRI, kPPDriveRD);

        public static final Distance kDriveRadius = Meters
                .of(Math.hypot(LobbyTunerConstants.FrontRight.LocationX, LobbyTunerConstants.FrontRight.LocationY));

        public static final AngularVelocity kMaxAngularVelocity = RadiansPerSecond
                .of(LobbyTunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / kDriveRadius.in(Meters));

        public static final LinearVelocity kPPMaxVelocity = MetersPerSecond.of(3.5);
        public static final LinearAcceleration kPPMaxAcceleration = MetersPerSecondPerSecond.of(3.0);
        public static final AngularVelocity kPPMaxAngularVelocity = RadiansPerSecond.of(3.0);
        public static final AngularAcceleration kPPMaxAngularAcceleration = RadiansPerSecondPerSecond.of(3.0);
        public static final Voltage kNominalVoltage = Volts.of(12.0);
        public static final PathConstraints kPPConstraints = new PathConstraints(kPPMaxVelocity, kPPMaxAcceleration,
                kPPMaxAngularVelocity, kPPMaxAngularAcceleration);

        public static final double kCloseDriveTP = 2.65;
        public static final double kCloseDriveTI = 0.0;
        public static final double kCloseDriveTD = 0.3;

        public static final double kCloseDriveRP = 5;
        public static final double kCloseDriveRI = 0.0;
        public static final double kCloseDriveRD = 0.0;
    }

    public class AutoConstants
    {
        public static final PIDController xPid = new PIDController(10.0, 0.0, 0.0);
        public static final PIDController yPid = new PIDController(10.0, 0.0, 0.0);
        public static final PIDController rPid = new PIDController(7.0, 0.0, 0.0);
        static
        {
            rPid.enableContinuousInput(0, 2 * Math.PI);
        }
    }

    public static class CameraConstants
    {

        public static final Transform3d kBackLeftCameraTransform = new Transform3d(
                new Translation3d(Meters.of(-0.14605), Meters.of(-0.1397), Meters.of(0.1778)),
                new Rotation3d(Degrees.zero(), Degrees.of(2.0), Degrees.of(78.5)));
    }

    public class VisionConstants
    {
        public static final Vector<N3> kStdDevs = VecBuilder.fill(0.9, 0.9, 999999);

        public class LimeLightConstants
        {
            public static final String kLimeLightName = "limelight-left";
            public static final int[] kValidIds =
            { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
                    30, 31 };

        }

        public class PhotonVisionConstants
        {
            public static final Transform3d kRobotToCamera = new Transform3d(new Translation3d(0.0, 0.0, 0.0),
                    new Rotation3d(0.0, 0.0, 0.0));
        }

    }

    public class Turret
    {
        public static Pose2d offset = new Pose2d(new Translation2d(Inches.of(6.264), Inches.of(6.300)),
                new Rotation2d(Degrees.of(2.5)));

        public class Suzie
        {
            private static int kMotorID = 19;
            private static int kDrivingEncoderID = 9;
            private static int kSensingEncoderID = 8;
            private static double kS = 0;
            private static double kV = 0.116;
            private static double kA = 0.110;
            private static double kP = 10;
            private static double kI = 0;
            private static double kD = 0.6;
            private static double kG = 0;
            private static double kCruiseVelocity = 0;
            private static double kAcceleration = 160;
            private static double kJerk = 1600;
            private static int kDrivingGearTeeth = 24;
            private static int kSensingGearTeeth = 25;
            private static int kTurntableGearTeeth = 120;
            private static boolean kInverted = false;
            private static Angle kLowerSoftLimit = Degrees.of(-180);
            private static Angle kUpperSoftLimit = Degrees.of(180);
            private static Angle kErrorTolerance = Degrees.of(1);
            private static MotorArrangementValue kMotorArrangement = MotorArrangementValue.Minion_JST;

            public static SuzieConstants kMinionConstants = new SuzieConstants(kMotorID, kDrivingEncoderID,
                    kSensingEncoderID, kS, kV, kA, kP, kI, kD, kG, kCruiseVelocity, kAcceleration, kJerk,
                    kDrivingGearTeeth, kSensingGearTeeth, kTurntableGearTeeth, kInverted, kLowerSoftLimit,
                    kUpperSoftLimit, kErrorTolerance, kMotorArrangement);
        }

        public class Hood
        {
            private static Distance kDangerZone = Feet.of(10);
            private static int kMotorID = 11;
            private static int kEncoderID = 2;
            private static double kS = 0;
            private static double kV = 0.12 * 10;
            private static double kA = 0.8 / 10;
            private static double kP = 10;
            private static double kI = 0;
            private static double kD = 0;
            private static double kG = 0;
            private static double kCruiseVelocity = 0;
            private static double kAcceleration = 160;
            private static double kJerk = 1600;
            private static double kGearRatio = 10;
            private static boolean kInverted = false;
            private static Angle kLowerSoftLimit = Degrees.of(0);
            private static Angle kUpperSoftLimit = Degrees.of(20);
            private static Angle kErrorTolerance = Degrees.of(1);
            private static MotorArrangementValue kMotorArrangement = MotorArrangementValue.Minion_JST;

            // Create list of all 4 trench positions
            private static List<Translation2d> kAllTrenchPositions = List.of(FieldConstants.kBlueTrench1,
                    FieldConstants.kBlueTrench2, FieldConstants.kRedTrench1, FieldConstants.kRedTrench2);

            public static String kTargetingDataFilepath = "src/main/java/frc/robot/subsystems/turret/targeting_data/HoodTargetingData.csv";

            // servo constants
            private static int kServoID = 9;
            private static Angle kLowerServoLimit = Degrees.of(145);
            private static Angle kUpperServoLimit = Degrees.of(-20);
            private static Angle kMechanismLowerAngle = Degrees.of(20);
            private static Angle kMechanismUpperAngle = Degrees.of(37);

            public static HoodConstants kMinionConstants = new HoodConstants(kMotorID, kEncoderID, kS, kV, kA, kP, kI,
                    kD, kG, kCruiseVelocity, kAcceleration, kJerk, kGearRatio, kInverted, kLowerSoftLimit,
                    kUpperSoftLimit, kErrorTolerance, kMotorArrangement, kDangerZone, kAllTrenchPositions,
                    kMechanismLowerAngle, kMechanismUpperAngle);
            public static HoodConstants kServoConstants = new HoodConstants(kServoID, kEncoderID, kS, kV, kA, kP, kI,
                    kD, kG, kCruiseVelocity, kAcceleration, kJerk, kGearRatio, kInverted, kLowerServoLimit,
                    kUpperServoLimit, kErrorTolerance, kMotorArrangement, kDangerZone, kAllTrenchPositions,
                    kMechanismLowerAngle, kMechanismUpperAngle);
        }

        public class Shooter
        {
            private static int kMotor1ID = 20;
            private static int kMotor2ID = 21;
            private static double kS = 0;
            private static double kV = 0.115;
            private static double kA = 1;
            private static double kP = 0.55;
            private static double kI = 0;
            private static double kD = 0;
            private static double kG = 0;
            private static double kCruiseVelocity = 100;
            private static double kAcceleration = 100;
            private static double kJerk = 0;
            private static boolean kMotor1Inverted = false;
            private static boolean kMotor2Inverted = true;
            private static AngularVelocity kErrorTolerance = RotationsPerSecond.of(10);
            public static String kTargetingDataFilepath = "src/main/java/frc/robot/subsystems/turret/targeting_data/ShooterTargetingData.csv";

            public static ShooterConstants kKrakenConstants = new ShooterConstants(kMotor1ID, kMotor2ID, kS, kV, kA, kP,
                    kI, kD, kG, kCruiseVelocity, kAcceleration, kJerk, kMotor1Inverted, kMotor2Inverted,
                    kErrorTolerance);
        }
    }

    public class FlickerConstants
    {
        public static final Current kJamCurrentThreshold = Amps.of(10);
        public static final Time kJamTimeout = Seconds.of(0.5);
        public static final double kDejamSpeed = 0.2;

        // TODO: check ALL values
        public static final int kMotorId = 18;
        public static final double kRampSpeed = 1.0;
        public static final boolean kMotorInverted = false;
        public static final double kGearRatio = 5.0;

        // PID gains for velocity control
        public static final double kV = 0.0925;
        public static final double kP = 0.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // Simulation constants
        public static final double kSimRpm = 500.0;
        public static final double kSimMoi = 0.02; // Moment of inertia in kg*m^2

        public static final double kErrorTolerance = 0.25;
    }

    public class IntakeConstants
    {
        public static final int kRollerMotorId = 17;
        public static final int kAngleMotorId = 16;
        public static final int kAngleEncoderId = 22;
        public static final double kDriverIntakeSpeed = 0.6;
        public static final double kDriverPurgeSpeed = 1.0;
        public static final Angle kDownAngle = Rotations.zero();
        public static final Angle kMiddleAngle = Rotations.of(-0.154);
        public static final Angle kStowedAngle = Rotations.of(-0.370);
    }

    public class CarouselConstants
    {
        public static final Current kJamCurrentThreshold = Amps.of(10);
        public static final Time kJamTimeout = Seconds.of(0.5);
        public static final double kDejamSpeed = 0.2;

        public static final int kMotorID = 15;
        public static final double kSpeed = 0.5;
        public static final boolean kInverted = false;
        public static final double kGearRatio = 10.0;

        public static final double kV = 0.1;
        public static final double kA = 0.05;
        public static final double kP = 0.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final AngularVelocity kErrorTolerance = RotationsPerSecond.of(0.25);
    }

    public class SpindexerConstants
    {
        public static final Time kDeJamTimeout = Seconds.of(1);
        public static final Time kDeJamTime = Seconds.of(0.3);
    }

    public class TalonFXConstants
    {
        public static final Temperature kMaxTemperature = Celsius.of(60.0);
    }
}
