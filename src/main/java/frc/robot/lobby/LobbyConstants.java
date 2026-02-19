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
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.lobby.generated.LobbyTunerConstants;

public class LobbyConstants
{
    public class DrivetrainConstants
    {
        public static final LinearVelocity kMaxSpeed = LobbyTunerConstants.kSpeedAt12Volts;
        public static final AngularVelocity kMaxAngularSpeed = RotationsPerSecond.of(3.0);
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
            public static int kEncoderDIOPin = 9;
            public static int kMotorID = 10;
            public static int kEncoderID = 1;
            public static double kS = 0;
            public static double kV = 0.116;
            public static double kA = 0.110;
            public static double kP = 10;
            public static double kI = 0;
            public static double kD = 0.6;
            public static double kG = 0;
            public static double kCruiseVelocity = 0;
            public static double kAcceleration = 160;
            public static double kJerk = 1600;
            public static double kGearRatio = 3 * 3 * 5;
            public static boolean kInverted = false;
            public static Angle kLowerSoftLimit = Degrees.of(-180);
            public static Angle kUpperSoftLimit = Degrees.of(180);
            public static Angle kErrorTolerance = Degrees.of(1);
            public static MotorArrangementValue kMotorArrangement = MotorArrangementValue.Minion_JST;
        }

        public class Hood
        {
            public static Distance kDangerZone = Feet.of(10);
            public static int kMotorID = 11;
            public static int kEncoderID = 2;
            public static double kS = 0;
            public static double kV = 0.12 * 10;
            public static double kA = 0.8 / 10;
            public static double kP = 10;
            public static double kI = 0;
            public static double kD = 0;
            public static double kG = 0;
            public static double kCruiseVelocity = 0;
            public static double kAcceleration = 160;
            public static double kJerk = 1600;
            public static double kGearRatio = 10;
            public static boolean kInverted = false;
            public static Angle kLowerSoftLimit = Degrees.of(0);
            public static Angle kUpperSoftLimit = Degrees.of(20);
            public static Angle kErrorTolerance = Degrees.of(1);
            public static MotorArrangementValue kMotorArrangement = MotorArrangementValue.Minion_JST;
            public static String kTargetingDataFilepath = "src/main/java/frc/robot/subsystems/turret/targeting_data/HoodTargetingData.csv";
        }

        public class Shooter
        {
            public static int kMotor1ID = 20;
            public static int kMotor2ID = 21;
            public static double kS = 0;
            public static double kV = 0.115;
            public static double kA = 0;
            public static double kP = 0.55;
            public static double kI = 0;
            public static double kD = 0;
            public static double kG = 0;
            public static double kCruiseVelocity = 100;
            public static double kAcceleration = 100;
            public static double kJerk = 0;
            public static boolean kMotor1Inverted = false;
            public static boolean kMotor2Inverted = true;
            public static AngularVelocity kErrorTolerance = RotationsPerSecond.of(10);
            public static String kTargetingDataFilepath = "src/main/java/frc/robot/subsystems/turret/targeting_data/ShooterTargetingData.csv";
        }
    }

    public class FlickerConstants
    {
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

    public class CarouselConstants
    {
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

    public class TalonFXConstants
    {
        public static final Temperature kMaxTemperature = Celsius.of(60.0);
    }
}