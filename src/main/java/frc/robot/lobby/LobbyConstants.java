package frc.robot.lobby;

import edu.wpi.first.math.VecBuilder;
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
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import frc.robot.lobby.generated.LobbyTunerConstants;

public class LobbyConstants {
    public class DrivetrainConstants {
        public static final LinearVelocity kMaxSpeed = LobbyTunerConstants.kSpeedAt12Volts;
        public static final AngularVelocity kMaxAngularSpeed = RotationsPerSecond.of(3.0);
    }

    public class AutoConstants {
        public static final PIDController xPid = new PIDController(10.0, 0.0, 0.0);
        public static final PIDController yPid = new PIDController(10.0, 0.0, 0.0);
        public static final PIDController rPid = new PIDController(7.0, 0.0, 0.0);
        static {
            rPid.enableContinuousInput(0, 2 * Math.PI);
        }
    }

    public static class CameraConstants {
        public static final Transform3d kFrontRightCameraTransform = new Transform3d(
                new Translation3d(Inches.of(15.0 - 3.0), Inches.of(-(15.0 - 7.5)), Inches.of(8.5)),
                new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.zero()));

        public static final Transform3d kFrontLeftCameraTransform = new Transform3d(
                new Translation3d(Inches.of(15.0 - 3.0), Inches.of(15.0 - 7.75), Inches.of(8.5)),
                new Rotation3d(Degrees.zero(), Degrees.of(-27.4), Degrees.of(53.4)));

        public static final Transform3d kCenterCameraTransform = new Transform3d(
                new Translation3d(Inches.of(15.0 - 2.5), Inches.zero(), Inches.of(9.5)),
                new Rotation3d(Degrees.zero(), Degrees.of(-25), Degrees.zero()));
    }

    public class VisionConstants {
        public static final edu.wpi.first.math.Vector<N3> kStdDevs = VecBuilder.fill(0.9, 0.9, 999999);

        public class LimeLightConstants {
            public static final String kLimeLightName = "limelight";
            public static final int[] kValidIds = { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };
        }

        public class PhotonVisionConstants {
            public static final Transform3d kRobotToCamera = new Transform3d(new Translation3d(0.0, 0.0, 0.0),
                    new Rotation3d(0.0, 0.0, 0.0));
        }

    }

    public class Turret {
        public static Pose2d offset = new Pose2d(new Translation2d(Inches.of(6.264), Inches.of(6.300)),
                new Rotation2d(Degrees.of(2.5)));

        public class Suzie {
            public static int kMotorID = 10;
            public static int kEncoderID = 1;
            public static double kS = 0;
            public static double kV = 0;
            public static double kA = 0;
            public static double kP = 0;
            public static double kI = 0;
            public static double kD = 0;
            public static double kG = 0;
            public static double kCruiseVelocity = 0;
            public static double kAcceleration = 0;
            public static double kJerk = 0;
            public static double kGearRatio = 1;
            public static boolean kInverted = false;
            public static Angle kLowerSoftLimit = Degrees.of(0);
            public static Angle kUpperSoftLimit = Degrees.of(360);
            public static Angle kErrorTolerance = Degrees.of(2);
            public static MotorArrangementValue kMotorArrangement = MotorArrangementValue.Minion_JST;
        }

        public class Hood {
            public static int kMotorID = 11;
            public static int kEncoderID = 2;
            public static double kS = 0;
            public static double kV = 0;
            public static double kA = 0;
            public static double kP = 0;
            public static double kI = 0;
            public static double kD = 0;
            public static double kG = 0;
            public static double kCruiseVelocity = 0;
            public static double kAcceleration = 0;
            public static double kJerk = 0;
            public static double kGearRatio = 1;
            public static boolean kInverted = false;
            public static Angle kLowerSoftLimit = Degrees.of(0);
            public static Angle kUpperSoftLimit = Degrees.of(20);
            public static Angle kErrorTolerance = Degrees.of(1);
            public static MotorArrangementValue kMotorArrangement = MotorArrangementValue.Minion_JST;
        }

        public class Shooter {
            public static int kMotor1ID = 12;
            public static int kMotor2ID = 13;
            public static double kS = 0;
            public static double kV = 0;
            public static double kA = 0;
            public static double kP = 0;
            public static double kI = 0;
            public static double kD = 0;
            public static double kG = 0;
            public static double kCruiseVelocity = 0;
            public static double kAcceleration = 0;
            public static double kJerk = 0;
            public static boolean kMotor1Inverted = false;
            public static boolean kMotor2Inverted = true;
            public static AngularVelocity kErrorTolerance = RotationsPerSecond.of(10);
        }
    }

    public class MotorConstants {
        public class TalonFXConstants {
            public static final Temperature kMaxTemperature = Celsius.of(60.0);
        }
    }
}