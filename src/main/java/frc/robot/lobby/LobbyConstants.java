package frc.robot.lobby;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
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

    public class ShooterConstants
    {
        public static final int kMotorId = 19;
        public static final double kIntakeSpeed = 0.6;
        public static final double kOuttakeSpeed = 0.89;
        public static final double kReentrySpeed = 0.2;
        public static final Time kReentryTimeout = Seconds.of(0.5);
        public static final double kSlowOuttakeSpeed = 0.4;
        public static final double kPurgeSpeed = 0.2;
        public static final boolean kMotorInverted = true;
        public static final int kBeamBreakId = 2;
        public static final double kStatorCurrentLimit = 40.0;
        public static final boolean kStatorCurrentLimitEnable = true;
        public static final Time kBruteOuttakeTimeout = Seconds.of(1.0);
    }

    public class FuelDetectionConstants
    {
        public static final String kLimelightName = "limelight-fuel";
        public static final double kTurnSpeedMultiplier = 5;
        public static final double kForwardSpeedMultiplier = 10;
        public static final Angle kZeroAngleCutoff = Degrees.of(23.0);
        public static final Angle kZeroAngleHorizontalCutoff = Degrees.zero();
        public static final double kForwardExponent = 3.0;
        public static final double kTurnExponent = 2.0;
        public static final double kTurnDampening = 40.0;
        public static final double kForwardDampening = 20.0;

    }

    public class MotorConstants
    {
        public class TalonFXConstants
        {
            public static final Temperature kMaxTemperature = Celsius.of(60.0);
        }
    }
}