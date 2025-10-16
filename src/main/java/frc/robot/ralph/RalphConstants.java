package frc.robot.ralph;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.ralph.generated.RalphTunerConstants;

public class RalphConstants
{
    public class DrivetrainConstants
    {
        public static final LinearVelocity kMaxSpeed = RalphTunerConstants.kSpeedAt12Volts;
        public static final AngularVelocity kMaxAngularSpeed = RotationsPerSecond.of(3.0);
    }

    public class VisionConstants
    {
        public static final edu.wpi.first.math.Vector<N3> kStdDevs = VecBuilder.fill(0.9, 0.9, 999999);

        public class LimeLightConstants
        {
            public static final String kLimeLightName = "limelight";
            public static final int[] kValidIds =
            { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };
        }

        public class PhotonVisionConstants
        {
            public static final Transform3d kRobotToCamera = new Transform3d(new Translation3d(0.0, 0.0, 0.0),
                    new Rotation3d(0.0, 0.0, 0.0));
        }
    }

    public static class ClimberConstants
    {
        public static final int kId = 17;
        public static final double kEncoderId = 23;
        public static final boolean kInverted = false;
        public static final double kGearRatio = 100.0;
        public static final Angle kLowerLimit = Rotations.of(-0.03);
        public static final Angle kUpperLimit = Rotations.of(0.22);
        public static final Angle kSweetAngle = Degrees.of(70.0);
        public static final double kClimbSpeed = 1;
    }
}
