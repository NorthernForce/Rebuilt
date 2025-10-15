package frc.robot.ralph;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorConfig;

public class RalphConstants
{
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

    public static class InnerElevatorConstants
    {
        public static final int kCanID = 15;
        public static final int kSensorID = 0;

        // outer ratios
        public static final double kGearBoxRatio = 12.0;
        public static final double kSprocketTeeth = 16.0;
        public static final Distance kSproketPitch = Inches.of(0.25);
        public static final Distance kSprocketCircumference = kSproketPitch.times(kSprocketTeeth);

        // talon configs
        public static final double kS = 0.017384;
        public static final double kV = Units.inchesToMeters(28.59);
        public static final double kA = 0.015;
        public static final double kP = 18;
        public static final double kI = 0.0;
        public static final double kD = 0;
        public static final double kG = 0.21;
        public static final double kCruiseVelocity = 160;
        public static final double kAcceleration = 0;
        public static final double kJerk = 0;
        public static final Distance kLowerLimit = Inches.of(0.0);
        public static final Distance kUpperLimit = Inches.of(24.8);

        public static final Mass kInnerElevatorMass = Pounds.of(6.0);
        public static final Distance kErrorTolerance = Inches.of(0.25);
        public static final double kHomingSpeed = 0.25;

        public static final ElevatorConfig kConfig = new ElevatorConfig(kS, kV, kA, kP, kI, kD, kG, kCruiseVelocity,
                kAcceleration, kJerk, kSprocketCircumference, kGearBoxRatio, true, kLowerLimit, kUpperLimit,
                kInnerElevatorMass, kErrorTolerance);
    }

    public static class OuterElevatorConstants
    {
        public static final int kCanID = 14;
        public static final int kSensorID = 1;

        // outer ratios
        public static final double kGearBoxRatio = 16.0;
        public static final double kSprocketTeeth = 22.0;
        public static final Distance kSprocketPitch = Inches.of(0.25);
        public static final Distance kSprocketCircumference = kSprocketPitch.times(kSprocketTeeth);

        // talon configs
        public static final double kS = 0.052289;
        public static final double kV = Units.inchesToMeters(19.868);
        public static final double kA = 0.015;
        public static final double kP = 18;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kG = 0.31;
        public static final double kCruiseVelocity = 160;
        public static final double kAcceleration = 0;
        public static final double kJerk = 0;
        public static final Distance kLowerLimit = Inches.of(0.0);
        public static final Distance kUpperLimit = Inches.of(26.8);

        public static final Mass kOuterElevatorMass = Pounds.of(14.0);
        public static final Distance kErrorTolerance = Inches.of(0.25);
        public static final double kHomingSpeed = 0.25;

        public static final ElevatorConfig kConfig = new ElevatorConfig(kS, kV, kA, kP, kI, kD, kG, kCruiseVelocity,
                kAcceleration, kJerk, kSprocketCircumference, kGearBoxRatio, false, kLowerLimit, kUpperLimit,
                kOuterElevatorMass, kErrorTolerance);
    }
}
