package frc.robot.subsystems.turret.hood;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.units.measure.Angle;

public interface HoodIO
{
    public static record HoodConstants(int kMotorID, int kEncoderID, double kS, double kV, double kA, double kP,
            double kI, double kD, double kG, double kCruiseVelocity, double kAcceleration, double kJerk,
            double kGearRatio, boolean kInverted, Angle kLowerSoftLimit, Angle kUpperSoftLimit, Angle kErrorTolerance,
            MotorArrangementValue kMotorArrangement) {
    };

    public default void update()
    {
    }

    public default void setTargetAngle(Angle angle)
    {
    }

    public default void setSpeed(double speed, boolean overrideSoftLimit)
    {
    }

    public default Angle getTargetAngle()
    {
        return Rotations.of(0);
    }

    public default Angle getAngle()
    {
        return Rotations.of(0);
    }

    public default boolean isAtTargetAngle()
    {
        return false;
    }

    public default void resetAngle()
    {
    }
}