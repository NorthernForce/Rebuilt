package frc.robot.subsystems.turret.hood;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.units.measure.Angle;

public interface HoodIO
{
    public static record HoodConstants(int motorID, int encoderID, double kS, double kV, double kA, double kP,
            double kI, double kD, double kG, double cruiseVelocity, double acceleration, double jerk, double gearRatio,
            boolean inverted, Angle lowerSoftLimit, Angle upperSoftLimit, Angle errorTolerance,
            MotorArrangementValue motorArrangement) {
    };

    public default void updateStatusSignals()
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