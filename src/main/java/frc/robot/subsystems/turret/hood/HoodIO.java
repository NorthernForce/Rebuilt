package frc.robot.subsystems.turret.hood;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import java.util.List;

import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public interface HoodIO
{
    public static record HoodConstants(int kMotorID, int kEncoderID, double kS, double kV, double kA, double kP,
            double kI, double kD, double kG, double kCruiseVelocity, double kAcceleration, double kJerk,
            double kGearRatio, boolean kInverted, Angle kLowerSoftLimit, Angle kUpperSoftLimit, Angle kErrorTolerance,
            MotorArrangementValue kMotorArrangement, Distance dangerZone, List<Translation2d> trenchPositions) {
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

    public default Distance getDangerZone()
    {
        return Feet.of(5);
    }

    public default void resetAngle()
    {
    }

    public default List<Translation2d> getTrenchPositions()
    {
        return List.of();
    }

    public default boolean inDangerProximity(Translation2d turretPosition, Distance dangerZone,
            List<Translation2d> trenchPositions)
    {
        for (Translation2d trench : trenchPositions)
        {
            if (turretPosition.getDistance(trench) < dangerZone.in(Meters))
            {
                return true;
            }
        }
        return false;
    }
}