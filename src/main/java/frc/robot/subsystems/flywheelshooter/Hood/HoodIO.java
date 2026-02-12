package frc.robot.subsystems.flywheelshooter.Hood;

import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.units.measure.Angle;

public interface HoodIO
{

    public default void setHoodAngle(Angle angle)
    {
    }

    public default Angle getHoodAngle()
    {
        return Rotations.of(0);
    }

    public default Angle getTargetAngle()
    {
        return Rotations.of(0);
    }

    public default boolean atTargetAngle()
    {
        return false;
    }

    public default void resetAngle()
    {
    }

}
