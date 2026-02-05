package frc.robot.subsystems.turret.hood;

import edu.wpi.first.units.measure.Angle;

public interface HoodIO {
    public default void setTargetAngle(Angle angle) {}
    public default void setSpeed(double speed, boolean overrideSoftLimit) {}

    public default Angle getTargetAngle() {}
    public default Angle getAngle() {}
    public default boolean isAtTargetAngle() {}

    public default void resetAngle() {}
}