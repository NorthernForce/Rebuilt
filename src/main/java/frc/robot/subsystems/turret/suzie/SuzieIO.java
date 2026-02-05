package frc.robot.subsystems.turret.suzie;

import edu.wpi.first.units.measure.Angle;

public interface SuzieIO {
    public static record SuzieConstants(double kS, double kV, double kA, double kP, double kI, double kD, double kG,
            double cruiseVelocity, double acceleration, double jerk, double gearRatio,
            boolean inverted, Angle lowerSoftLimit, Angle upperSoftLimit)

    public default void setTargetAngle(Angle angle) {}
    public default void setSpeed(double speed, boolean overrideSoftLimit) {}

    public default Angle getTargetAngle() {}
    public default Angle getAngle() {}
    public default boolean isAtTargetAngle() {}

    public default void resetAngle() {}
}