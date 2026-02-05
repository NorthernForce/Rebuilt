package frc.robot.subsystems.turret.shooter;

import edu.wpi.first.units.measure.AngularVelocity;

public interface ShooterIO {
    public default void setTargetSpeed(double speed) {}

    public default AngularVelocity getTargetSpeed() {}
    public default AngularVelocity getSpeed() {}
    public default boolean isAtTargetSpeed() {}
}