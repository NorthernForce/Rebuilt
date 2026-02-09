package frc.robot.subsystems.turret.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public interface ShooterIO {
    public static record ShooterConstants(int kMotor1ID, int kMotor2ID, double kS, double kV, double kA, double kP,
            double kI, double kD, double kG, double kCruiseVelocity, double kAcceleration, double kJerk,
            boolean kMotor1Inverted, boolean kMotor2Inverted, AngularVelocity kErrorTolerance) {
    }

    public default void updateStatusSignals() {
    }

    public default void setTargetSpeed(AngularVelocity speed) {
    }

    public default AngularVelocity getTargetSpeed() {
        return RotationsPerSecond.of(0);
    }

    public default AngularVelocity getSpeed() {
        return RotationsPerSecond.of(0);
    }

    public default boolean isAtTargetSpeed() {
        return false;
    }
}