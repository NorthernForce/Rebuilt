package frc.robot.lobby.subsystems.turret.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.ControlRequest;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.PowerDistribution;

public interface ShooterIO
{
    public static record ShooterConstants(int kMotor1ID, int kMotor2ID, double kS, double kV, double kA, double kP,
            double kI, double kD, double kG, double kCruiseVelocity, double kAcceleration, double kJerk,
            boolean kMotor1Inverted, boolean kMotor2Inverted, AngularVelocity kErrorTolerance) {
    }

    public default void update()
    {
    }

    public default void setMotorControl(ControlRequest request)
    {
    }

    public default void setTargetSpeed(AngularVelocity speed)
    {
    }

    public default void setTargetDutyCycle(double value)
    {
    }

    public default AngularVelocity getTargetSpeed()
    {
        return RotationsPerSecond.of(0);
    }

    public default AngularVelocity getSpeed()
    {
        return RotationsPerSecond.of(0);
    }

    public default boolean isAtTargetSpeed()
    {
        return false;
    }

    public default Voltage getVoltage()
    {
        return Volts.zero();
    }

    public default Angle getPosition()
    {
        return Rotations.zero();
    }

    public void start();

    public void stop();

    public double getMotor1Current();

    public double getMotor2Current();
}