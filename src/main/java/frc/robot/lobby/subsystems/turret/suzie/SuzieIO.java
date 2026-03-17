package frc.robot.lobby.subsystems.turret.suzie;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface SuzieIO
{
    public static record SuzieConstants(int kMotorID, int kDrivingEncoderID, int kSensingEncoderID, double kS,
            double kV, double kA, double kP, double kI, double kD, double kG, double kCruiseVelocity,
            double kAcceleration, double kJerk, double kExpoV, double kExpoA, double kRotorToTurntableRatio,
            int kDrivingGearTeeth, int kSensingGearTeeth, int kTurntableGearTeeth, boolean kInverted,
            Angle kLowerSoftLimit, Angle kUpperSoftLimit, Angle kErrorTolerance,
            MotorArrangementValue kMotorArrangement/* , EasyCRTConfig kCRTConfig */) {
    }

    public default void update()
    {
    }

    public default void setMotorControl(ControlRequest request)
    {
    }

    public default void setTargetAngle(Angle angle)
    {
    }

    public default void start()
    {
    }

    public default void stop()
    {
    }

    public default void setSpeed(double speed)
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

    public default void resetAngle(Angle angle)
    {
    }

    public default void resetCRT()
    {
    }

    public default Voltage getVoltage()
    {
        return Volts.of(0);
    }

    public default AngularVelocity getVelocity()
    {
        return RotationsPerSecond.of(0);
    }

    public default void enableSoftLimits()
    {
    }

    public default void disableSoftLimits()
    {
    }

    public default String getAppliedControlName()
    {
        return "";
    }

    public double getCurrent();

}