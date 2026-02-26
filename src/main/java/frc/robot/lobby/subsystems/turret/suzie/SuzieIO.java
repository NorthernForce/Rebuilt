package frc.robot.lobby.subsystems.turret.suzie;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.units.measure.Angle;

public interface SuzieIO
{
    public static record SuzieConstants(int kMotorID, int kDrivingEncoderID, int kSensingEncoderID, double kS,
            double kV, double kA, double kP, double kI, double kD, double kG, double kCruiseVelocity,
            double kAcceleration, double kJerk, int kDrivingGearTeeth, int kSensingGearTeeth, int kTurntableGearTeeth,
            boolean kInverted, Angle kLowerSoftLimit, Angle kUpperSoftLimit, Angle kErrorTolerance,
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

    public default String getAppliedControlName()
    {
        return "";
    }

}