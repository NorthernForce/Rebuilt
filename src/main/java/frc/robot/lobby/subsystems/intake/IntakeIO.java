package frc.robot.lobby.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public interface IntakeIO
{
    public void intake(double speed);

    public void purgeIntake(double speed);

    public void stopIntake();

    public void setAngle(Angle angle);

    public void resetAngle();

    public void runToStowAngle();

    public void runToMidAngle();

    public void runToIntakeAngle();

    /** Apply a voltage to the arm angle motor (for SysId characterization). */
    public void setArmVoltage(Voltage voltage);

    public void setPower(double power);

    /** Log arm position and velocity signals for SysId. */
    public default void logArmSignals()
    {
    }

    /** Get arm position in rotations (for SysId logging). */
    public default double getArmPosition()
    {
        return 0;
    }

    /** Get arm velocity in rotations per second (for SysId logging). */
    public default double getArmVelocity()
    {
        return 0;
    }

    /** Get arm motor voltage (for SysId logging). */
    public default double getArmVoltage()
    {
        return 0;
    }

    /** Disable software limits (for SysId characterization). */
    public default void disableSoftLimits()
    {
    }

    /** Re-enable software limits after SysId. */
    public default void enableSoftLimits()
    {
    }
}
