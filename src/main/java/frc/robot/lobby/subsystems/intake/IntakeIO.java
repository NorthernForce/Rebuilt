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

    /** Log arm position and velocity signals for SysId. */
    public default void logArmSignals()
    {
    }
}
