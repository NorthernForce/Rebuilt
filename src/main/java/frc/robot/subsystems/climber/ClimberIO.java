package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO
{

    // This class handles the data logging (voltage, current, height)
    @AutoLog
    public static class ClimberIOInputs
    {
        public double positionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ClimberIOInputs inputs)
    {
    }

    /** Run the motor at the specified voltage. */
    public default void setVoltage(double volts)
    {
    }

    /** Stop the motor. */
    public default void stop()
    {
    }

    /** Set the current position (e.g. when homing/resetting). */
    public default void setPosition(double positionMeters)
    {
    }
}