package frc.robot.subsystems.climber;

public interface ClimberIO
{

    public static class ClimberIOInputs
    {
        public double positionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelsius = 0.0;
        public boolean atBottomLimit = false;
    }

    public default void updateInputs(ClimberIOInputs inputs)
    {
    }

    public default void setVoltage(double volts)
    {
    }

    public default void stop()
    {
    }

    public default void setPosition(double positionMeters)
    {
    }

    public default void setServoAngle(double angleDeg)
    {
    }
}