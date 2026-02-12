package frc.robot.subsystems.flywheelshooter.Flywheel;

public interface FlywheelIO
{

    public default void run(double speed)
    {
    }

    public default void stop()
    {
    }

    public default double getSpeed()
    {
        return 0.0;
    }

    public default double getTargetSpeed()
    {
        return 0.0;
    }

    public default boolean atTargetSpeed()
    {
        return false;
    }

}
