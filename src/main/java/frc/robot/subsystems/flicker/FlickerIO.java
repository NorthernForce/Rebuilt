package frc.robot.subsystems.flicker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public interface FlickerIO
{
    public default void rampFlicker(double speed)
    {
    };

    public default void stopFlicker()
    {
    };

    public default double getFlickerSpeed()
    {
        return 0.0;
    };

    public default boolean flickerIsAtSpeed()
    {
        return false;
    };

    public default Command getRampCommand(double speed)
    {
        return Commands.none();
    };

    public default Command getStopCommand()
    {
        return Commands.none();
    };
}
