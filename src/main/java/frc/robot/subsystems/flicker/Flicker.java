package frc.robot.subsystems.flicker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flicker extends SubsystemBase
{
    public static FlickerIO m_io;

    public Flicker(FlickerIO io)
    {
        m_io = io;
    }

    public void rampFlicker(double speed)
    {
        m_io.rampFlicker(speed);
    }

    public void stopFlicker()
    {
        m_io.stopFlicker();
    }

    public double getFlickerSpeed()
    {
        return m_io.getFlickerSpeed();
    }

    public boolean flickerIsAtSpeed()
    {
        return m_io.flickerIsAtSpeed();
    }

    public FlickerIO getIO()
    {
        return m_io;
    }

    public Command getRampCommand(double speed)
    {
        return m_io.getRampCommand(speed);
    }

    public Command getStopCommand()
    {
        return m_io.getStopCommand();
    }

    public double getTargetSpeed()
    {
        return m_io.getFlickerSpeed();
    }

    public double getCurrentSpeed()
    {
        return m_io.getFlickerSpeed();
    }
}
