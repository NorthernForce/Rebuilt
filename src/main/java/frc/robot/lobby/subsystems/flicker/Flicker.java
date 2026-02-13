package frc.robot.lobby.subsystems.flicker;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flicker extends SubsystemBase
{
    private static final double kSimDtSeconds = 0.02; // 20ms loop period

    private FlickerIO m_io;

    public Flicker(FlickerIO io)
    {
        m_io = io;
    }

    public void rampFlicker()
    {
        m_io.rampFlicker();
    }

    public void stopFlicker()
    {
        m_io.stopFlicker();
    }

    public FlickerIO getIO()
    {
        return m_io;
    }

    public Command getRampCommand()
    {
        return Commands.runOnce(() -> m_io.rampFlicker(), this);
    }

    public Command getStopCommand()
    {
        return Commands.runOnce(() -> m_io.stopFlicker(), this);
    }

    public double getSpeed()
    {
        return m_io.getSpeed();
    }

    @Override
    public void periodic()
    {
        // Update simulation physics (no-op on real robot)
        m_io.updateSimulation(kSimDtSeconds);

        DogLog.log("FlickerSpeed", m_io.getSpeed());
    }
}