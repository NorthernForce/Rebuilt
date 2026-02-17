package frc.robot.lobby.subsystems.spindexer.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lobby.subsystems.spindexer.Spindexer;

public class RampFlickerContinuous extends Command
{
    private final Spindexer m_spindexer;

    public RampFlickerContinuous(Spindexer spindexer)
    {
        addRequirements(spindexer);
        m_spindexer = spindexer;
    }

    @Override
    public void initialize()
    {
        m_spindexer.getFlicker().rampFlicker();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {
        m_spindexer.getFlicker().stopFlicker();
    }
}
