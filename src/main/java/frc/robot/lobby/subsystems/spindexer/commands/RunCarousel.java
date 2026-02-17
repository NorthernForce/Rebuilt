package frc.robot.lobby.subsystems.spindexer.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lobby.subsystems.spindexer.Spindexer;

public class RunCarousel extends Command
{
    private final Spindexer m_spindexer;

    public RunCarousel(Spindexer spindexer)
    {
        addRequirements(spindexer);
        m_spindexer = spindexer;
    }

    @Override
    public void initialize()
    {
        m_spindexer.getCarousel().startCarousel();
    }

    @Override
    public void end(boolean interrupted)
    {
        m_spindexer.getCarousel().stopCarousel();
    }
}
