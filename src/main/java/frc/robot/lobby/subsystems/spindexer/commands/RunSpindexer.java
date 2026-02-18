package frc.robot.lobby.subsystems.spindexer.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lobby.subsystems.spindexer.Spindexer;

public class RunSpindexer extends Command
{
    private final Spindexer spindexer;

    public RunSpindexer(Spindexer spindexer)
    {
        addRequirements(spindexer);
        this.spindexer = spindexer;
    }

    @Override
    public void execute()
    {
        spindexer.getCarousel().startCarousel();
        spindexer.getFlicker().rampFlicker();
    }

    @Override
    public void end(boolean interrupte)
    {
        spindexer.getCarousel().stopCarousel();
        spindexer.getFlicker().stopFlicker();
    }
}
