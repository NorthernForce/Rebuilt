package frc.robot.lobby.subsystems.spindexer.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.lobby.subsystems.spindexer.Spindexer;

public class PurgeSpindexer extends Command
{
    private final Spindexer spindexer;

    public PurgeSpindexer(Spindexer spindexer)
    {
        this.spindexer = spindexer;
        addRequirements(spindexer);
    }

    @Override
    public void initialize()
    {
        spindexer.getCarousel().dejam();
        spindexer.getFlicker().dejam();
    }

    @Override
    public void end(boolean interrupted)
    {
        spindexer.getCarousel().startCarousel();
        spindexer.getFlicker().stopFlicker();
    }
}