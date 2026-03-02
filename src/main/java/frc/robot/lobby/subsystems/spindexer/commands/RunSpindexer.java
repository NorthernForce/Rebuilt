package frc.robot.lobby.subsystems.spindexer.commands;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lobby.subsystems.spindexer.Spindexer;

public class RunSpindexer extends Command
{
    private final Spindexer spindexer;
    private final Timer timer;
    private final Time dejamTime;

    public RunSpindexer(Spindexer spindexer, Time dejamTime)
    {
        addRequirements(spindexer);
        this.spindexer = spindexer;
        timer = new Timer();
        this.dejamTime = dejamTime;
    }

    @Override
    public void initialize()
    {
        timer.stop();
        timer.reset();
    }

    @Override
    public void execute()
    {
        if (spindexer.getJammed())
        {
            timer.restart();
        }
        if (timer.isRunning() && timer.hasElapsed(dejamTime) && !spindexer.getJammed())
        {
            timer.stop();
        }
        if (!timer.isRunning())
        {
            spindexer.getCarousel().startCarousel();
            spindexer.getFlicker().rampFlicker();
        } else
        {
            spindexer.getCarousel().dejam();
            spindexer.getFlicker().dejam();
        }
    }

    @Override
    public void end(boolean interrupte)
    {
        spindexer.getCarousel().stopCarousel();
        spindexer.getFlicker().stopFlicker();
    }
}
