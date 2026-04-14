package frc.robot.lobby.subsystems.spindexer.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lobby.subsystems.spindexer.Spindexer;

public class RunSpindexer extends Command
{
    private final Spindexer spindexer;
    private final Timer timer;
    private final Timer postDejam;
    private final Time postDejamTime;
    private final Time dejamTime;
    private final BooleanSupplier turretPreppedSupplier;

    public RunSpindexer(Spindexer spindexer, Time dejamTime, Time postDejamTime, BooleanSupplier turretPreppedSupplier)
    {
        addRequirements(spindexer);
        this.spindexer = spindexer;
        timer = new Timer();
        this.dejamTime = dejamTime;
        this.postDejamTime = postDejamTime;
        this.turretPreppedSupplier = turretPreppedSupplier;
        postDejam = new Timer();
    }

    public RunSpindexer(Spindexer spindexer, Time dejamTime, Time postDejamTime)
    {
        this(spindexer, dejamTime, postDejamTime, () ->
        {
            return true;
        });
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
            postDejam.stop();
            postDejam.reset();
            if (turretPreppedSupplier.getAsBoolean())
            {
                spindexer.getCarousel().startCarousel();
                spindexer.getFlicker().rampFlicker();
            } else
            {
                spindexer.getCarousel().stopCarousel();
                spindexer.getFlicker().stopFlicker();
            }
        } else
        {
            if (!postDejam.isRunning())
            {
                postDejam.start();
            }
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
