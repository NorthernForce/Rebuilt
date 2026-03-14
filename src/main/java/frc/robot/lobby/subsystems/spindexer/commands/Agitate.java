package frc.robot.lobby.subsystems.spindexer.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.lobby.subsystems.spindexer.Spindexer;

public class Agitate extends Command
{
    private final Spindexer spindexer;
    private double speed;
    private final Time switchInterval;
    private final Timer timer;

    public Agitate(Spindexer spindexer, double speed, Time switchInterval)
    {
        addRequirements(spindexer);
        this.spindexer = spindexer;
        this.speed = speed;
        this.switchInterval = switchInterval;
        timer = new Timer();
        timer.stop();
        timer.reset();
    }

    public Agitate(Spindexer spindexer)
    {
        this(spindexer, 0.1, Seconds.of(0.25));
        timer.start();
    }

    @Override
    public void initialize()
    {
        spindexer.getCarousel().setPower(speed);
        spindexer.getCarousel().startCarousel();
        spindexer.getFlicker().stopFlicker();

    }

    @Override
    public void execute()
    {
        if (timer.hasElapsed(switchInterval))
        {
            speed = -speed;
            spindexer.getCarousel().setPower(speed);
            spindexer.getCarousel().startCarousel();
            timer.restart();
        }
    }
}
