package frc.robot.lobby.subsystems.spindexer.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lobby.subsystems.spindexer.Spindexer;

public class DejamSpindexer extends Command
{
    private Timer runSeconds = new Timer();
    private Spindexer spindexer;

    public DejamSpindexer(Spindexer spindexer)
    {
        this.spindexer = spindexer;
        addRequirements(spindexer);
    }

    @Override
    public void initialize()
    {
        runSeconds.start();
        spindexer.getCarousel().dejam();
        spindexer.getFlicker().dejam();
    }

    @Override
    public boolean isFinished()
    {
        return runSeconds.hasElapsed(0.5);
    }

    @Override
    public void end(boolean interrupted)
    {
        spindexer.getCarousel().stopCarousel();
        spindexer.getCarousel().resetJamDetection();
        spindexer.getFlicker().stopFlicker();
        spindexer.getFlicker().resetJamDetection();
    }
}
