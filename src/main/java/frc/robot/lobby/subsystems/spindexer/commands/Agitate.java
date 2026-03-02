package frc.robot.lobby.subsystems.spindexer.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.lobby.subsystems.spindexer.Spindexer;

public class Agitate extends Command {
    private final Spindexer spindexer;
    private final double speed;

    public Agitate(Spindexer spindexer, double speed)
    {
        addRequirements(spindexer);
        this.spindexer = spindexer;
        this.speed = speed;
    }

    public Agitate(Spindexer spindexer)
    {
        this(spindexer, 0.1);
    }

    @Override
    public void initialize()
    {
        spindexer.getCarousel().setPower(speed);
        spindexer.getFlicker().stopFlicker();
    }
}
