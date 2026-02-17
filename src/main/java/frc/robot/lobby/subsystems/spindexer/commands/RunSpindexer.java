package frc.robot.lobby.subsystems.spindexer.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lobby.subsystems.spindexer.Spindexer;

public class RunSpindexer extends SequentialCommandGroup
{
    public RunSpindexer(Spindexer spindexer)
    {
        addRequirements(spindexer);
        addCommands(new RampFlicker(spindexer),
                new RunCarousel(spindexer).alongWith(new RampFlickerContinuous(spindexer)));
    }
}
