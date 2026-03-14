package frc.robot.lobby.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lobby.subsystems.climber.Climber;

public class HomeCommand extends Command
{

    private Climber climber;

    public HomeCommand(Climber climber)
    {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute()
    {
        climber.homeDown();
    }

    @Override
    public boolean isFinished()
    {
        return climber.isAtBottom();
    }

    @Override
    public void end(boolean interupted)
    {
        climber.stopMotor();
    }
}
