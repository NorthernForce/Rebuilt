package frc.robot.subsystems.climber.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class RetractClimber extends Command
{
    private final Climber climber;

    public RetractClimber(Climber climber)
    {
        addRequirements(climber);
        this.climber = climber;
    }

    @Override
    public void initialize()
    {
        climber.setState(Climber.ClimberState.RETRACT);
    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}