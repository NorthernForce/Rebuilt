package frc.robot.ralph;

import org.northernforce.util.NFRRobotContainer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RalphContainer implements NFRRobotContainer
{

    @Override
    public void bindOI()
    {
        new RalphOI().bind(this);
    }

    @Override
    public Command getAutonomousCommand()
    {
        return Commands.none();
    }

}
