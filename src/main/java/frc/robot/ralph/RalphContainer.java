package frc.robot.ralph;

import org.northernforce.util.NFRRobotContainer;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ralph.RalphConstants.InnerElevatorConstants;
import frc.robot.ralph.RalphConstants.OuterElevatorConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.elevator.SimElevator;

public class RalphContainer implements NFRRobotContainer
{
    private final Superstructure superstructure;

    public RalphContainer()
    {
        if (RobotBase.isReal())
        {
            superstructure = new Superstructure();
        } else
        {
            superstructure = new Superstructure(
                    new SimElevator(InnerElevatorConstants.kCanID, InnerElevatorConstants.kSensorID,
                            InnerElevatorConstants.kConfig),
                    new SimElevator(OuterElevatorConstants.kCanID, OuterElevatorConstants.kSensorID,
                            OuterElevatorConstants.kConfig));
        }
    }

    @Override
    public void periodic()
    {

    }

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

    public Superstructure getSuperstructure()
    {
        return superstructure;
    }
}
