package frc.robot.ralph;

import org.northernforce.util.NFRRobotContainer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ralph.generated.RalphTunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RalphContainer implements NFRRobotContainer
{
    private final CommandSwerveDrivetrain drive;

    public RalphContainer()
    {
        drive = new CommandSwerveDrivetrain(RalphTunerConstants.DrivetrainConstants, RalphTunerConstants.FrontLeft,
                RalphTunerConstants.FrontRight, RalphTunerConstants.BackLeft, RalphTunerConstants.BackRight);
    }

    /**
     * gets the drive subsystem
     *
     * @return the drive subsystem
     */
    public CommandSwerveDrivetrain getDrive()
    {
        return drive;
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

}
