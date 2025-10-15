package frc.robot.ralph;

import org.northernforce.util.NFRRobotContainer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ralph.subsystems.shooter.ShooterIOTalonFX;

public class RalphContainer implements NFRRobotContainer
{
    private final ShooterIOTalonFX shooter;

    public RalphContainer()
    {
        shooter = new ShooterIOTalonFX(RalphConstants.ShooterConstants.kMotorId,
                RalphConstants.ShooterConstants.kMotorSpeed);
        bindOI();
    }

    @Override
    public void bindOI()
    {
        new RalphOI().bind(this);
    }

    public ShooterIOTalonFX getShooter()
    {
        return shooter;
    }

    @Override
    public Command getAutonomousCommand()
    {
        return Commands.none();
    }

}
