package frc.robot.ralph;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Base class for IO
 */

public class RalphOI
{
    private CommandXboxController driverController = new CommandXboxController(0);
    private CommandXboxController manipulatorController = new CommandXboxController(1);

    public void bind(RalphContainer container)
    {
        manipulatorController.leftTrigger().whileTrue(container.getShooter().getIntakeCommand());
        manipulatorController.rightTrigger().whileTrue(container.getShooter().getOuttakeCommand());
        driverController.leftTrigger().whileTrue(container.getShooter().getIntakeCommand());
        driverController.rightTrigger().whileTrue(container.getShooter().getOuttakeCommand());
    }
}
