package frc.robot.ralph;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;

public class RalphOI
{
    private CommandXboxController driverController = new CommandXboxController(0);
    private CommandXboxController manipulatorController = new CommandXboxController(1);

    public void bind(RalphContainer container)
    {
        manipulatorController.povLeft().onTrue(Commands.run(
                () -> container.getSuperstructure().setState(SuperstructureState.L1), container.getSuperstructure()));
        manipulatorController.povLeft()
                .onFalse(Commands.run(() -> container.getSuperstructure().setState(SuperstructureState.MANUAL)));
        manipulatorController.povUp().onTrue(Commands.run(
                () -> container.getSuperstructure().setState(SuperstructureState.L2), container.getSuperstructure()));
        manipulatorController.povUp()
                .onFalse(Commands.run(() -> container.getSuperstructure().setState(SuperstructureState.MANUAL)));
        manipulatorController.povRight().onTrue(Commands.run(
                () -> container.getSuperstructure().setState(SuperstructureState.L3), container.getSuperstructure()));
        manipulatorController.povRight()
                .onFalse(Commands.run(() -> container.getSuperstructure().setState(SuperstructureState.MANUAL)));
        manipulatorController.povDown().onTrue(Commands.run(
                () -> container.getSuperstructure().setState(SuperstructureState.L4), container.getSuperstructure()));
        manipulatorController.povDown()
                .onFalse(Commands.run(() -> container.getSuperstructure().setState(SuperstructureState.MANUAL)));
        manipulatorController.a()
                .whileTrue(Commands.run(() -> container.getSuperstructure().setState(SuperstructureState.CORAL_STATION),
                        container.getSuperstructure()));
        manipulatorController.a()
                .onFalse(Commands.run(() -> container.getSuperstructure().setState(SuperstructureState.MANUAL)));
    }
}
