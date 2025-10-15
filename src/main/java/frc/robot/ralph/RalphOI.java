package frc.robot.ralph;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;

import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import java.util.function.DoubleSupplier;

public class RalphOI
{
    private static DoubleSupplier inputProc(DoubleSupplier input)
    {
        return () ->
        {
            double x = MathUtil.applyDeadband(input.getAsDouble(), 0.1, 1);
            return -x * Math.abs(x);
        };
    }

    public void bind(RalphContainer container)
    {
        var driveController = new CommandXboxController(0);
        var manipulatorController = new CommandXboxController(1);

        var drive = container.getDrive();

        drive.setDefaultCommand(drive.driveByJoystick(inputProc(driveController::getLeftX),
                inputProc(driveController::getLeftY), inputProc(driveController::getRightX)));
        driveController.back().onTrue(drive.resetOrientation());

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
