package frc.robot.ralph;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RalphOI
{
    public void bind(RalphContainer container)
    {
        var driveController = new CommandXboxController(0);
        var manipController = new CommandXboxController(1);

        Function<DoubleSupplier, DoubleSupplier> inputProc = input -> () -> MathUtil.applyDeadband(input.getAsDouble(),
                0.1, 1);
        container.getDrive()
                .setDefaultCommand(container.getDrive().driveByJoystick(inputProc.apply(driveController::getLeftX),
                        inputProc.apply(driveController::getLeftY), inputProc.apply(driveController::getRightX),
                        RalphConstants.DrivetrainConstants.kMaxSpeed,
                        RalphConstants.DrivetrainConstants.kMaxAngularSpeed));
    }
}
