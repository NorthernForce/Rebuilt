package frc.robot.ralph;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
        
        container.getDrive()
                .setDefaultCommand(container.getDrive().driveByJoystick(inputProc(driveController::getLeftX),
                        inputProc(driveController::getLeftY), inputProc(driveController::getRightX)));
    }
}
