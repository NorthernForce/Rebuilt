package frc.robot.lobby;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class LobbyOI
{
    private static DoubleSupplier inputProc(DoubleSupplier input)
    {
        return () ->
        {
            double x = MathUtil.applyDeadband(input.getAsDouble(), 0.1, 1);
            return -x * Math.abs(x);
        };
    }

    public void bind(LobbyContainer container)
    {
        var driveController = new CommandXboxController(0);
        var manipulatorController = new CommandXboxController(1);

        var drive = container.getDrive();
        var climber = container.getClimber(); // Get the climber
        // Drive Bindings
        drive.setDefaultCommand(drive.driveByJoystick(inputProc(driveController::getLeftY),
                inputProc(driveController::getLeftX), inputProc(driveController::getRightX)));
        driveController.back().onTrue(drive.resetOrientation());

        // climber bindings
        climber.setDefaultCommand(climber.stow(() -> climber.getInputs().positionMeters < 0.1));

        // Climb Sequence
        manipulatorController.povUp().whileTrue(climber.climbSequence());
    }
}
