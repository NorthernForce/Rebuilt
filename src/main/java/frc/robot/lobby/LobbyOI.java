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
        var intake = container.getIntake();

        drive.setDefaultCommand(drive.driveByJoystick(inputProc(driveController::getLeftY),
                inputProc(driveController::getLeftX), inputProc(driveController::getRightX)));
        driveController.back().onTrue(drive.resetOrientation());

        manipulatorController.rightBumper().whileTrue(intake.intake(LobbyConstants.IntakeConstants.kDriverIntakeSpeed))
                .onFalse(intake.stopIntake());
        manipulatorController.leftBumper()
                .whileTrue(intake.purgeIntake(LobbyConstants.IntakeConstants.kDriverPurgeSpeed))
                .onFalse(intake.stopIntake());
    }
}
