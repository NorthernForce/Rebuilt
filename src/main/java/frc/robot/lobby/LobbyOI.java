package frc.robot.lobby;

import java.util.function.DoubleSupplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.turret.commands.PrepTurretCommand;

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

        drive.setDefaultCommand(drive.driveByJoystick(inputProc(driveController::getLeftY),
                inputProc(driveController::getLeftX), inputProc(driveController::getRightX)));
        driveController.back().onTrue(drive.resetOrientation());

        manipulatorController.a()
                .whileTrue(new PrepTurretCommand(() -> container.getDrive().getState().Pose, container.getTurret()));
        manipulatorController.b().onTrue(Commands.runOnce(() ->
        {
            // container.getTurret().getHoodTargetingCalculator().addData(5.0, 1.0);
            DogLog.log("Turret/csvValue", container.getTurret().getHoodTargetingCalculator().getValueForDistance(5.0));
        }));
    }
}
