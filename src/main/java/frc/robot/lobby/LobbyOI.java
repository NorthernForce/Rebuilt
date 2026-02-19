package frc.robot.lobby;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.turret.commands.PrepTurretCommand;
import frc.robot.lobby.subsystems.spindexer.commands.RunSpindexer;

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

        // Pass turret's hood constants for danger zone calculation
        var hood = container.getTurret().getHood();
        container.getTurret().setDefaultCommand(container.getTurret().runBasedOnLocation(() -> drive.getState().Pose,
                hood.getDangerZone(), hood.getTrenchPositions()));

        // Log when trigger is pressed to verify controller works
        driveController.a().onTrue(Commands.runOnce(() -> container
                .resetOdometry(new Pose2d(Meters.of(0), Meters.of(0), new Rotation2d(Degrees.of(180))))));

        manipulatorController.b().onTrue(Commands.runOnce(() ->
        {
            DogLog.log("Turret/csvValue", container.getTurret().getHoodTargetingCalculator().getValueForDistance(5.0));
        }));
        manipulatorController.rightTrigger().whileTrue(new RunSpindexer(container.getSpindexer())
                .alongWith(new PrepTurretCommand(() -> container.getDrive().getState().Pose, container.getTurret())));
        driveController.a().onTrue(Commands.runOnce(() -> container
                .resetOdometry(new Pose2d(Meters.of(0), Meters.of(0), new Rotation2d(Degrees.of(180))))));
    }
}
