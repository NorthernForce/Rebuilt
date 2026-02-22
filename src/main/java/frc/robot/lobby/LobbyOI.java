package frc.robot.lobby;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.Set;
import java.util.function.DoubleSupplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.turret.commands.PrepTurretCommand;
import frc.robot.subsystems.turret.commands.PrepTurretWithValues;
import frc.robot.lobby.subsystems.spindexer.Spindexer;
import frc.robot.lobby.subsystems.spindexer.commands.DejamSpindexer;
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
        var intake = container.getIntake();

        drive.setDefaultCommand(drive.driveByJoystick(inputProc(driveController::getLeftY),
                inputProc(driveController::getLeftX), inputProc(driveController::getRightX)));
        driveController.back().onTrue(drive.resetOrientation());

        // Pass turret's hood constants for danger zone calculation
        var hood = container.getTurret().getHood();
        // container.getTurret().setDefaultCommand(container.getTurret().runBasedOnLocation(()
        // -> drive.getState().Pose,
        // hood.getDangerZone(), hood.getTrenchPositions()));

        // Log when trigger is pressed to verify controller works
        driveController.a().onTrue(Commands.runOnce(() -> container
                .resetOdometry(new Pose2d(Meters.of(0), Meters.of(0), new Rotation2d(Degrees.of(180))))));

        manipulatorController.b().onTrue(Commands.runOnce(() ->
        {
            DogLog.log("Turret/csvValue", container.getTurret().getHoodTargetingCalculator().getValueForDistance(5.0));
        }));

        driveController.rightTrigger().whileTrue(new RunSpindexer(container.getSpindexer()));
        // driveController.leftTrigger().whileTrue(new
        // PrepTurretWithValues(container.getTurret()));
        // driveController.povUp()
        // .onTrue(Commands.runOnce(() -> hood.setTargetAngle(Degrees.of(180)),
        // container.getTurret()))
        // .onFalse(Commands.runOnce(() -> hood.setTargetAngle(Degrees.zero()),
        // container.getTurret()));
        // manipulatorController.leftTrigger().whileTrue(intake.getRunToIntakeAngleCommand());
        // intake.setDefaultCommand(intake.getRunToStowAngleCommand());
        manipulatorController.leftTrigger().whileTrue(intake.intake(0.75)).onFalse(intake.stopIntake());
        manipulatorController.rightTrigger().whileTrue(
                new RunSpindexer(container.getSpindexer()).alongWith(new PrepTurretWithValues(container.getTurret())));
        // manipulatorController.rightTrigger().whileTrue(new
        // RunSpindexer(container.getSpindexer()));
        // manipulatorController.leftBumper().whileTrue(new
        // PrepTurretWithValues(container.getTurret()));
        driveController.a().onTrue(Commands.runOnce(() -> container
                .resetOdometry(new Pose2d(Meters.of(0), Meters.of(0), new Rotation2d(Degrees.of(180))))));

        manipulatorController.povLeft().whileTrue(container.getTurret().getSuzie().setSpeed(0.05))
                .onFalse(container.getTurret().getSuzie().setSpeed(0));
        manipulatorController.povRight().whileTrue(container.getTurret().getSuzie().setSpeed(-0.05))
                .onFalse(container.getTurret().getSuzie().setSpeed(0));
    }
}
