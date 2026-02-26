package frc.robot.lobby;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.Set;
import java.util.function.DoubleSupplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lobby.subsystems.spindexer.Spindexer;
import frc.robot.lobby.subsystems.spindexer.commands.RunSpindexer;
import frc.robot.lobby.subsystems.spindexer.flicker.FlickerIO;
import frc.robot.lobby.subsystems.turret.commands.PrepTurretCommand;
import frc.robot.lobby.subsystems.turret.commands.PrepTurretWithValues;

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

        if (DriverStation.isTest())
        {
            driveController.rightBumper().whileTrue(
                    Commands.run(() -> container.getSpindexer().getCarousel().setPower(1), container.getSpindexer()));

            container.getTurret().getShooter().start();

            driveController.rightTrigger().whileTrue(Commands
                    .run(() -> container.getSpindexer().getFlicker().setPower(1), container.getTurret()).alongWith(
                            Commands.run(() -> container.getSpindexer().getCarousel().setPower(1),
                                    container.getSpindexer()),
                            Commands.run(() -> container.getTurret().getShooter().start(), container.getTurret())))
                    .onFalse(
                            Commands.run(() -> container.getSpindexer().getFlicker().setPower(0), container.getTurret())
                                    .alongWith(
                                            Commands.run(() -> container.getSpindexer().getCarousel().setPower(0),
                                                    container.getSpindexer()),
                                            Commands.run(() -> container.getTurret().getShooter().stop(),
                                                    container.getTurret())));

            drive.setDefaultCommand(drive.driveByJoystick(inputProc(driveController::getLeftY),
                    inputProc(driveController::getLeftX), inputProc(driveController::getRightX)));

            driveController.povUp().whileTrue(Commands.run(() -> container.getTurret().getHood().setSpeed(0.2, false)))
                    .onFalse(Commands.runOnce(() -> container.getTurret().getHood().setSpeed(0, false)));

            driveController.povDown()
                    .whileTrue(Commands.run(() -> container.getTurret().getHood().setSpeed(-0.2, false)))
                    .onFalse(Commands.runOnce(() -> container.getTurret().getHood().setSpeed(0, false)));

            driveController.leftBumper().whileTrue(container.getIntake().getRunToIntakeAngleCommand())
                    .onFalse(container.getIntake().getRunToStowAngleCommand());

            driveController.povLeft().whileTrue(container.getTurret().getSuzie().setSpeed(0.05))
                    .onFalse(container.getTurret().getSuzie().setSpeed(0));

            driveController.povRight().whileTrue(container.getTurret().getSuzie().setSpeed(-0.05))
                    .onFalse(container.getTurret().getSuzie().setSpeed(0));

        } else
        {
            drive.setDefaultCommand(drive.driveByJoystick(inputProc(driveController::getLeftY),
                    inputProc(driveController::getLeftX), inputProc(driveController::getRightX)));

            driveController.back().onTrue(drive.resetOrientation());

            var hood = container.getTurret().getHood();

            driveController.a().onTrue(Commands.runOnce(() -> container
                    .resetOdometry(new Pose2d(Meters.of(0), Meters.of(0), new Rotation2d(Degrees.of(180))))));

            manipulatorController.b().onTrue(Commands.runOnce(() ->
            {
                DogLog.log("Turret/csvValue",
                        container.getTurret().getHoodTargetingCalculator().getValueForDistance(5.0));
            }));

            (new Trigger(() -> container.getSpindexer().getJammed())).whileTrue(
                    Commands.defer(() -> container.getSpindexer().runBackwards(), Set.of(container.getSpindexer())));

            driveController.povUp()
                    .onTrue(Commands.runOnce(() -> hood.setTargetAngle(Degrees.of(180)), container.getTurret()))
                    .onFalse(Commands.runOnce(() -> hood.setTargetAngle(Degrees.zero()), container.getTurret()));

            manipulatorController.b().onTrue(Commands.runOnce(() ->
            {
                DogLog.log("Turret/csvValue",
                        container.getTurret().getHoodTargetingCalculator().getValueForDistance(5.0));
            }));

            driveController.rightTrigger().whileTrue(
                    new RunSpindexer(container.getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime));

            driveController.leftTrigger().whileTrue(intake.intake(0.75)).onFalse(intake.stopIntake());

            container.getTurret().getShooter().setDefaultCommand(container.getTurret().getShooter().stop());

            driveController.povLeft().whileTrue(container.getTurret().getSuzie().setSpeed(0.05))
                    .onFalse(container.getTurret().getSuzie().setSpeed(0));

            driveController.povRight().whileTrue(container.getTurret().getSuzie().setSpeed(-0.05))
                    .onFalse(container.getTurret().getSuzie().setSpeed(0));
        }
    }
}
