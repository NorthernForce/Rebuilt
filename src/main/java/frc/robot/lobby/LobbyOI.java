package frc.robot.lobby;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.lobby.subsystems.spindexer.commands.RunSpindexer;
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
        var turret = container.getTurret();
        var suzie = container.getTurret().getSuzie();
        var spindexer = container.getSpindexer();
        var hood = container.getTurret().getHood();
        var shooter = container.getTurret().getShooter();
        var leds = container.getLeds();

        drive.setDefaultCommand(drive.driveByJoystick(inputProc(driveController::getLeftY),
                inputProc(driveController::getLeftX), inputProc(driveController::getRightX)));
        intake.setDefaultCommand(intake.stopIntake().andThen(intake.getRunToIntakeAngleCommand()));
        // turret.setDefaultCommand(new AimTurretCommand(container));
        turret.setDefaultCommand(turret.runBasedOnLocation(() -> drive.getPose(),
                LobbyConstants.Turret.Hood.kDangerZone, LobbyConstants.Turret.Hood.kAllTrenchPositions));
        // spindexer.setDefaultCommand(new Agitate(spindexer));
        // turret.setDefaultCommand(container.getTurret().runBasedOnLocation(() ->
        // drive.getPose(),
        // hood.getDangerZone(), hood.getTrenchPositions()));
        // shooter.setDefaultCommand(Commands.run(() -> shooter.stop(), shooter));

        driveController.back().onTrue(drive.resetOrientation());
        driveController.x().toggleOnTrue(intake.stopIntake().andThen(intake.getRunToStowAngleCommand()).repeatedly());
        driveController.y().toggleOnTrue((Commands
                .waitUntil(() -> turret.getSuzie().isAtTargetAngle() && turret.getShooter().isAtTargetSpeed())
                .andThen(new RunSpindexer(container.getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime,
                        LobbyConstants.SpindexerConstants.kPostDeJamTime, () -> turret.isAtTargetPose()))
                .alongWith(new PrepTurretCommand(container))));
        driveController.b().whileTrue(Commands
                .parallel(container.getClimber().runUp().withTimeout(6), container.driveToPreClimbPosition())
                .andThen(container.driveToClimbPost()).andThen(container.getClimber().runDown().withTimeout(8)));
        driveController.leftTrigger().whileTrue(intake.intakeMoving()).onFalse(intake.stopIntake());

        driveController.rightTrigger().whileTrue((Commands
                .waitUntil(() -> turret.getSuzie().isAtTargetAngle() && turret.getShooter().isAtTargetSpeed())
                .andThen(new RunSpindexer(container.getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime,
                        LobbyConstants.SpindexerConstants.kPostDeJamTime, () -> turret.isAtTargetPose()))
                .alongWith(new PrepTurretCommand(container)))
                .alongWith(Commands.waitSeconds(1.0).andThen(intake.pump())));

        driveController.start().onTrue(Commands.runOnce(() -> suzie.resetAngle()));

        driveController.leftBumper()
                .whileTrue(new PrepTurretWithValues(turret, RotationsPerSecond.of(100), Degrees.of(21))
                        .alongWith(Commands.waitSeconds(0.5)
                                .andThen(new RunSpindexer(container.getSpindexer(),
                                        LobbyConstants.SpindexerConstants.kDeJamTime,
                                        LobbyConstants.SpindexerConstants.kPostDeJamTime,
                                        () -> turret.isAtTargetPoseStupid()))))
                .onFalse(Commands.runOnce(() -> turret.stop()));

        driveController.rightBumper().whileTrue((Commands
                .waitUntil(() -> turret.getSuzie().isAtTargetAngle() && turret.getShooter().isAtTargetSpeed())
                .andThen(new RunSpindexer(container.getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime,
                        LobbyConstants.SpindexerConstants.kPostDeJamTime, () -> turret.isAtTargetPose()))
                .alongWith(new PrepTurretCommand(container))));

        driveController.povUp().whileTrue(container.getClimber().runUp())
                .onFalse(Commands.runOnce(() -> container.getClimber().stopMotor(), container.getClimber()));
        driveController.povDown().whileTrue(container.getClimber().runDown())
                .onFalse(Commands.runOnce(() -> container.getClimber().stopMotor(), container.getClimber()));
        driveController.a().onTrue(Commands.runOnce(() -> turret.resetTrim()));
        driveController.povLeft().onTrue(Commands.runOnce(() -> suzie.start(), suzie))
                .onFalse(Commands.runOnce(() -> suzie.stop(), suzie))
                .whileTrue(Commands.sequence(Commands
                        .runOnce(() -> turret.setOffsetAngle(turret.getOffsetAngle().minus(Degrees.of(1))), turret),
                        Commands.waitSeconds(0.02)).repeatedly());
        driveController.povRight().onTrue(Commands.runOnce(() -> suzie.start(), suzie))
                .onFalse(Commands.runOnce(() -> suzie.stop(), suzie))
                .whileTrue(Commands.sequence(Commands
                        .runOnce(() -> turret.setOffsetAngle(turret.getOffsetAngle().plus(Degrees.of(1))), turret),
                        Commands.waitSeconds(0.02)).repeatedly());

        manipulatorController.back().onTrue(drive.resetOrientation());
        manipulatorController.x()
                .toggleOnTrue(intake.stopIntake().andThen(intake.getRunToStowAngleCommand()).repeatedly());
        manipulatorController.y().toggleOnTrue((Commands
                .waitUntil(() -> turret.getSuzie().isAtTargetAngle() && turret.getShooter().isAtTargetSpeed())
                .andThen(new RunSpindexer(container.getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime,
                        LobbyConstants.SpindexerConstants.kPostDeJamTime, () -> turret.isAtTargetPose()))
                .alongWith(new PrepTurretCommand(container))));

        manipulatorController.leftTrigger().whileTrue(intake.intakeMoving()).onFalse(intake.stopIntake());

        manipulatorController.rightTrigger().whileTrue((Commands
                .waitUntil(() -> turret.getSuzie().isAtTargetAngle() && turret.getShooter().isAtTargetSpeed())
                .andThen(new RunSpindexer(container.getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime,
                        LobbyConstants.SpindexerConstants.kPostDeJamTime, () -> turret.isAtTargetPose()))
                .alongWith(new PrepTurretCommand(container)))
                .alongWith(Commands.waitSeconds(1.0).andThen(intake.pump())));

        manipulatorController.start().onTrue(Commands.runOnce(() -> suzie.resetAngle()));

        manipulatorController.leftBumper()
                .whileTrue(new PrepTurretWithValues(turret, RotationsPerSecond.of(120), Degrees.of(21))
                        .alongWith(Commands.waitSeconds(0.5)
                                .andThen(new RunSpindexer(container.getSpindexer(),
                                        LobbyConstants.SpindexerConstants.kDeJamTime,
                                        LobbyConstants.SpindexerConstants.kPostDeJamTime, () ->
                                        {
                                            return true;
                                        }))))
                .onFalse(Commands.runOnce(() -> turret.stop()));

        manipulatorController.rightBumper().whileTrue((Commands
                .waitUntil(() -> turret.getSuzie().isAtTargetAngle() && turret.getShooter().isAtTargetSpeed())
                .andThen(new RunSpindexer(container.getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime,
                        LobbyConstants.SpindexerConstants.kPostDeJamTime, () -> turret.isAtTargetPose()))
                .alongWith(new PrepTurretCommand(container))));

        manipulatorController.povUp().whileTrue(container.getClimber().runUp())
                .onFalse(Commands.runOnce(() -> container.getClimber().stopMotor(), container.getClimber()));
        manipulatorController.povDown().whileTrue(container.getClimber().runDown())
                .onFalse(Commands.runOnce(() -> container.getClimber().stopMotor(), container.getClimber()));

        manipulatorController.povLeft().whileTrue(Commands.run(() -> suzie.setSpeed(0.1), suzie))
                .onFalse(Commands.runOnce(() -> suzie.setSpeed(0), suzie));
        manipulatorController.povRight().whileTrue(Commands.run(() -> suzie.setSpeed(-0.1), suzie))
                .onFalse(Commands.runOnce(() -> suzie.setSpeed(0), suzie));
    }
}
