package frc.robot.lobby;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lobby.subsystems.spindexer.commands.RunSpindexer;
import frc.robot.lobby.subsystems.turret.commands.PrepTurretCommand;
import frc.robot.lobby.subsystems.turret.commands.PrepTurretStupid;
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

        driveController.leftTrigger().whileTrue(intake.intakeMoving()).onFalse(intake.stopIntake());
        driveController.leftBumper().whileTrue(intake.purgeIntake()).onFalse(intake.stopIntake());

        driveController.rightTrigger().whileTrue((Commands
                .waitUntil(() -> turret.getSuzie().isAtTargetAngle() && turret.getShooter().isAtTargetSpeed())
                .andThen(new RunSpindexer(container.getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime,
                        LobbyConstants.SpindexerConstants.kPostDeJamTime, () -> turret.isAtTargetPose()))
                .alongWith(new PrepTurretCommand(container)))
                .alongWith(Commands.waitSeconds(1.0).andThen(intake.pump())));

        driveController.start().onTrue(Commands.runOnce(() -> suzie.resetCRT()));

        driveController.rightBumper().whileTrue(new PrepTurretWithValues(turret).alongWith(Commands.waitSeconds(0.5)
                .andThen(new RunSpindexer(container.getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime,
                        LobbyConstants.SpindexerConstants.kPostDeJamTime, () -> turret.isAtTargetPoseStupid()))))
                .onFalse(Commands.runOnce(() -> turret.stop()));

        manipulatorController.leftTrigger().whileTrue(intake.intakeMoving()).onFalse(intake.stopIntake());
        manipulatorController.rightTrigger().whileTrue(Commands.waitSeconds(0.25)
                .andThen(new RunSpindexer(container.getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime,
                        LobbyConstants.SpindexerConstants.kPostDeJamTime, () -> turret.isAtTargetPose()))
                .alongWith(new PrepTurretStupid(container, () -> container.getTurret()
                        .calculateFieldRelativeShooterPosition(container.getDrive().getPose()))));

        driveController.povUp().whileTrue(container.getClimber().runUp())
                .onFalse(Commands.runOnce(() -> container.getClimber().stopMotor(), container.getClimber()));
        driveController.povDown().whileTrue(container.getClimber().runDown())
                .onFalse(Commands.runOnce(() -> container.getClimber().stopMotor(), container.getClimber()));

        driveController.povLeft().whileTrue(Commands.run(() -> suzie.setSpeed(0.1), suzie))
                .onFalse(Commands.runOnce(() -> suzie.setSpeed(0), suzie));
        driveController.povRight().whileTrue(Commands.run(() -> suzie.setSpeed(-0.1), suzie))
                .onFalse(Commands.runOnce(() -> suzie.setSpeed(0), suzie));
    }
}
