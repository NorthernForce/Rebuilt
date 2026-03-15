package frc.robot.lobby;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
        var spindexer = container.getSpindexer();
        var suzie = container.getTurret().getSuzie();
        var hood = container.getTurret().getHood();
        var shooter = container.getTurret().getShooter();

        drive.setDefaultCommand(drive.driveByJoystick(inputProc(driveController::getLeftY),
                inputProc(driveController::getLeftX), inputProc(driveController::getRightX)));
        intake.setDefaultCommand(intake.stopIntake().andThen(intake.getRunToIntakeAngleCommand()));
        // turret.setDefaultCommand(new AimTurretCommand(() -> container.predictPose(),
        // turret));
        // spindexer.setDefaultCommand(new Agitate(spindexer));
        // turret.setDefaultCommand(container.getTurret().runBasedOnLocation(() ->
        // drive.getPose(),
        // hood.getDangerZone(), hood.getTrenchPositions()));
        // shooter.setDefaultCommand(Commands.run(() -> shooter.stop(), shooter));

        driveController.back().onTrue(drive.resetOrientation());
        driveController.x().toggleOnTrue(intake.stopIntake().andThen(intake.getRunToStowAngleCommand()).repeatedly());

        driveController.leftTrigger().whileTrue(intake.intakeMoving()).onFalse(intake.stopIntake());
        driveController.rightTrigger().whileTrue((Commands
                .waitUntil(() -> turret.getSuzie().isAtTargetAngle() && turret.getShooter().isAtTargetSpeed())
                .andThen(new RunSpindexer(container.getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime))
                .alongWith(new PrepTurretCommand(container))));

        driveController.start().onTrue(Commands.runOnce(() -> suzie.resetCRT()));

        manipulatorController.leftStick().whileTrue(intake.driveByJoystick(() -> manipulatorController.getLeftY()));

        driveController.rightBumper()
                .whileTrue(new PrepTurretWithValues(turret).alongWith(Commands.waitSeconds(0.5).andThen(
                        new RunSpindexer(container.getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime))))
                .onFalse(Commands.runOnce(() -> turret.stop()));

        // driveController.rightBumper().whileTrue(Commands.waitSeconds(0.25)
        // .andThen(new RunSpindexer(container.getSpindexer(),
        // LobbyConstants.SpindexerConstants.kDeJamTime))
        // .alongWith(new PrepTurretStupid(container)));

        manipulatorController.leftTrigger().whileTrue(intake.intakeMoving()).onFalse(intake.stopIntake());
        manipulatorController.rightTrigger().whileTrue(Commands.waitSeconds(0.25)
                .andThen(new RunSpindexer(container.getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime))
                .alongWith(new PrepTurretStupid(container)));

        // manipulatorController.b().onTrue(Commands.runOnce(() ->
        // {
        // DogLog.log("Turret/csvValue",
        // turret.getHoodTargetingCalculator().getValueForDistance(5.0));
        // }));

        // driveController.leftTrigger().whileTrue(new
        // PrepTurretWithValues(container.getTurret()));
        driveController.povUp().whileTrue(container.getClimber().runUp())
                .onFalse(Commands.runOnce(() -> container.getClimber().stopMotor(), container.getClimber()));
        // Temporary test: hold D-pad down to command elevator up, release to home
        driveController.povDown().whileTrue(container.getClimber().runDown())
                .onFalse(Commands.runOnce(() -> container.getClimber().stopMotor(), container.getClimber()));
        // manipulatorController.leftTrigger().whileTrue(intake.getRunToIntakeAngleCommand());
        // intake.setDefaultCommand(intake.getRunToStowAngleCommand());

        // manipulatorController.rightTrigger().whileTrue(new
        // RunSpindexer(container.getSpindexer()));
        // manipulatorController.leftBumper().whileTrue(new
        // PrepTurretWithValues(turret));

        // manipulatorController.a().onTrue(Commands.runOnce(() -> suzie.resetAngle()));

        driveController.povLeft().whileTrue(Commands.runOnce(() -> suzie.setSpeed(0.2), suzie))
                .onFalse(Commands.runOnce(() -> suzie.setSpeed(0), suzie));
        driveController.povRight().whileTrue(Commands.runOnce(() -> suzie.setSpeed(-0.2), suzie))
                .onFalse(Commands.runOnce(() -> suzie.setSpeed(0), suzie));
    }
}
