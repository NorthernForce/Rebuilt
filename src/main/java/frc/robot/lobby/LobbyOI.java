package frc.robot.lobby;

import static edu.wpi.first.units.Units.Degrees;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.lobby.subsystems.spindexer.commands.Agitate;
import frc.robot.lobby.subsystems.spindexer.commands.RunSpindexer;
import frc.robot.lobby.subsystems.turret.commands.PrepTurretCommand;

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
        intake.setDefaultCommand(intake.stopIntake().andThen(intake.getRunToMidAngleCommand()));
        // spindexer.setDefaultCommand(new Agitate(spindexer));
        // turret.setDefaultCommand(container.getTurret().runBasedOnLocation(() ->
        // drive.getState().Pose,
        // hood.getDangerZone(), hood.getTrenchPositions()));
        shooter.setDefaultCommand(Commands.run(() -> shooter.stop(), shooter));

        driveController.back().onTrue(drive.resetOrientation());

        driveController.leftTrigger().whileTrue(intake.intakeMoving()).onFalse(intake.stopIntake());
        driveController.rightTrigger().whileTrue(Commands
                .waitUntil(() -> turret.getSuzie().isAtTargetAngle() && turret.getShooter().isAtTargetSpeed())
                .andThen(new RunSpindexer(container.getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime))
                .alongWith(new PrepTurretCommand(() -> drive.getPose(), turret)));

        manipulatorController.leftStick().whileTrue(intake.driveByJoystick(() -> manipulatorController.getLeftY()));

        manipulatorController.leftTrigger().whileTrue(intake.intakeMoving()).onFalse(intake.stopIntake());
        manipulatorController.rightTrigger().whileTrue(Commands.waitSeconds(0.25)
                .andThen(new RunSpindexer(container.getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime))
                .alongWith(new PrepTurretCommand(() -> drive.getPose(), turret)));

        // manipulatorController.b().onTrue(Commands.runOnce(() ->
        // {
        // DogLog.log("Turret/csvValue",
        // turret.getHoodTargetingCalculator().getValueForDistance(5.0));
        // }));

        // driveController.leftTrigger().whileTrue(new
        // PrepTurretWithValues(turret));
        driveController.povUp()
                .onTrue(hood.setTargetMechanismAngle(Degrees.of(28.5))
                        .andThen(Commands.runOnce(() -> hood.start(), turret.getHood())))
                .onFalse(hood.setTargetMechanismAngle(Degrees.of(20))
                        .andThen(Commands.runOnce(() -> hood.start(), turret.getHood())));
        driveController.povDown().onTrue(Commands.runOnce(() -> hood.setZeroLatch()));
        // manipulatorController.leftTrigger().whileTrue(intake.getRunToIntakeAngleCommand());
        // intake.setDefaultCommand(intake.getRunToStowAngleCommand());

        // manipulatorController.rightTrigger().whileTrue(new
        // RunSpindexer(container.getSpindexer()));
        // manipulatorController.leftBumper().whileTrue(new
        // PrepTurretWithValues(turret));

        // manipulatorController.a().onTrue(Commands.runOnce(() -> suzie.resetAngle()));

        driveController.povLeft().whileTrue(Commands.runOnce(() -> suzie.setSpeed(0.05), suzie))
                .onFalse(Commands.runOnce(() -> suzie.setSpeed(0), suzie));
        driveController.povRight().whileTrue(Commands.runOnce(() -> suzie.setSpeed(-0.05), suzie))
                .onFalse(Commands.runOnce(() -> suzie.setSpeed(0), suzie));

        driveController.y().whileTrue(

                Commands.run(() ->
                {
                    Optional<Rotation2d> velAngle = container.getMovementDirectionRobotRelative();
                    if (velAngle.isEmpty())
                    {
                        DogLog.log("VelDir", Double.NaN);
                    } else
                    {
                        Rotation2d targetAngle = velAngle.get().plus(Rotation2d.k180deg);
                        DogLog.log("VelTarget", targetAngle.getDegrees());
                        suzie.setTargetAngle(Degrees.of(targetAngle.getDegrees()));
                        CommandScheduler.getInstance().schedule(Commands.runOnce(() -> suzie.start(), suzie));
                    }

                })).onFalse(Commands.runOnce(() -> suzie.stop(), suzie));
    }
}
