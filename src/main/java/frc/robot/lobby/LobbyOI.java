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
import frc.robot.subsystems.climber.commands.RunToPosition;
import frc.robot.lobby.LobbyConstants.ClimberConstants.ClimbLevels;
import frc.robot.lobby.subsystems.spindexer.commands.RunSpindexer;
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

        drive.setDefaultCommand(drive.driveByJoystick(inputProc(driveController::getLeftY),
                inputProc(driveController::getLeftX), inputProc(driveController::getRightX)));
        intake.setDefaultCommand(intake.stopIntake().andThen(intake.getRunToMidAngleCommand()));

        manipulatorController.leftStick().whileTrue(intake.driveByJoystick(() -> manipulatorController.getLeftY()));

        driveController.back().onTrue(drive.resetOrientation());
        container.getClimber().setDefaultCommand(container.getClimber().getHomingCommand());
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

        // driveController.leftTrigger().whileTrue(new
        // PrepTurretWithValues(container.getTurret()));
        driveController.povUp().whileTrue(new RunToPosition(container, ClimbLevels.L3));
        // Temporary test: hold D-pad down to command elevator up, release to home
        driveController.povDown().onTrue(container.getClimber().runUp());
        // manipulatorController.leftTrigger().whileTrue(intake.getRunToIntakeAngleCommand());
        // intake.setDefaultCommand(intake.getRunToStowAngleCommand());
        driveController.leftTrigger().whileTrue(intake.intakeMoving());
        driveController.rightTrigger()
                .whileTrue(new RunSpindexer(container.getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime)
                        .alongWith(new PrepTurretWithValues(container.getTurret())));
        container.getTurret().getShooter().setDefaultCommand(container.getTurret().getShooter().stop());
        // manipulatorController.rightTrigger().whileTrue(new
        // RunSpindexer(container.getSpindexer()));
        // manipulatorController.leftBumper().whileTrue(new
        // PrepTurretWithValues(container.getTurret()));
        driveController.a().onTrue(Commands.runOnce(() -> container
                .resetOdometry(new Pose2d(Meters.of(0), Meters.of(0), new Rotation2d(Degrees.of(180))))));
        manipulatorController.a().onTrue(Commands.runOnce(() -> container.getTurret().getSuzie().resetAngle()));

        driveController.povLeft().whileTrue(container.getTurret().getSuzie().setSpeed(0.05))
                .onFalse(container.getTurret().getSuzie().setSpeed(0));
        driveController.povRight().whileTrue(container.getTurret().getSuzie().setSpeed(-0.05))
                .onFalse(container.getTurret().getSuzie().setSpeed(0));
    }
}
