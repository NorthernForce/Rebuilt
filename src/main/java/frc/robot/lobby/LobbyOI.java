package frc.robot.lobby;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.function.DoubleSupplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
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
        DoubleSupplier driveXInput = inputProc(driveController::getLeftY);
        DoubleSupplier driveYInput = inputProc(driveController::getLeftX);
        DoubleSupplier driveOmegaInput = inputProc(driveController::getRightX);

        drive.setDefaultCommand(drive.driveByJoystick(driveXInput, driveYInput, driveOmegaInput));
        intake.setDefaultCommand(intake.stopIntake().andThen(intake.getRunToMidAngleCommand()));
        // spindexer.setDefaultCommand(new Agitate(spindexer));
        // turret.setDefaultCommand(container.getTurret().runBasedOnLocation(() ->
        // drive.getState().Pose,
        // hood.getDangerZone(), hood.getTrenchPositions()));
        // shooter.setDefaultCommand(Commands.run(() -> shooter.stop(), shooter));

        driveController.back().onTrue(drive.resetOrientation());
        driveController.x().toggleOnTrue(intake.stopIntake().andThen(intake.getRunToStowAngleCommand()).repeatedly());

        driveController.leftTrigger().whileTrue(intake.intakeMoving()).onFalse(intake.stopIntake());
        driveController.rightTrigger().whileTrue(Commands
                .waitUntil(() -> turret.getSuzie().isAtTargetAngle() && turret.getShooter().isAtTargetSpeed())
                .andThen(new RunSpindexer(container.getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime))
                .alongWith(new PrepTurretCommand(() -> container.predictPose(), turret)));

        driveController.start().onTrue(Commands.runOnce(() -> suzie.resetCRT()));

        manipulatorController.leftStick().whileTrue(intake.driveByJoystick(() -> manipulatorController.getLeftY()));

        manipulatorController.leftBumper().whileTrue(new PrepTurretWithValues(turret));

        driveController.rightBumper().whileTrue(Commands.waitSeconds(0.25)
                .andThen(new RunSpindexer(container.getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime))
                .alongWith(new PrepTurretStupid(() -> container.predictPose(), turret)));

        manipulatorController.leftTrigger().whileTrue(intake.intakeMoving()).onFalse(intake.stopIntake());
        manipulatorController.rightTrigger()
                .whileTrue(
                        Commands.waitSeconds(0.25)
                                .andThen(new RunSpindexer(container.getSpindexer(),
                                        LobbyConstants.SpindexerConstants.kDeJamTime))
                                .alongWith(new PrepTurretStupid(() -> new Pose2d(
                                        FieldConstants.kRedHubPosition.toTranslation2d()
                                                .plus(new Translation2d(Inches.of(108), Inches.zero())),
                                        new Rotation2d(Degrees.zero())), turret)));

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

        driveController.y().whileTrue(drive.driveByJoystickFacingAngle(driveXInput, driveYInput, () ->
        {
            double x = driveXInput.getAsDouble();
            double y = driveYInput.getAsDouble();

            Rotation2d targetAngle;
            if (Math.hypot(x, y) < 0.01)
            {
                targetAngle = drive.getPose().getRotation();
            } else
            {
                DogLog.log("VelNewDir", Math.toDegrees(Math.atan2(y, x)));
                targetAngle = Rotation2d.fromRadians(Math.atan2(y, x)).plus(Rotation2d.k180deg);
            }

            DogLog.log("VelTarget", targetAngle.getDegrees());
            return targetAngle;
        }));

        driveController.b().whileTrue(drive.driveByJoystickFacingAngle(driveXInput, driveYInput, () ->
        {
            double x = driveXInput.getAsDouble();
            double y = driveYInput.getAsDouble();

            Rotation2d targetAngle;
            if (Math.hypot(x, y) < 0.01)
            {
                targetAngle = drive.getPose().getRotation();
            } else
            {
                DogLog.log("VelNewDir", Math.toDegrees(Math.atan2(y, x)));
                targetAngle = Rotation2d.fromRadians(Math.atan2(y, x));
            }

            DogLog.log("VelTarget", targetAngle.getDegrees());
            return targetAngle;
        }));
    }
}
