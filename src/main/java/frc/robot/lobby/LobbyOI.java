package frc.robot.lobby;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.lobby.subsystems.CommandSwerveDrivetrain;
import frc.robot.lobby.subsystems.intake.Intake;
import frc.robot.lobby.subsystems.spindexer.commands.Agitate;
import frc.robot.lobby.subsystems.spindexer.commands.RunSpindexer;
import frc.robot.lobby.subsystems.turret.commands.PrepTurretCommand;
import frc.robot.lobby.subsystems.turret.commands.PrepTurretWithValues;
import frc.robot.lobby.subsystems.turret.suzie.Suzie;

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
        manipulatorController.povLeft().whileTrue(Commands.runOnce(() -> suzie.setSpeed(0.05), suzie))
                .onFalse(Commands.runOnce(() -> suzie.setSpeed(0), suzie));
        manipulatorController.povRight().whileTrue(Commands.runOnce(() -> suzie.setSpeed(-0.05), suzie))
                .onFalse(Commands.runOnce(() -> suzie.setSpeed(0), suzie));
        driveController.leftTrigger().whileTrue(intake.intakeMoving()).onFalse(intake.stopIntake());
        driveController.rightTrigger().whileTrue(Commands
                .waitUntil(() -> turret.getSuzie().isAtTargetAngle() && turret.getShooter().isAtTargetSpeed())
                .andThen(new RunSpindexer(container.getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime))
                .alongWith(new PrepTurretCommand(() -> container.predictPose(), turret)));

        manipulatorController.leftStick().whileTrue(intake.driveByJoystick(() -> manipulatorController.getLeftY()));

        manipulatorController.leftBumper().whileTrue(new PrepTurretWithValues(turret));
        manipulatorController.rightBumper()
                .whileTrue(new RunSpindexer(spindexer, LobbyConstants.SpindexerConstants.kDeJamTime));

        manipulatorController.leftTrigger().whileTrue(intake.intakeMoving()).onFalse(intake.stopIntake());
        manipulatorController.rightTrigger().whileTrue(Commands.waitSeconds(0.25)
                .andThen(new RunSpindexer(container.getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime))
                .alongWith(new PrepTurretCommand(() -> container.predictPose(), turret)));

        drive.setDefaultCommand(drive.driveByJoystick(inputProc(driveController::getLeftY),
                inputProc(driveController::getLeftX), inputProc(driveController::getRightX)));
        intake.setDefaultCommand(intake.stopIntake().andThen(intake.getRunToMidAngleCommand()));
        // spindexer.setDefaultCommand(new Agitate(spindexer));
        // turret.setDefaultCommand(container.getTurret().runBasedOnLocation(() ->
        // drive.getState().Pose,
        // hood.getDangerZone(), hood.getTrenchPositions()));
        shooter.setDefaultCommand(Commands.run(() -> shooter.stop(), shooter));

        driveController.leftTrigger().whileTrue(intake.intakeMoving());
        driveController.rightTrigger()
                .whileTrue(new RunSpindexer(container.getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime)
                        .alongWith(new PrepTurretWithValues(container.getTurret())));

    }

    public void bindTest(LobbyContainer container)
    {
        CommandXboxController driveController = new CommandXboxController(0);
        CommandXboxController manipulatorController = new CommandXboxController(1);
        CommandSwerveDrivetrain drive = container.getDrive();
        Intake intake = container.getIntake();
        Suzie suzie = container.getTurret().getSuzie();
        driveController.back().onTrue(drive.resetOrientation());

        driveController.rightTrigger()
                .whileTrue(new RunSpindexer(container.getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime)
                        .alongWith(new PrepTurretWithValues(container.getTurret())));
        drive.setDefaultCommand(drive.driveByJoystick(inputProc(driveController::getLeftY),
                inputProc(driveController::getLeftX), inputProc(driveController::getRightX)));

        driveController.leftBumper()
                .whileTrue(Commands.run(() -> container.getTurret().getHood().setSpeed(0.2, false)));

        driveController.rightBumper()
                .whileTrue(Commands.run(() -> container.getTurret().getHood().setSpeed(-0.2, false)));

        driveController.leftTrigger().whileTrue(intake.intakeMoving());
        intake.setDefaultCommand(intake.stopIntake());
        driveController.povLeft().whileTrue(Commands.runOnce(() -> suzie.setSpeed(0.05), suzie))
                .onFalse(Commands.runOnce(() -> suzie.setSpeed(0), suzie));
        driveController.povRight().whileTrue(Commands.runOnce(() -> suzie.setSpeed(-0.05), suzie))
                .onFalse(Commands.runOnce(() -> suzie.setSpeed(0), suzie));
        driveController.a().onTrue(Commands.runOnce(() -> container
                .resetOdometry(new Pose2d(Meters.of(0), Meters.of(0), new Rotation2d(Degrees.of(180))))));
    }
}
