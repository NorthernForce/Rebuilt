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

        var hood = container.getTurret().getHood();
        driveController.a().onTrue(Commands.runOnce(() -> container
                .resetOdometry(new Pose2d(Meters.of(0), Meters.of(0), new Rotation2d(Degrees.of(180))))));
        driveController.leftTrigger().whileTrue(intake.intakeMoving());
        driveController.rightTrigger()
                .whileTrue(new RunSpindexer(container.getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime)
                        .alongWith(new PrepTurretWithValues(container.getTurret())));
        container.getTurret().getShooter().setDefaultCommand(container.getTurret().getShooter().stop());
        driveController.a().onTrue(Commands.runOnce(() -> container
                .resetOdometry(new Pose2d(Meters.of(0), Meters.of(0), new Rotation2d(Degrees.of(180))))));

        driveController.povLeft().whileTrue(container.getTurret().getSuzie().setSpeed(0.05))
                .onFalse(container.getTurret().getSuzie().setSpeed(0));
        driveController.povRight().whileTrue(container.getTurret().getSuzie().setSpeed(-0.05))
                .onFalse(container.getTurret().getSuzie().setSpeed(0));

    }

    public void bindTest(LobbyContainer container)
    {
        CommandXboxController driveController = new CommandXboxController(0);
        CommandXboxController manipulatorController = new CommandXboxController(1);
        CommandSwerveDrivetrain drive = container.getDrive();
        Intake intake = container.getIntake();
        driveController.rightTrigger()
                .whileTrue(new RunSpindexer(container.getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime)
                        .alongWith(new PrepTurretWithValues(container.getTurret())));
        drive.setDefaultCommand(drive.driveByJoystick(inputProc(driveController::getLeftY),
                inputProc(driveController::getLeftX), inputProc(driveController::getRightX)));

        driveController.leftBumper()
                .whileTrue(Commands.run(() -> container.getTurret().getHood().setSpeed(0.2, false)));

        container.getTurret().getHood().setDefaultCommand(container.getTurret().getHood().stop());
        driveController.rightBumper()
                .whileTrue(Commands.run(() -> container.getTurret().getHood().setSpeed(-0.2, false)));

        driveController.leftTrigger().whileTrue(intake.intakeMoving());
        intake.setDefaultCommand(intake.stopIntake());
        container.getTurret().getShooter().setDefaultCommand(container.getTurret().getShooter().stop());

        driveController.povLeft().whileTrue(container.getTurret().getSuzie().setSpeed(0.05))
                .onFalse(container.getTurret().getSuzie().setSpeed(0));

        driveController.povRight().whileTrue(container.getTurret().getSuzie().setSpeed(-0.05))
                .onFalse(container.getTurret().getSuzie().setSpeed(0));
    }
}
