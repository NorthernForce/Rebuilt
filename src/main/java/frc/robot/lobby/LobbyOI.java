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
import frc.robot.lobby.subsystems.spindexer.commands.RunSpindexer;
import frc.robot.lobby.subsystems.spindexer.flicker.FlickerIO;
import frc.robot.lobby.subsystems.turret.commands.PrepTurretCommand;
import frc.robot.lobby.subsystems.turret.commands.PrepTurretWithValues;

public class LobbyOI {
        private static DoubleSupplier inputProc(DoubleSupplier input) {
                return () -> {
                        double x = MathUtil.applyDeadband(input.getAsDouble(), 0.1, 1);
                        return -x * Math.abs(x);
                };
        }

        public void bind(LobbyContainer container) {
                var driveController = new CommandXboxController(0);
                var manipulatorController = new CommandXboxController(1);
                var drive = container.getDrive();
                var intake = container.getIntake();
                if (DriverStation.isTest()) {
                        driveController.rightBumper().whileTrue(Commands.run(() -> container.getSpindexer().getCarousel().setPower(1), container.getSpindexer()));
                        container.getTurret().getShooter().start();
                        driveController.rightTrigger().whileTrue(Commands.run(()->container.getSpindexer().getFlicker().setPower(1), container.getTurret()).alongWith(Commands.run(() -> container.getSpindexer().getCarousel().setPower(1), container.getSpindexer()), Commands.run(()->container.getTurret().getShooter().start(), container.getTurret()))).onFalse(Commands.run(()->container.getSpindexer().getFlicker().setPower(0), container.getTurret()).alongWith(Commands.run(() -> container.getSpindexer().getCarousel().setPower(0), container.getSpindexer()), Commands.run(()->container.getTurret().getShooter().stop(), container.getTurret())));
                       drive.setDefaultCommand(drive.driveByJoystick(inputProc(driveController::getLeftY),
                                        inputProc(driveController::getLeftX), inputProc(driveController::getRightX)));
                        driveController.povUp().whileTrue(Commands.run(()->container.getTurret().getHood().setSpeed(0.2, false))).onFalse(Commands.runOnce(()->container.getTurret().getHood().setSpeed(0, false)));
                        driveController.povDown().whileTrue(Commands.run(()->container.getTurret().getHood().setSpeed(-0.2, false))).onFalse(Commands.runOnce(()->container.getTurret().getHood().setSpeed(0, false)));
                        driveController.leftBumper().whileTrue(container.getIntake().getRunToIntakeAngleCommand()).onFalse(container.getIntake().getRunToStowAngleCommand());
                        
                        
                } else {

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
                                        .resetOdometry(new Pose2d(Meters.of(0), Meters.of(0),
                                                        new Rotation2d(Degrees.of(180))))));

                        manipulatorController.b().onTrue(Commands.runOnce(() -> {
                                DogLog.log("Turret/csvValue", container.getTurret().getHoodTargetingCalculator()
                                                .getValueForDistance(5.0));
                        }));
                        (new Trigger(() -> container.getSpindexer().getJammed())).whileTrue(
                                        Commands.defer(() -> container.getSpindexer().runBackwards(),
                                                        Set.of(container.getSpindexer())));
                        driveController.rightTrigger().whileTrue(new RunSpindexer(container.getSpindexer()));
                        // driveController.leftTrigger().whileTrue(new
                        // PrepTurretWithValues(container.getTurret()));
                        driveController.povUp()
                                        .onTrue(Commands.runOnce(() -> hood.setTargetAngle(Degrees.of(180)),
                                                        container.getTurret()))
                                        .onFalse(Commands.runOnce(() -> hood.setTargetAngle(Degrees.zero()),
                                                        container.getTurret()));
                        // manipulatorController.leftTrigger().whileTrue(intake.getRunToIntakeAngleCommand());
                        // intake.setDefaultCommand(intake.getRunToStowAngleCommand());
                        manipulatorController.leftBumper().whileTrue(intake.intake(0.75)).onFalse(intake.stopIntake());
                        manipulatorController.leftTrigger().whileTrue(new PrepTurretWithValues(container.getTurret()));
                        manipulatorController.rightTrigger().whileTrue(new RunSpindexer(container.getSpindexer()));
                        driveController.leftTrigger().whileTrue(
                                        container.roughDriveToPose(new Pose2d(Meters.of(3), Meters.of(3),
                                                        new Rotation2d(Degrees.of(0)))));
                        driveController.rightTrigger().whileTrue(
                                        container.driveToPose(new Pose2d(Meters.of(3), Meters.of(3),
                                                        new Rotation2d(Degrees.of(0)))));

                        driveController.a().onTrue(Commands.runOnce(() -> container
                                        .resetOdometry(new Pose2d(Meters.of(0), Meters.of(0),
                                                        new Rotation2d(Degrees.of(180))))));
                }
        }
}
