package frc.robot.lobby;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Set;
import java.util.function.DoubleSupplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

        // Auto-trench behaviour: when the robot is within a configured radius of a
        // trench
        // disable manual driving (by scheduling commands that require the drive
        // subsystem),
        // pre-align heading to face the trench, then drive through the trench with an
        // offset.
        var trenchTrigger = new Trigger(() ->
        {
            var pose = drive.getState().Pose;
            double x = pose.getX();
            double y = pose.getY();
            double radius = LobbyConstants.AutoTrenchConstants.kTriggerRadius;
            Translation2d[] trenches = new Translation2d[]
            { FieldConstants.kBlueTrench1, FieldConstants.kBlueTrench2, FieldConstants.kRedTrench1,
                    FieldConstants.kRedTrench2 };
            for (Translation2d t : trenches)
            {
                double dx = t.getX() - x;
                double dy = t.getY() - y;
                if (Math.hypot(dx, dy) <= radius)
                {
                    return true;
                }
            }
            return false;
        });

        // Compute sequence at trigger time so we use the current robot pose when
        // activating. Use whileTrue so the auto sequence runs while inside the radius
        // and holds the drivetrain subsystem (disables manual driving during the
        // sequence).
        trenchTrigger.whileTrue(Commands.defer(() ->
        {
            Pose2d current = drive.getState().Pose;

            // find nearest trench
            Translation2d[] trenches = new Translation2d[]
            { FieldConstants.kBlueTrench1, FieldConstants.kBlueTrench2, FieldConstants.kRedTrench1,
                    FieldConstants.kRedTrench2 };
            Translation2d nearest = null;
            double best = Double.MAX_VALUE;
            for (Translation2d t : trenches)
            {
                double d = Math.hypot(t.getX() - current.getX(), t.getY() - current.getY());
                if (d < best)
                {
                    best = d;
                    nearest = t;
                }
            }
            if (nearest == null)
            {
                return Commands.none();
            }

            final String nearestDesc = nearest.getX() + "," + nearest.getY();

            // heading toward trench
            double dx = nearest.getX() - current.getX();
            double dy = nearest.getY() - current.getY();
            double angle = Math.atan2(dy, dx);

            // Round heading to the nearest 90 degrees (PI/2) so the robot will
            // align to the closest cardinal orientation before driving through.
            double ninety = Math.PI / 2.0;
            double rawNearest = Math.round(angle / ninety) * ninety;
            final double angleNearest90 = (rawNearest > Math.PI) ? rawNearest - 2.0 * Math.PI : rawNearest;

            // Compute an entry point before the trench and an exit point beyond it so the
            // robot drives completely through the trench instead of stopping at the center.
            double entryDist = LobbyConstants.AutoTrenchConstants.kApproachDistance;
            double throughDist = LobbyConstants.AutoTrenchConstants.kThroughDistance;

            double entryX = nearest.getX() - Math.cos(angleNearest90) * entryDist;
            double entryY = nearest.getY() - Math.sin(angleNearest90) * entryDist;
            Pose2d entryPose = new Pose2d(entryX, entryY, new Rotation2d(angleNearest90));

            double throughX = nearest.getX() + Math.cos(angleNearest90) * throughDist;
            double throughY = nearest.getY() + Math.sin(angleNearest90) * throughDist;
            Pose2d throughPose = new Pose2d(throughX, throughY, new Rotation2d(angleNearest90));

            // Pre-align rotation-only command: rotate to the nearest 90-degree heading
            PIDController preRotatePid = new PIDController(7.0, 0.0, 0.0);
            preRotatePid.enableContinuousInput(-Math.PI, Math.PI);
            preRotatePid.setSetpoint(angleNearest90);

            double tol = Math.toRadians(4.0);

            var rotateCommand = Commands.run(() ->
            {
                double curr = drive.getState().Pose.getRotation().getRadians();
                double output = preRotatePid.calculate(curr, angleNearest90);
                double maxOmega = LobbyConstants.DrivetrainConstants.kMaxAngularSpeed.in(RadiansPerSecond);
                double clipped = MathUtil.clamp(output, -maxOmega, maxOmega);
                drive.fieldRelativeDrive(new ChassisSpeeds(0.0, 0.0, clipped));
            }, drive).until(() ->
            {
                double curr = drive.getState().Pose.getRotation().getRadians();
                double err = Math.atan2(Math.sin(angleNearest90 - curr), Math.cos(angleNearest90 - curr));
                return Math.abs(err) < tol;
            });

            return Commands.sequence(
                    Commands.runOnce(() -> DogLog.log("Auto/Trench", "Auto-trench activate, nearest=" + nearestDesc),
                            drive),
                    // Rotate in place to pre-align
                    rotateCommand,
                    // Move to entry (positioned before trench)
                    container.driveToPose(entryPose),
                    // Drive through to exit point past the trench
                    container.driveToPose(throughPose));
        }, Set.of(drive)));
    }
}