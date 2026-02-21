package frc.robot.lobby;

import java.util.Set;
import java.util.function.DoubleSupplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants;
import frc.robot.lobby.subsystems.spindexer.commands.RunSpindexer;
import frc.robot.subsystems.turret.commands.PrepTurretWithValues;

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
        driveController.back().onTrue(drive.resetOrientation());

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
        (new Trigger(() -> container.getSpindexer().getJammed())).whileTrue(
                Commands.defer(() -> container.getSpindexer().runBackwards(), Set.of(container.getSpindexer())));
        driveController.rightTrigger().whileTrue(new RunSpindexer(container.getSpindexer()));
        // driveController.leftTrigger().whileTrue(new
        // PrepTurretWithValues(container.getTurret()));
        driveController.povUp()
                .onTrue(Commands.runOnce(() -> hood.setTargetAngle(Degrees.of(180)), container.getTurret()))
                .onFalse(Commands.runOnce(() -> hood.setTargetAngle(Degrees.zero()), container.getTurret()));
        // manipulatorController.leftTrigger().whileTrue(intake.getRunToIntakeAngleCommand());
        // intake.setDefaultCommand(intake.getRunToStowAngleCommand());
        manipulatorController.leftBumper().whileTrue(intake.intake(0.75)).onFalse(intake.stopIntake());
        manipulatorController.leftTrigger().whileTrue(new PrepTurretWithValues(container.getTurret()));
        manipulatorController.rightTrigger().whileTrue(new RunSpindexer(container.getSpindexer()));
        driveController.a().onTrue(Commands.runOnce(() -> container
                .resetOdometry(new Pose2d(Meters.of(0), Meters.of(0), new Rotation2d(Degrees.of(180))))));

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

            // Compute an entry point before the trench and an exit point beyond it so the
            // robot drives completely through the trench instead of stopping at the center.
            double entryDist = LobbyConstants.AutoTrenchConstants.kApproachDistance;
            double throughDist = LobbyConstants.AutoTrenchConstants.kThroughDistance;

            double entryX = nearest.getX() - Math.cos(angle) * entryDist;
            double entryY = nearest.getY() - Math.sin(angle) * entryDist;
            Pose2d entryPose = new Pose2d(entryX, entryY, new Rotation2d(angle));

            double throughX = nearest.getX() + Math.cos(angle) * throughDist;
            double throughY = nearest.getY() + Math.sin(angle) * throughDist;
            Pose2d throughPose = new Pose2d(throughX, throughY, new Rotation2d(angle));

            return Commands.sequence(
                    Commands.runOnce(() -> DogLog.log("Auto/Trench", "Auto-trench activate, nearest=" + nearestDesc),
                            drive),
                    // Move to entry (pre-align and position)
                    container.driveToPose(entryPose),
                    // Drive through to exit point past the trench
                    container.driveToPose(throughPose));
        }, Set.of(drive)));
    }
}
