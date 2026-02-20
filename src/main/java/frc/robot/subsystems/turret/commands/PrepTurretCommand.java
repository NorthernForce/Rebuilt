package frc.robot.subsystems.turret.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.Turret.TurretPose;

public class PrepTurretCommand extends Command
{
    private Supplier<Pose2d> robotPoseSupplier;
    private Turret turret;

    public PrepTurretCommand(Supplier<Pose2d> robotPoseSupplier, Turret turret)
    {
        addRequirements(turret);
        this.robotPoseSupplier = robotPoseSupplier;
        this.turret = turret;
    }

    @Override
    public void initialize()
    {
        DogLog.log("Turret/PrepCommand/Running", true);
    }

    @Override
    public void execute()
    {
        Pose2d currentPose = robotPoseSupplier.get();
        DogLog.log("Turret/PrepCommand/RobotPose", currentPose);

        // Calculate turret position and use it for danger check
        Translation2d turretPosition = turret.calculateFieldRelativeShooterPosition(currentPose);
        DogLog.log("Turret/PrepCommand/TurretPosition", turretPosition);

        var hood = turret.getHood();
        boolean inDanger = turret.isInDangerProximity(turretPosition, hood.getIO().getDangerZone(),
                hood.getIO().getTrenchPositions());
        DogLog.log("Turret/PrepCommand/InDanger", inDanger);

        // Calculate the target pose
        TurretPose targetPose = turret.calculateTargetPose(currentPose);
        DogLog.log("Turret/PrepCommand/CalculatedSuzieAngle", targetPose.suzieAngle().in(Radians));
        DogLog.log("Turret/PrepCommand/CalculatedHoodAngle", targetPose.hoodAngle().in(Degrees));
        DogLog.log("Turret/PrepCommand/DistanceToHub", turret.getDistanceToHub(currentPose));

        if (inDanger)
        {
            // Keep suzie and shooter targeting, but force hood down
            turret.setTargetPose(new TurretPose(targetPose.suzieAngle(), Degrees.of(0), targetPose.shooterSpeed()));
        } else
        {
            turret.setTargetPose(targetPose);
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        turret.setTargetPose(TurretPose.kZero);
        DogLog.log("Turret/PrepCommand/Running", false);
    }
}
