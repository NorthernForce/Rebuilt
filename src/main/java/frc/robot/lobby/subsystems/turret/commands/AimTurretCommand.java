package frc.robot.lobby.subsystems.turret.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lobby.LobbyContainer;
import frc.robot.lobby.subsystems.turret.Turret;
import frc.robot.lobby.subsystems.turret.Turret.TurretPose;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AimTurretCommand extends Command
{
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<Translation2d> turretPositionSupplier;
    private final Turret turret;
    private final DoubleSupplier joystickVelocitySupplier;

    public AimTurretCommand(LobbyContainer container, DoubleSupplier joystickVelocitySupplier,
            Supplier<Translation2d> turretTranslationSupplier)
    {
        addRequirements(container.getTurret(), container.getTurret().getSuzie(), container.getTurret().getHood(),
                container.getTurret().getShooter());
        this.turret = container.getTurret();
        robotPoseSupplier = () -> container.getDrive().getPose();
        turretPositionSupplier = turretTranslationSupplier;
        this.joystickVelocitySupplier = joystickVelocitySupplier;
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

        Translation2d turretPosition;
        if (joystickVelocitySupplier.getAsDouble() < 0.1)
        {
            turretPosition = turret.calculateFieldRelativeShooterPosition(currentPose);
        } else
        {
            turretPosition = turretPositionSupplier.get();
        }
        DogLog.log("Turret/PrepCommand/TurretPosition", turretPosition);

        var hood = turret.getHood();
        boolean inDanger = turret.isInDangerProximity(turretPosition, hood.getDangerZone(), hood.getTrenchPositions());
        DogLog.log("Turret/PrepCommand/InDanger", inDanger);

        // Calculate the target pose
        TurretPose targetPose = turret.calculateTargetPose(turretPositionSupplier.get(), currentPose);
        DogLog.log("Turret/PrepCommand/CalculatedSuzieAngle", targetPose.suzieAngle().in(Radians));
        DogLog.log("Turret/PrepCommand/CalculatedHoodAngle", targetPose.hoodAngle().in(Degrees));
        DogLog.log("Turret/PrepCommand/DistanceToHub", turret.getDistanceToHub(currentPose));
        turret.setTargetPose(
                new TurretPose(targetPose.suzieAngle(), targetPose.hoodAngle(), targetPose.shooterSpeed()));

        turret.getSuzie().start();
        turret.getHood().start();
        turret.getShooter().stop();
    }

    @Override
    public void end(boolean interrupted)
    {
        DogLog.log("Turret/PrepCommand/Running", false);
        turret.stop();
    }
}
