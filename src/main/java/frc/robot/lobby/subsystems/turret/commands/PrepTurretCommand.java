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

public class PrepTurretCommand extends Command
{
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Turret turret;
    private final Supplier<Translation2d> turretPositionSupplier;
    private final DoubleSupplier joystickVelocitySupplier;

    public PrepTurretCommand(LobbyContainer container, DoubleSupplier joystickVelocitySupplier)
    {
        addRequirements(container.getTurret(), container.getTurret().getSuzie(), container.getTurret().getHood(),
                container.getTurret().getShooter());
        this.turret = container.getTurret();
        robotPoseSupplier = () -> container.getDrive().getPose();
        turretPositionSupplier = () -> container.predictTurretPose();
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

        // Calculate the target pose
        TurretPose targetPose = turret.calculateTargetPose(turretPosition, robotPoseSupplier.get());
        DogLog.log("Turret/PrepCommand/CalculatedSuzieAngle", targetPose.suzieAngle().in(Radians));
        DogLog.log("Turret/PrepCommand/CalculatedHoodAngle", targetPose.hoodAngle().in(Degrees));
        DogLog.log("Turret/PrepCommand/DistanceToHub", turret.getDistanceToHub(currentPose));
        turret.setTargetPose(
                new TurretPose(targetPose.suzieAngle(), targetPose.hoodAngle(), targetPose.shooterSpeed()));

        turret.start();
    }

    @Override
    public void end(boolean interrupted)
    {
        turret.setTargetPose(TurretPose.kZero);
        DogLog.log("Turret/PrepCommand/Running", false);
        turret.stop();
    }
}
