package frc.robot.lobby.subsystems.turret.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lobby.subsystems.turret.Turret;
import frc.robot.lobby.subsystems.turret.Turret.TurretPose;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PrepTurretWithValues extends Command
{
    private Supplier<Pose2d> robotPoseSupplier;
    private Turret turret;

    public PrepTurretWithValues(Turret turret)
    {
        addRequirements(turret);
        this.turret = turret;
    }

    @Override
    public void initialize()
    {
        DogLog.log("Turret/PrepCommand/Running", true);
        turret.getShooter().start();
    }

    @Override
    public void end(boolean interrupted)
    {
        turret.getShooter().stop();
        DogLog.log("Turret/PrepCommand/Running", false);
    }
}
