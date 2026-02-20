package frc.robot.subsystems.turret.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.Turret.TurretPose;

public class PrepTurretWithValues extends Command
{
    private Supplier<Pose2d> robotPoseSupplier;
    private Turret turret;

    public PrepTurretWithValues(Turret turret)
    {
        addRequirements(turret.getShooter());
        this.turret = turret;
    }

    @Override
    public void initialize()
    {
        DogLog.log("Turret/PrepCommand/Running", true);
        turret.getShooter().getIO().start();
    }

    @Override
    public void end(boolean interrupted)
    {
        turret.getShooter().getIO().stop();
        DogLog.log("Turret/PrepCommand/Running", false);
    }
}
