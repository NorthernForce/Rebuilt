package frc.robot.subsystems.turret.commands;

import java.util.function.Supplier;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.turret.Turret;

public class PrepTurretWithValues extends ParallelCommandGroup
{
    public PrepTurretWithValues(Turret turret)
    {
        addCommands(turret.getShooter().start());
    }
}
