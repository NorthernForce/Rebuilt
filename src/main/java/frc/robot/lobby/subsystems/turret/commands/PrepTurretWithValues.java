package frc.robot.lobby.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lobby.subsystems.turret.Turret;
import frc.robot.lobby.subsystems.turret.Turret.TurretPose;

public class PrepTurretWithValues extends ParallelCommandGroup
{
    public PrepTurretWithValues(Turret turret)
    {
        addCommands(
                // Commands.run(() -> turret.getSuzie().start(), turret.getSuzie()),
                Commands.run(() -> turret.getHood().start(), turret.getHood()),
                Commands.run(() -> turret.getShooter().start(), turret.getShooter()));
    }
}
