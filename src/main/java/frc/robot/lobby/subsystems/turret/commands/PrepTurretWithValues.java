package frc.robot.lobby.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.lobby.subsystems.turret.Turret;

public class PrepTurretWithValues extends ParallelCommandGroup
{
    public PrepTurretWithValues(Turret turret)
    {
        addCommands(turret.getShooter().start());
    }
}
