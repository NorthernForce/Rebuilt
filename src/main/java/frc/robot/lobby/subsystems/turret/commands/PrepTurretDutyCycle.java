package frc.robot.lobby.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lobby.subsystems.turret.Turret;
import frc.robot.lobby.subsystems.turret.Turret.TurretPose;

public class PrepTurretDutyCycle extends ParallelCommandGroup
{
    public PrepTurretDutyCycle(Turret turret)
    {
        addRequirements(turret);
        addCommands(Commands.run(() -> turret.getHood().start()), Commands.run(() -> turret.getShooter().start()));
    }
}
