package frc.robot.lobby.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lobby.subsystems.turret.Turret;

public class PrepTurretWithValues extends ParallelCommandGroup
{
    public PrepTurretWithValues(Turret turret)
    {
        addRequirements(turret, turret.getSuzie(), turret.getHood(), turret.getShooter());
        addCommands(
                // Commands.run(() -> turret.getSuzie().start(), turret.getSuzie()),
                Commands.run(() -> turret.getHood().start()), Commands.run(() -> turret.getShooter().start()));
    }

    public PrepTurretWithValues(Turret turret, AngularVelocity shooterVelocity, Angle hoodAngle)
    {
        addRequirements(turret, turret.getSuzie(), turret.getHood(), turret.getShooter());
        addCommands(Commands.runOnce(() -> turret.getShooter().setTargetSpeed(shooterVelocity)),
                Commands.runOnce(() -> turret.getHood().setTargetMechanismAngle(hoodAngle)),
                Commands.run(() -> turret.getHood().start()), Commands.run(() -> turret.getShooter().start()));
    }
}
