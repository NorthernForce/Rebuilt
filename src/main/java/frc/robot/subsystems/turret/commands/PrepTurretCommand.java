package frc.robot.subsystems.turret.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;

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
    public void execute()
    {
        turret.setTargetPose(turret.calculateTargetPose(robotPoseSupplier.get()));
    }

    @Override
    public void end(boolean interrupted)
    {
        turret.setTargetPose(TurretPose.kZero);
    }
}
