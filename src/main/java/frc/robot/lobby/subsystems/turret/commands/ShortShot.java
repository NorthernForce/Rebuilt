package frc.robot.lobby.subsystems.turret.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lobby.subsystems.turret.Turret;

public class ShortShot extends Command
{
    private final Turret turret;
    private final AngularVelocity velocity;

    public ShortShot(Turret turret, AngularVelocity velocity)
    {
        this.turret = turret;
        this.velocity = velocity;
    }

    public ShortShot(Turret turret)
    {
        this(turret, RotationsPerSecond.of(25));
    }

    @Override
    public void initialize()
    {
        turret.getHood().setTargetMechanismAngle(Degrees.of(0));
        turret.getShooter().setTargetSpeed(velocity);

        turret.start();
    }
}
