package frc.robot.ralph;

import frc.robot.ralph.subsystems.shooter.ShooterIOTalonFX;

public class RalphContainer
{
    private final ShooterIOTalonFX motor;

    public RalphContainer()
    {
        motor = new ShooterIOTalonFX(1);
    }

    public ShooterIOTalonFX getMotor()
    {
        return motor;
    }
}