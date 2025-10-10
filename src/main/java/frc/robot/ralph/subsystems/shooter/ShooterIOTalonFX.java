package frc.robot.ralph.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterIOTalonFX
{
    private int id;
    private TalonFX talonFX;

    public ShooterIOTalonFX(int id)
    {
        this.id = id;
        talonFX = new TalonFX(id);
        talonFX.getConfigurator().apply(new TalonFXConfiguration());
    }

    public void intake()
    {
        talonFX.set(0.5);
    }

    public void outtake()
    {
        talonFX.set(-0.5);
    }

    public void stop()
    {
        talonFX.set(0);

    }

    public int getId()
    {
        return id;
    }
}