package frc.robot.ralph.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterIOTalonFX extends SubsystemBase
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

    public Command getIntakeCommand()
    {
        return Commands.run(() -> intake(), this);
    }

    public Command getOuttakeCommand()
    {
        return Commands.run(() -> outtake(), this);
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