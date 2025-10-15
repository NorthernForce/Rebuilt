package frc.robot.ralph.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Basic ejector/shooter mechanism - basic, no beam break sensor technology
 * implemented
 */

public class ShooterIOTalonFX extends SubsystemBase
{
    private int id;
    private TalonFX talonFX;
    private double speed;

    /**
     * Intake/Shoot Ralph mechanism
     * 
     * @param id    ID of TalonFX
     * @param speed speed to set motor to
     */

    public ShooterIOTalonFX(int id, double speed)
    {
        this.speed = speed;
        this.id = id;
        talonFX = new TalonFX(id);
        talonFX.getConfigurator().apply(new TalonFXConfiguration());
    }

    /**
     * Set motor to run at a certain speed to intake
     */

    public void intake()
    {
        talonFX.set(speed);
    }

    /**
     * Set motor to run at a certain speed to outtake
     */

    public void outtake()
    {
        talonFX.set(-speed);
    }

    /**
     * Get command to intake
     */

    public Command getIntakeCommand()
    {
        return Commands.run(() -> intake(), this);
    }

    /**
     * Get command to outtake
     */

    public Command getOuttakeCommand()
    {
        return Commands.run(() -> outtake(), this);
    }

    /**
     * Stop the motor
     */

    public void stop()
    {
        talonFX.set(0);
    }

    /**
     * Get the ID of the TalonFX
     * 
     * @return ID of TalonFX
     */

    public int getId()
    {
        return id;
    }
}