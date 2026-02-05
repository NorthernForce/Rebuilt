package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase
{
    private final TalonFXS rollerMotor;

    public Intake(int motorID)
    {
        rollerMotor = new TalonFXS(motorID);
    }

    public Command intake(double speed)
    {
        return runOnce(() ->
        {
            rollerMotor.set(speed);
        });
    }

    public Command purgeIntake(double speed)
    {
        return runOnce(() ->
        {
            rollerMotor.set(-speed);
        });
    }

    public Command stopIntake()
    {
        return runOnce(() ->
        {
            rollerMotor.set(0);
        });
    }
}
