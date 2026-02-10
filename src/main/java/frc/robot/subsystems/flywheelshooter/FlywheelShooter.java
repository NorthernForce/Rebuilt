package frc.robot.subsystems.flywheelshooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelShooter extends SubsystemBase
{
    private final TalonFX hoodMotor;
    private final TalonFX flywheelMotor;

    public FlywheelShooter(int hoodMotorID, int flywheelMotorID)
    {
        hoodMotor = new TalonFX(hoodMotorID);
        flywheelMotor = new TalonFX(flywheelMotorID);
    }

    public void setHoodAngle(Angle angle)
    {
        hoodMotor.setPosition(angle);
    }

    public Angle getHoodAngle()
    {
        return hoodMotor.getPosition().getValue();
    }

    public void setFlywheelSpeed(double speed)
    {
        flywheelMotor.set(speed);
    }

    public Command setFlywheelSpeedCommand(double speed)
    {
        return run(() -> setFlywheelSpeed(speed));
    }

    public Command setHoodAngleCommand(Angle angle)
    {
        return run(() -> setHoodAngle(angle));
    }

    @Override
    public void periodic()
    {
    }
}
