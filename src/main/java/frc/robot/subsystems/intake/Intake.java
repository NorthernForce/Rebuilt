package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase
{
    public IntakeIO io;

    public Intake(IntakeIO io)
    {
        this.io = io;
    }

    public Command purgeIntake(double speed)
    {
        return run(() -> io.intake(speed));
    }

    public Command intake(double speed)
    {
        return run(() -> io.purgeIntake(speed));
    }

    public Command stopIntake()
    {
        return run(() -> io.stopIntake());
    }

}
