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

    public Command getRunToIntakeAngleCommand()
    {
        return run(() -> io.runToIntakeAngle());
    }

    public Command getRunToStowAngleCommand()
    {
        return run(() -> io.runToStowAngle());
    }

    public Command getRunToMidAngleCommand()
    {
        return run(() -> io.runToMidAngle());
    }

    public Command stopIntake()
    {
        return run(() -> io.stopIntake());
    }

}
