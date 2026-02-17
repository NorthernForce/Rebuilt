package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IntakeIO extends Subsystem
{
    public Command intake(double speed);

    public Command purgeIntake(double speed);

    public Command stopIntake();

    public Command setAngle(Angle angle);

    public Command resetAngle();
}
