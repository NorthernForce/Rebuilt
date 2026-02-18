package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

public interface IntakeIO
{
    public void intake(double speed);

    public void purgeIntake(double speed);

    public void stopIntake();

    public void setAngle(Angle angle);

    public void resetAngle();
}
