package frc.robot.ralph.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ralph.subsystems.shooter.ManipulatorTalonFX;

public class Intake extends Command
{
    private final ManipulatorTalonFX m_shooter;

    public Intake(ManipulatorTalonFX shooter)
    {
        addRequirements(shooter);
        m_shooter = shooter;
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
    }

    @Override
    public boolean isFinished()
    {
        return m_shooter.hasCoral();
    }
}
