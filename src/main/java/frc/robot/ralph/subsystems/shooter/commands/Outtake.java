package frc.robot.ralph.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ralph.subsystems.shooter.ManipulatorTalonFX;
import frc.robot.ralph.subsystems.shooter.ManipulatorTalonFX.ManipulatorState;

public class Outtake extends Command
{
    private final ManipulatorTalonFX m_manipulator;

    public Outtake(ManipulatorTalonFX manipulator)
    {
        addRequirements(manipulator);
        m_manipulator = manipulator;
    }

    @Override
    public void initialize()
    {
        this.m_manipulator.setState(ManipulatorState.EXPULSANDO);
    }

    @Override
    public boolean isFinished()
    {
        return !this.m_manipulator.hasCoralInSensor();
    }

    @Override
    public void execute()
    {

    }
}
