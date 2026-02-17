package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lobby.LobbyConstants;

public class ClimberSubsystem extends SubsystemBase
{

    private final ClimberIO io;

    private final ClimberIO.ClimberIOInputs inputs = new ClimberIO.ClimberIOInputs();

    private final Servo hookServo;

    public ClimberSubsystem(ClimberIO io)
    {
        this.io = io;
        this.hookServo = new Servo(LobbyConstants.ClimberConstants.kServoId);
    }

    @Override
    public void periodic()
    {
        io.updateInputs(inputs);

        if (inputs.atBottomLimit && inputs.velocityMetersPerSec < 0)
        {
            io.stop();
        }
    }

    public Command runVoltage(double volts)
    {
        return this.run(() -> io.setVoltage(volts));
    }

    public Command stop()
    {
        return this.runOnce(() -> io.stop());
    }

    public Command extendHooks()
    {
        return this.runOnce(() -> hookServo.setAngle(LobbyConstants.ClimberConstants.kHookExtendAngle.getDegrees()));
    }

    public Command retractHooks()
    {
        return this.runOnce(() -> hookServo.setAngle(LobbyConstants.ClimberConstants.kHookRetractAngle.getDegrees()));
    }

    public Command runToPosition(double meters)
    {
        return this.run(() -> io.setPositionControl(meters))
                .until(() -> Math.abs(inputs.positionMeters - meters) < 0.01);
    }

    private Command climbCycle()
    {
        return Commands.sequence(extendHooks(),

                runToPosition(LobbyConstants.ClimberConstants.kRaiseHeightMeters),

                runVoltage(LobbyConstants.ClimberConstants.kPullDownVolts).until(() -> inputs.atBottomLimit));
    }

    public Command climbSequence()
    {
        return Commands.sequence(climbCycle(), climbCycle(), climbCycle(), stop());
    }

    public Command homeClimber()
    {
        return runVoltage(LobbyConstants.ClimberConstants.kHomingVolts).until(() -> inputs.atBottomLimit)
                .andThen(stop()).andThen(() -> io.setPosition(0.0));
    }

    public Command stow()
    {
        return this.run(() ->
        {
            hookServo.setAngle(LobbyConstants.ClimberConstants.kHookRetractAngle.getDegrees());

            if (!inputs.atBottomLimit)
            {
                io.setVoltage(LobbyConstants.ClimberConstants.kHomingVolts);
            } else
            {
                io.stop();
                io.setPosition(0.0);
            }
        });
    }
}