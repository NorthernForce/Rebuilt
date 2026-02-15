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
        return this.runOnce(() -> hookServo.setAngle(LobbyConstants.ClimberConstants.kHookExtendAngle));
    }

    public Command retractHooks()
    {
        return this.runOnce(() -> hookServo.setAngle(LobbyConstants.ClimberConstants.kHookRetractAngle));
    }

    public Command climbSequence()
    {
        return Commands.sequence(extendHooks(),
                runVoltage(LobbyConstants.ClimberConstants.kRaiseVolts)
                        .withTimeout(LobbyConstants.ClimberConstants.kRaiseTimeoutSeconds),
                runVoltage(LobbyConstants.ClimberConstants.kPullDownVolts).until(() -> inputs.atBottomLimit), stop());
    }

    public Command homeClimber()
    {
        return runVoltage(LobbyConstants.ClimberConstants.kHomingVolts).until(() -> inputs.atBottomLimit)
                .andThen(stop()).andThen(() -> io.setPosition(0.0));
    }
}