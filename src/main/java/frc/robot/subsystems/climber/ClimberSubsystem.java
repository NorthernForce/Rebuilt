package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase
{

    private final ClimberIO io;
    // private final ClimberIOInputsAutoLogged inputs = new
    // ClimberIOInputsAutoLogged();

    // Constructor accepts the IO layer (Hardware or Sim)
    public ClimberSubsystem(ClimberIO io)
    {
        this.io = io;
    }

    @Override
    public void periodic()
    {
        // io.updateInputs(inputs);
        // Logger.processInputs("Climber", inputs);
    }

    /**
     * command to run the climber at a specific voltage.
     * 
     * @param volts Voltage (-12 to 12)
     */
    public Command runVoltage(double volts)
    {
        return this.run(() -> io.setVoltage(volts));
    }

    /**
     * Command to stop the climber.
     */
    public Command stop()
    {
        return this.runOnce(() -> io.stop());
    }
}