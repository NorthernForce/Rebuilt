package frc.robot.lobby.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Intake extends SubsystemBase
{
    public IntakeIO io;

    private final SysIdRoutine m_armSysIdRoutine;

    public Intake(IntakeIO io)
    {
        this.io = io;

        m_armSysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // Default ramp rate (1 V/s)
                        Volts.of(4), // Dynamic step voltage — keep low for an arm to avoid damage
                        null, // Default timeout (10 s)
                        state -> SignalLogger.writeString("IntakeArm_SysId_State", state.toString())),
                new SysIdRoutine.Mechanism(
                        voltage -> io.setArmVoltage(voltage),
                        null, // Using CTRE SignalLogger, no WPILib log consumer needed
                        this));
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

    /**
     * Quasistatic SysId test — slowly ramps voltage on the intake arm.
     * Use this to measure kS (static friction) and kV (velocity gain).
     */
    public Command sysIdArmQuasistatic(SysIdRoutine.Direction direction)
    {
        return m_armSysIdRoutine.quasistatic(direction);
    }

    /**
     * Dynamic SysId test — applies a step voltage to the intake arm.
     * Use this to measure kA (acceleration gain).
     */
    public Command sysIdArmDynamic(SysIdRoutine.Direction direction)
    {
        return m_armSysIdRoutine.dynamic(direction);
    }

}
