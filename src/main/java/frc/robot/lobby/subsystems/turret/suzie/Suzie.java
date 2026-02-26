package frc.robot.lobby.subsystems.turret.suzie;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;

import dev.doglog.DogLog;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Suzie extends SubsystemBase
{
    protected final SuzieIO io;
    protected final DoubleSubscriber m_targetAngleOverride;
    protected final SysIdRoutine m_sysId;

    public Suzie(SuzieIO io)
    {
        this.io = io;
        m_targetAngleOverride = DogLog.tunable("Turret/Suzie/Target Angle Override (Degrees)", Degrees.of(0.0),
                newAngle ->
                {
                    setTargetAngle(Degrees.of(newAngle));
                });
        m_sysId = new SysIdRoutine(new SysIdRoutine.Config(null, // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 V
                null, // Use default timeout (10 s)
                state -> SignalLogger.writeString("SysIdTurret", state.toString() // Log state with SignalLogger class
                )), new SysIdRoutine.Mechanism(output -> io.setMotorControl(new VoltageOut(output)), // Apply voltage
                                                                                                     // output to motor
                        null, // Do not log
                        this // Require this subsystem
        ));
    }

    public Command setSpeed(double speed)
    {
        return run(() -> io.setSpeed(speed));
    }

    public Command start()
    {
        return run(() -> io.start());
    }

    public Command stop()
    {
        return run(() -> io.stop());
    }

    public SuzieIO getIO()
    {
        return io;
    }

    public Angle getAngle()
    {
        return io.getAngle();
    }

    public Angle getTargetAngle()
    {
        return io.getTargetAngle();
    }

    public boolean isAtTargetAngle()
    {
        return io.isAtTargetAngle();
    }

    public void setTargetAngle(Angle angle)
    {
        io.setTargetAngle(angle);
    }

    public Command getSysIdQuasistaticForward()
    {
        return m_sysId.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command getSysIdQuasistaticReverse()
    {
        return m_sysId.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command getSysIdDynamicForward()
    {
        return m_sysId.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command getSysIdDynamicReverse()
    {
        return m_sysId.dynamic(SysIdRoutine.Direction.kReverse);
    }

    @Override
    public void periodic()
    {
        io.update();
    }
}
