package frc.robot.lobby.subsystems.turret.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Shooter extends SubsystemBase
{
    protected final ShooterIO io;
    protected final DoubleSubscriber m_targetVelocityOverride;
    protected final DoubleSubscriber m_targetDutyCycleOverride;
    protected final SysIdRoutine m_sysId;

    public Shooter(ShooterIO io)
    {
        this.io = io;
        m_targetVelocityOverride = DogLog.tunable("Turret/Shooter/Target Velocity Override (RotationsPerSecond)",
                RotationsPerSecond.of(0.0), newVelocity ->
                {
                    setTargetSpeed(RotationsPerSecond.of(newVelocity));
                });
        m_targetDutyCycleOverride = DogLog.tunable("Turret/Shooter/Target Duty Cycle Override", 0.0, newDutyCycle ->
        {
            setTargetDutyCycle(newDutyCycle);
        });
        m_sysId = new SysIdRoutine(new SysIdRoutine.Config(null, // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 V
                null, // Use default timeout (10 s)
                state -> SignalLogger.writeString("SysIdShooter", state.toString() // Log state with SignalLogger class
                )), new SysIdRoutine.Mechanism(output -> io.setMotorControl(new VoltageOut(output)), // Apply voltage
                                                                                                     // output to motor
                        null, // Do not log
                        this // Require this subsystem
        ));
    }

    public ShooterIO getIO()
    {
        return io;
    }

    public AngularVelocity getTargetSpeed()
    {
        return io.getTargetSpeed();
    }

    public Command start()
    {
        return run(() -> io.start());
    }

    public Command stop()
    {
        return run(() -> io.stop());
    }

    public void setTargetSpeed(AngularVelocity speed)
    {
        io.setTargetSpeed(speed);
    }

    public void setTargetDutyCycle(double speed)
    {
        io.setTargetDutyCycle(speed);
    }

    public AngularVelocity getSpeed()
    {
        return io.getSpeed();
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
        DogLog.log("Turret/Shooter/TargetSpeed", io.getTargetSpeed().in(RotationsPerSecond));
    }
}
