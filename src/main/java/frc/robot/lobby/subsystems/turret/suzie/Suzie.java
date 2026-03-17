package frc.robot.lobby.subsystems.turret.suzie;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.VoltageOut;

import dev.doglog.DogLog;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
        m_sysId = new SysIdRoutine(new SysIdRoutine.Config(Volts.of(0.5).per(Second), Volts.of(6), // Reduce dynamic
                                                                                                   // step voltage to 6
                                                                                                   // V
                Seconds.of(4), // Time out after 4 s
                state -> DogLog.log("Turntable_SysId_State", state.toString()) // Log state with DogLog
        ), new SysIdRoutine.Mechanism(output -> io.setMotorControl(new VoltageOut(output)), // Apply voltage
                                                                                            // output to motor
                log -> log.motor("Turntable").voltage(io.getVoltage()).angularPosition(io.getAngle())
                        .angularVelocity(io.getVelocity()), // Log motor voltage, position, and velocity
                this // Require this subsystem
        ));
    }

    public void setSpeed(double speed)
    {
        io.setSpeed(speed);
    }

    public void start()
    {
        io.start();
    }

    public void stop()
    {
        io.stop();
    }

    public SuzieIO getIO()
    {
        return io;
    }

    public Angle getAngle()
    {
        return io.getAngle();
    }

    public void resetAngle()
    {
        io.resetAngle(Degrees.of(0.0));
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

    public void resetCRT()
    {
        io.resetCRT();
    }

    public void resetEncoders()
    {
        io.setDrivingEncoderOffset(Degrees.of(0.0));
        io.setSensingEncoderOffset(Degrees.of(0.0));

        Preferences.setDouble("drivingEncoderOffset", io.getDrivingEncoderAngle().in(Degrees));
        Preferences.setDouble("sensingEncoderOffset", io.getSensingEncoderAngle().in(Degrees));

        io.setDrivingEncoderOffset(io.getDrivingEncoderAngle());
        io.setSensingEncoderOffset(io.getSensingEncoderAngle());
        io.resetCRT();
    }

    public Command getSysIdQuasistaticForward()
    {
        return constructFullSysIdCommand(m_sysId.quasistatic(SysIdRoutine.Direction.kForward));
    }

    public Command getSysIdQuasistaticReverse()
    {
        return constructFullSysIdCommand(m_sysId.quasistatic(SysIdRoutine.Direction.kReverse));
    }

    public Command getSysIdDynamicForward()
    {
        return constructFullSysIdCommand(m_sysId.dynamic(SysIdRoutine.Direction.kForward));
    }

    public Command getSysIdDynamicReverse()
    {
        return constructFullSysIdCommand(m_sysId.dynamic(SysIdRoutine.Direction.kReverse));
    }

    private Command constructFullSysIdCommand(Command sysId)
    {
        return Commands.sequence(runOnce(() -> io.disableSoftLimits()), sysId, runOnce(() -> io.enableSoftLimits()));
    }

    @Override
    public void periodic()
    {
        io.update();
    }

    public double getCurrent()
    {
        return io.getCurrent();
    }
}
