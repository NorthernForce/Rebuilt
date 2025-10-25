package frc.robot.ralph.subsystems.shooter;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ralph.subsystems.shooter.commands.Intake;
import frc.robot.ralph.subsystems.shooter.commands.Outtake;
import frc.robot.util.CTREUtil;
import frc.robot.util.Status;

/**
 * Basic ejector/shooter mechanism - basic, no beam break sensor technology
 * implemented
 */

public class ManipulatorTalonFX extends SubsystemBase
{
    private int m_id;
    private boolean m_canIntake = true;
    private Timer m_timer = new Timer();
    private TalonFXS m_talonFXS;
    private double m_intakeSpeed;
    private double m_outtakeSpeed;
    private double m_reintakeSpeed;
    private Time m_reintakeTimeout;
    private Time m_bruteOuttakeTimeout;
    private double m_slowOuttakeSpeed;
    private double m_purgeSpeed;
    private DigitalInput m_sensor;
    private int m_sensorId;
    private boolean m_inverted;
    private ManipulatorState m_state = ManipulatorState.HAMBRIENTO;

    /**
     * Intake/Shoot Ralph mechanism
     * 
     * @param id    ID of TalonFX
     * @param speed speed to set motor to
     */

    public ManipulatorTalonFX(int id, double intakeSpeed, double outtakeSpeed, double slowOuttakeSpeed,
            double purgeSpeed, double reintakeSpeed, Time reintakeTimeout, Time bruteOuttakeTimeout, boolean inverted,
            int sensorId, double statorCurrentLimit, boolean statorCurrentLimitEnable)
    {
        m_intakeSpeed = intakeSpeed;
        m_outtakeSpeed = outtakeSpeed;
        m_purgeSpeed = purgeSpeed;
        m_slowOuttakeSpeed = slowOuttakeSpeed;
        m_reintakeSpeed = reintakeSpeed;
        m_reintakeTimeout = reintakeTimeout;
        m_bruteOuttakeTimeout = bruteOuttakeTimeout;
        m_id = id;
        m_sensorId = sensorId;
        m_inverted = inverted;
        m_talonFXS = new TalonFXS(id);
        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = statorCurrentLimitEnable;
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        config.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;
        m_talonFXS.getConfigurator().apply(config);
        m_sensor = new DigitalInput(sensorId);
    }

    public int getSensorId()
    {
        return m_sensorId;
    }

    public boolean isInverted()
    {
        return m_inverted;
    }

    /**
     * Set motor speed
     * 
     * @param speed speed to set motor to
     */

    public void set(double speed)
    {
        m_talonFXS.set(speed);
    }

    /**
     * Get command to intake
     */

    public Command intake()
    {
        return new Intake(this);
    }

    /**
     * Get command to outtake
     */

    public Command outtake()
    {
        return new Outtake(this);
    }

    /**
     * Stop the motor
     */

    public void stop()
    {
        set(0);
    }

    public boolean hasCoralInSensor()
    {
        return !m_sensor.get();
    }

    public boolean hasCoral()
    {
        return m_state == ManipulatorState.FELIZ;
    }

    public void setCanIntake(boolean canIntake)
    {
        m_canIntake = canIntake;
    }

    public Command slowOuttake()
    {
        return runOnce(() ->
        {
            m_state = ManipulatorState.EXPULSANDO_LENTO;
        }).alongWith(Commands.waitUntil(() -> !hasCoralInSensor()));
    }

    public ManipulatorState getState()
    {
        return m_state;
    }

    public void setState(ManipulatorState state)
    {
        m_state = state;
        if (state == ManipulatorState.EXPULSANDO_BRUTO)
        {
            m_timer.restart();
        }
    }

    /**
     * Get the ID of the TalonFX
     * 
     * @return ID of TalonFX
     */

    public int getId()
    {
        return m_id;
    }

    @Override
    public void periodic()
    {
        switch (m_state)
        {
        case HAMBRIENTO:
            set(m_canIntake ? m_intakeSpeed : 0);
            if (hasCoralInSensor())
            {
                m_state = ManipulatorState.PURGANDO;
            }
            break;
        case PURGANDO:
            set(-m_purgeSpeed);
            if (!hasCoralInSensor())
            {
                m_state = ManipulatorState.REINGRESO;
                m_timer.restart();
            }
            break;
        case REINGRESO:
            set(m_reintakeSpeed);
            if (m_timer.hasElapsed(m_reintakeTimeout.in(Seconds)))
            {
                m_state = ManipulatorState.HAMBRIENTO;
            }
            if (hasCoralInSensor())
            {
                m_state = ManipulatorState.FELIZ;
            }
            break;
        case FELIZ:
            stop();
            break;
        case EXPULSANDO:
            set(m_slowOuttakeSpeed);
            if (!hasCoralInSensor())
            {
                m_state = ManipulatorState.HAMBRIENTO;
            }
            break;
        case EXPULSANDO_LENTO:
            set(m_slowOuttakeSpeed);
            if (!hasCoralInSensor())
            {
                m_state = ManipulatorState.HAMBRIENTO;
            }
            break;
        case EXPULSANDO_BRUTO:
            set(m_outtakeSpeed);
            if (m_timer.hasElapsed(m_bruteOuttakeTimeout.in(Seconds)))
            {
                m_state = ManipulatorState.HAMBRIENTO;
            }
        }
    }

    /**
     * Get the status of the manipulator. Since the sensor cannot be "detected", the
     * severity is always OK, but that does not mean that it is actively sensing.
     * 
     * @return the status of the manipulator, including motor and sensor status
     */

    public Status getStatus()
    {
        Status sensorStatus = new Status("Manipulator Sensor Status", () -> true,
                () -> (m_sensor.get() ? "No Coral" : "Coral Detected"));
        Status motorStatus = CTREUtil.getTalonFXSStatus(m_talonFXS);
        return new Status("Manipulator Status", motorStatus, sensorStatus);
    }

    /**
     * States for the manipulator
     */

    public enum ManipulatorState
    {
        HAMBRIENTO, PURGANDO, REINGRESO, EXPULSANDO, FELIZ, EXPULSANDO_LENTO, EXPULSANDO_BRUTO
    }
}