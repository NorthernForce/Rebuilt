package frc.robot.lobby.subsystems.spindexer.flicker;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FlickerIOTalonFXS implements FlickerIO
{
    private TalonFXS m_motor;
    private double m_rampSpeed;
    private double m_errorTolerance;
    private double m_gearRatio;
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

    public FlickerIOTalonFXS(FlickerParameters parameters)
    {
        m_rampSpeed = parameters.rampSpeed();
        m_gearRatio = parameters.gearRatio();
        m_errorTolerance = parameters.errorTolerance();
        m_motor = new TalonFXS(parameters.motorId());

        m_motor.getConfigurator()
                .apply(new CommutationConfigs().withMotorArrangement(MotorArrangementValue.Minion_JST));
        m_motor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive));
    }

    @Override
    public void rampFlicker()
    {
        // Target is mechanism RPS, but TalonFXS expects rotor RPS
        m_motor.set(m_rampSpeed);
    }

    @Override
    public void stopFlicker()
    {
        m_motor.set(0);
    }
}
