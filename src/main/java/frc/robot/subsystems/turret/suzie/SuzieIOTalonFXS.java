package frc.robot.subsystems.turret.suzie;

import frc.robot.subsystems.turret.suzie.SuzieIO;

public class SuzieIOTalonFXS implements SuzieIO {
    protected final TalonFXS m_motor;
    protected final StatusSignal<Angle> m_position;
    protected final StatusSignal<Temperature> m_temperature;
    protected final StatusSignal<Voltage> m_voltage;
    protected final StatusSignal<Current> m_current;
    protected final StatusSignal<AngularVelocity> m_velocity;
    protected final StatusSignal<AngularVelocity> m_rotorVelocity;
    protected final Supplier<boolean> m_isPresent;

    public SuzieIOTalonFXS(SuzieIO.SuzieConstants constants){}
}