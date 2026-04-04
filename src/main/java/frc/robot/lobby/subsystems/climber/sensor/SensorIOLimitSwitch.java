package frc.robot.lobby.subsystems.climber.sensor;

import edu.wpi.first.wpilibj.DigitalInput;

public class SensorIOLimitSwitch implements SensorIO
{
    private final DigitalInput m_sensor;
    private final boolean signalCreatedOnPress;

    public SensorIOLimitSwitch(int sensorPin, boolean signalCreatedOnPress)
    {
        m_sensor = new DigitalInput(sensorPin);
        this.signalCreatedOnPress = signalCreatedOnPress;
    }

    @Override
    public boolean atLimit()
    {
        if (signalCreatedOnPress)
        {
            return m_sensor.get();
        } else
        {
            return !m_sensor.get();
        }
    }
}
