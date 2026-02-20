package frc.robot.lobby.subsystems.spindexer.flicker;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.sim.TalonFXSSimState;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlickerIOTalonFXSSim implements FlickerIO
{
    private double m_gearRatio;
    private double m_rampSpeed;
    private double m_errorTolerance;
    private double m_simMaxRpm;
    private TalonFXS m_motorController;

    private TalonFXSSimState m_simState;
    private FlywheelSim m_flywheelSim;

    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
    private double m_rotorPosition = 0;
    private final Current jamCurrentThreshold;
    private final Time jamTimeout;
    private double nanoTimeLastChecked = 0.0;
    private final double dejamSpeed;

    public FlickerIOTalonFXSSim(FlickerSimParameters parameters)
    {
        m_rampSpeed = parameters.rampSpeed();
        m_errorTolerance = parameters.errorTolerance();
        m_gearRatio = parameters.gearRatio();
        m_motorController = new TalonFXS(parameters.motorId());
        jamCurrentThreshold = parameters.jamCurrentThreshold();
        jamTimeout = parameters.jamTimeout();
        // Configure PID gains for velocity control
        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.Slot0.kV = parameters.kV();
        config.Slot0.kP = parameters.kP();
        config.Slot0.kI = parameters.kI();
        config.Slot0.kD = parameters.kD();
        m_motorController.getConfigurator().apply(config);

        DCMotor minionMotor = DCMotor.getMinion(1);
        LinearSystem<N1, N1, N1> systemId = LinearSystemId.createFlywheelSystem(minionMotor, parameters.simMoi(),
                parameters.gearRatio());
        m_flywheelSim = new FlywheelSim(systemId, minionMotor);

        m_simState = m_motorController.getSimState();
        m_simMaxRpm = parameters.simMaxRpm();
        dejamSpeed = parameters.dejamSpeed();

    }

    public void simulationPeriodic(double dtSeconds)
    {
        m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        double motorVoltage = m_simState.getMotorVoltage();

        m_flywheelSim.setInputVoltage(motorVoltage);
        m_flywheelSim.update(dtSeconds);

        // Mechanism velocity -> Rotor velocity (multiply by gear ratio)
        double mechanismRps = m_flywheelSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
        double rotorRps = mechanismRps * m_gearRatio;

        // Integrate position
        m_rotorPosition += rotorRps * dtSeconds;

        m_simState.setRotorVelocity(rotorRps);
        m_simState.setRawRotorPosition(m_rotorPosition);
    }

    @Override
    public void updateSimulation(double dtSeconds)
    {
        simulationPeriodic(dtSeconds);
    }

    @Override
    public void rampFlicker()
    {
        // Target is mechanism RPS, but TalonFXS expects rotor RPS
        double targetMechanismRps = m_rampSpeed;
        double targetRotorRps = targetMechanismRps * m_gearRatio;
        System.out.println("rampFlicker called! Target Mechanism RPS: " + targetMechanismRps + " | Target Rotor RPS: "
                + targetRotorRps);
        m_motorController.setControl(m_velocityRequest.withVelocity(targetRotorRps));
    }

    @Override
    public void stopFlicker()
    {
        m_motorController.setControl(m_velocityRequest.withVelocity(0));
    }

    @Override
    public double getPower()
    {
        return m_motorController.get();
    }

    @Override
    public void setPower(double power)
    {
        m_rampSpeed = power;
    }

    @Override
    public double getTargetPower()
    {
        return m_rampSpeed;
    }

    @Override
    public boolean getJammed()
    {
        double currentTime = System.nanoTime();
        if (m_motorController.getTorqueCurrent().getValueAsDouble() > jamCurrentThreshold.in(Amps))
        {
            if (currentTime - nanoTimeLastChecked > jamTimeout.in(Seconds) * Math.pow(10, 9))
            {
                return true;
            } else
            {
                return false;
            }
        } else
        {
            nanoTimeLastChecked = currentTime;
            return false;
        }
    }

    @Override
    public void dejam()
    {
        m_motorController.set(-dejamSpeed);
    }

    @Override
    public void resetJamDetection()
    {
        nanoTimeLastChecked = System.nanoTime();
    }
}