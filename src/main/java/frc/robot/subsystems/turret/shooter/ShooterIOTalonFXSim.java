package frc.robot.subsystems.turret.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOTalonFXSim extends ShooterIOTalonFX
{
    private final ShooterConstants constants;
    private final TalonFXSimState m_simState;
    private final DCMotorSim m_dcMotorSim;

    public ShooterIOTalonFXSim(ShooterConstants constants)
    {
        super(constants);
        this.constants = constants;
        m_simState = m_motor1.getSimState();
        m_dcMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(constants.kV(), constants.kA()),
                DCMotor.getKrakenX60Foc(2));
    }

    @Override
    public void update()
    {
        super.update();
        m_simState.setSupplyVoltage(RobotController.getInputVoltage());
        m_dcMotorSim.setInputVoltage(
                constants.kMotor1Inverted() ? -m_simState.getMotorVoltage() : m_simState.getMotorVoltage());
        m_dcMotorSim.update(0.02);
        double rotationsPerSecond = m_dcMotorSim.getAngularVelocity().in(RotationsPerSecond);
        double dRot = rotationsPerSecond * 0.02;
        m_simState.addRotorPosition(constants.kMotor1Inverted() ? -dRot : dRot);
        m_simState.setRotorVelocity(constants.kMotor1Inverted() ? -rotationsPerSecond : rotationsPerSecond);
    }
}
