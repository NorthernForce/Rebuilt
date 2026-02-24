package frc.robot.subsystems.turret.suzie;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.sim.TalonFXSSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SuzieIOTalonFXSSim extends SuzieIOTalonFXS
{
    private SuzieConstants constants;
    private TalonFXSSimState m_simState;
    private DCMotorSim m_dcMotorSim;

    public SuzieIOTalonFXSSim(SuzieConstants constants)
    {
        super(constants);
        this.constants = constants;
        m_simState = m_motor.getSimState();
        DCMotor motor;
        switch (constants.kMotorArrangement())
        {
        case Minion_JST:
            motor = DCMotor.getMinion(1);
            break;
        case NEO550_JST:
            motor = DCMotor.getNeo550(1);
            break;

        default:
            motor = DCMotor.getMinion(1);
        }
        m_dcMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(constants.kV(), constants.kA()), motor);
    }

    @Override
    public void update()
    {
        super.update();
        m_simState.setSupplyVoltage(RobotController.getInputVoltage());
        m_dcMotorSim
                .setInputVoltage(constants.kInverted() ? -m_simState.getMotorVoltage() : m_simState.getMotorVoltage());
        m_dcMotorSim.update(0.02);
        double rotationsPerSecond = m_dcMotorSim.getAngularVelocity().in(RotationsPerSecond) * constants.kGearRatio();
        double dRot = rotationsPerSecond * 0.02;
        m_simState.addRotorPosition(constants.kInverted() ? -dRot : dRot);
        m_simState.setRotorVelocity(constants.kInverted() ? -rotationsPerSecond : rotationsPerSecond);
    }
}
