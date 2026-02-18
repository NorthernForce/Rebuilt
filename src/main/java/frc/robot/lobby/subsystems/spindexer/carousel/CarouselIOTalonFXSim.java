package frc.robot.lobby.subsystems.spindexer.carousel;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class CarouselIOTalonFXSim extends CarouselIOTalonFX
{
    private final CarouselConstants constants;
    private final TalonFXSimState m_simState;
    private final DCMotorSim m_dcMotorSim;

    public CarouselIOTalonFXSim(CarouselConstants constants)
    {
        super(constants);
        this.constants = constants;
        m_simState = m_motor.getSimState();
        m_dcMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(0.0, 0.0), DCMotor.getKrakenX60Foc(1));
    }

    @Override
    public void update()
    {
        super.update();
        m_simState.setSupplyVoltage(RobotController.getInputVoltage());
        m_dcMotorSim
                .setInputVoltage(constants.kInverted() ? -m_simState.getMotorVoltage() : m_simState.getMotorVoltage());
        m_dcMotorSim.update(0.02);
        double rotationsPerSecond = m_dcMotorSim.getAngularVelocity().in(RotationsPerSecond);
        double dRot = rotationsPerSecond * 0.02;
        m_simState.addRotorPosition(constants.kInverted() ? -dRot : dRot);
        m_simState.setRotorVelocity(constants.kInverted() ? -rotationsPerSecond : rotationsPerSecond);
    }
}
