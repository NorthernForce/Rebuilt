package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class SimElevator extends Elevator
{
    private final TalonFXSimState state;
    private final ElevatorSim sim;

    public SimElevator(int canID, int sensorID, ElevatorConfig config)
    {
        super(canID, sensorID, config);
        state = motor.getSimState();
        sim = new ElevatorSim(DCMotor.getKrakenX60(1), config.kGearRatio(), config.kMass().in(Kilograms),
                config.kSprocketCircumference().in(Meters) / (2 * Math.PI), 0, config.kUpperLimit().in(Meters), true,
                0);
    }

    @Override
    public void update()
    {
        BaseStatusSignal.refreshAll(position);

        state.setSupplyVoltage(RobotController.getInputVoltage());
        sim.setInputVoltage(config.kInverted() ? -state.getMotorVoltage() : state.getMotorVoltage());
        sim.update(0.02);
        double sprocketRotations = sim.getVelocityMetersPerSecond() / (config.kSprocketCircumference().in(Meters));
        double rotationsPerSecond = sprocketRotations * config.kGearRatio();
        double dRot = rotationsPerSecond * 0.02;
        state.addRotorPosition(config.kInverted() ? -dRot : dRot);
        state.setRotorVelocity(RotationsPerSecond.of(config.kInverted() ? -rotationsPerSecond : rotationsPerSecond));
    }
}
