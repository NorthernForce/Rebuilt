package frc.robot.lobby.subsystems.climber;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ClimberIOTalonFXSim implements ClimberIO
{
    private final TalonFX motor;
    private final TalonFXSimState simMotor;
    private final ElevatorSim elevatorSim;

    private final double drumRadiusMeters = 0.0127;
    private final Pose2d upperRedClimbPosition;
    private final Pose2d lowerRedClimbPosition;
    private final Pose2d upperBlueClimbPosition;
    private final Pose2d lowerBlueClimbPosition;
    private final double power;
    private final double gearRatio;

    public ClimberIOTalonFXSim(ClimberParameters params)
    {
        gearRatio = params.gearRatio();
        motor = new TalonFX(params.motorID());
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = params.topSoftRotations().in(Rotations);
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = params.topSoftRotations().in(Rotations);
        config.MotorOutput.Inverted = params.inverted() ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        motor.getConfigurator().apply(config);
        simMotor = motor.getSimState();

        double massKg = 1.89655188991;

        elevatorSim = new ElevatorSim(
                LinearSystemId.createElevatorSystem(DCMotor.getKrakenX60(1), massKg, drumRadiusMeters, gearRatio),
                DCMotor.getKrakenX60(1), 0, params.maxHeight().in(Meters), true, 0);

        upperRedClimbPosition = params.upperRedPrepPose();
        lowerRedClimbPosition = params.lowerRedPrepPose();
        upperBlueClimbPosition = params.upperBluePrepPose();
        lowerBlueClimbPosition = params.lowerBluePrepPose();
        power = params.dutyCyclePower();
    }

    @Override
    public void updateSimulation()
    {
        simMotor.setSupplyVoltage(RobotController.getBatteryVoltage());

        double motorVoltage = simMotor.getMotorVoltage();
        elevatorSim.setInputVoltage(motorVoltage);

        elevatorSim.update(0.02);

        double positionMeters = elevatorSim.getPositionMeters();
        double motorRotations = positionMeters / (2 * Math.PI * drumRadiusMeters) * gearRatio;
        double velocityMps = elevatorSim.getVelocityMetersPerSecond();
        double motorRps = velocityMps / (2 * Math.PI * drumRadiusMeters) * gearRatio;

        simMotor.setRawRotorPosition(motorRotations);
        simMotor.setRotorVelocity(motorRps);

        DogLog.log("ClimberSim/MotorVoltage", motorVoltage);
        DogLog.log("ClimberSim/PositionMeters", positionMeters);
        DogLog.log("ClimberSim/MotorRotations", motorRotations);
        DogLog.log("ClimberSim/VelocityMps", velocityMps);
    }

    @Override
    public void runUp()
    {
        motor.set(power);
    }

    @Override
    public void homeDown()
    {
        motor.set(-power);
    }

    @Override
    public void stopMotor()
    {
        motor.stopMotor();
    }

    @Override
    public Pose2d getNearestPreclimbPosition(Pose2d robotPose)
    {
        DogLog.log("Climber/RobotPose", robotPose);
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
        {
            double distanceToUpperBlue = robotPose.getTranslation()
                    .getDistance(upperBlueClimbPosition.getTranslation());
            double distanceToLowerBlue = robotPose.getTranslation()
                    .getDistance(lowerBlueClimbPosition.getTranslation());
            return distanceToUpperBlue < distanceToLowerBlue ? upperBlueClimbPosition : lowerBlueClimbPosition;
        } else if (DriverStation.getAlliance().get() == Alliance.Red)
        {
            double distanceToUpperRed = robotPose.getTranslation().getDistance(upperRedClimbPosition.getTranslation());
            double distanceToLowerRed = robotPose.getTranslation().getDistance(lowerRedClimbPosition.getTranslation());
            return distanceToUpperRed < distanceToLowerRed ? upperRedClimbPosition : lowerRedClimbPosition;
        } else
        {
            return robotPose;
        }
    }

    @Override
    public double getRotations()
    {
        return motor.getPosition().getValueAsDouble();
    }

}
