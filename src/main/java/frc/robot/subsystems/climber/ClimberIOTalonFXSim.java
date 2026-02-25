package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.lobby.LobbyConstants.ClimberConstants.ClimbLevels;

public class ClimberIOTalonFXSim implements ClimberIO
{
    private final TalonFX motor;
    private final TalonFXSimState simMotor;
    private final ElevatorSim elevatorSim;
    private final double gearRatio;
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private ClimbLevels currentLevel;
    private final ClimbLevels bottomLevel;
    private final ClimbLevels l1;
    private final ClimbLevels l2;
    private final ClimbLevels l3;
    private final double slowSpeed;
    private final double drumRadiusMeters = 0.0127; // 0.5 inch radius drum
    private double setRotations = 0;
    private final double topRotations;
    private static final double TOP_POSITION_TOLERANCE = 5.0; // rotations tolerance
    private final Pose2d upperRedClimbPosition;
    private final Pose2d lowerRedClimbPosition;
    private final Pose2d upperBlueClimbPosition;
    private final Pose2d lowerBlueClimbPosition;
    private final Servo hookServo;

    public ClimberIOTalonFXSim(ClimberParameters params)
    {
        motor = new TalonFX(params.motorID());
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = params.kP();
        config.Slot0.kI = params.kI();
        config.Slot0.kD = params.kD();
        config.Slot0.kV = params.kV();
        config.Slot0.kG = params.kG();
        motor.getConfigurator().apply(config);
        simMotor = motor.getSimState();
        gearRatio = params.gearRatio();

        double massKg = 1.89655188991;
        double maxHeightMeters = params.maxHeight().in(Meters);

        elevatorSim = new ElevatorSim(
                LinearSystemId.createElevatorSystem(DCMotor.getKrakenX60(1), massKg, drumRadiusMeters, gearRatio),
                DCMotor.getKrakenX60(1), 0, maxHeightMeters, true, 0);

        slowSpeed = params.slowSpeed();
        bottomLevel = params.bottomLevel();
        currentLevel = bottomLevel;
        l1 = params.l1Height();
        l2 = params.l2Height();
        l3 = params.l3Height();
        topRotations = params.topRotations();
        upperRedClimbPosition = params.upperRedPrepPose();
        lowerRedClimbPosition = params.lowerRedPrepPose();
        upperBlueClimbPosition = params.upperBluePrepPose();
        lowerBlueClimbPosition = params.lowerBluePrepPose();
        hookServo = new Servo(params.servoID());
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
    }

    @Override
    public void runUp()
    {
        setRotations = topRotations;
        motor.setControl(positionRequest.withPosition(setRotations));
    }

    @Override
    public void homeDown()
    {
        motor.set(-slowSpeed);
    }

    @Override
    public ClimbLevels getLevel()
    {
        return currentLevel;
    }

    @Override
    public boolean atBottom()
    {
        // In simulation, check if the elevator is at or very close to the bottom
        return elevatorSim.getPositionMeters() < 0.001;
    }

    @Override
    public boolean atTop()
    {
        double currentPosition = motor.getPosition().getValueAsDouble();
        return Math.abs(currentPosition - topRotations) < TOP_POSITION_TOLERANCE;
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
    public void setHookPosition(double position)
    {
        hookServo.set(position);
    }

    @Override
    public double getRotations()
    {
        return motor.getPosition().getValueAsDouble();
    }

}
