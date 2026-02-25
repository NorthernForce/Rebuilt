package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.lobby.LobbyConstants.ClimberConstants.ClimbLevels;

public class ClimberIOTalonFX implements ClimberIO
{
    private final int motorID;
    private final int sensorID;
    private final TalonFX motor;
    private final DigitalInput limitSwitch;
    private final double gearRatio;
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private ClimbLevels currentLevel;
    private final ClimbLevels bottomLevel;
    private final ClimbLevels l1;
    private final ClimbLevels l2;
    private final ClimbLevels l3;
    private final double slowSpeed;
    private double setRotations = 0;
    private final double topRotations;
    private static final double TOP_POSITION_TOLERANCE = 5.0; // rotations tolerance
    private final Pose2d upperRedClimbPosition;
    private final Pose2d lowerRedClimbPosition;
    private final Pose2d upperBlueClimbPosition;
    private final Pose2d lowerBlueClimbPosition;
    private final Servo hookServo;

    public ClimberIOTalonFX(ClimberParameters params)
    {
        motorID = params.motorID();
        sensorID = params.bottomLimitSwitchId();
        motor = new TalonFX(motorID);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = params.kP();
        config.Slot0.kI = params.kI();
        config.Slot0.kD = params.kD();
        config.Slot0.kV = params.kV();
        config.Slot0.kG = params.kG();
        motor.getConfigurator().apply(config);
        limitSwitch = new DigitalInput(sensorID);
        gearRatio = params.gearRatio();
        slowSpeed = params.slowSpeed();
        bottomLevel = params.bottomLevel();
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
        return limitSwitch.get();
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

    public Pose2d getNearestPreclimbPosition(Pose2d robotPose)
    {
        DogLog.log("Climber/RobotPose", robotPose);
        if (DriverStation.getAlliance().get() == Alliance.Blue)
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
