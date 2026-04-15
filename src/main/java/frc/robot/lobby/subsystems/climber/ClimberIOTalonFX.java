package frc.robot.lobby.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ClimberIOTalonFX implements ClimberIO
{
    private final int motorID;
    private final TalonFX motor;

    private final Pose2d upperRedPreClimbPosition;
    private final Pose2d lowerRedPreClimbPosition;
    private final Pose2d upperBluePreClimbPosition;
    private final Pose2d lowerBluePreClimbPosition;
    private final Pose2d upperRedClimbPosition;
    private final Pose2d lowerRedClimbPosition;
    private final Pose2d upperBlueClimbPosition;
    private final Pose2d lowerBlueClimbPosition;
    private final double power;

    public ClimberIOTalonFX(ClimberParameters params)
    {
        motorID = params.motorID();
        motor = new TalonFX(motorID);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = params.inverted() ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor.getConfigurator().apply(config);
        upperRedPreClimbPosition = params.upperRedPrepPose();
        lowerRedPreClimbPosition = params.lowerRedPrepPose();
        upperBluePreClimbPosition = params.upperBluePrepPose();
        lowerBluePreClimbPosition = params.lowerBluePrepPose();
        upperRedClimbPosition = params.upperRedPose();
        lowerRedClimbPosition = params.lowerRedPose();
        upperBlueClimbPosition = params.upperBluePose();
        lowerBlueClimbPosition = params.lowerBluePose();
        power = params.dutyCyclePower();
    }

    @Override
    public void runUp()
    {
        motor.set(1.0);
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

    public Pose2d getNearestPreclimbPosition(Pose2d robotPose)
    {
        DogLog.log("Climber/RobotPose", robotPose);
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
        {
            double distanceToUpperBlue = robotPose.getTranslation()
                    .getDistance(upperBluePreClimbPosition.getTranslation());
            double distanceToLowerBlue = robotPose.getTranslation()
                    .getDistance(lowerBluePreClimbPosition.getTranslation());
            return distanceToUpperBlue < distanceToLowerBlue ? upperBluePreClimbPosition : lowerBluePreClimbPosition;
        } else if (DriverStation.getAlliance().get() == Alliance.Red)
        {
            double distanceToUpperRed = robotPose.getTranslation()
                    .getDistance(upperRedPreClimbPosition.getTranslation());
            double distanceToLowerRed = robotPose.getTranslation()
                    .getDistance(lowerRedPreClimbPosition.getTranslation());
            return distanceToUpperRed < distanceToLowerRed ? upperRedPreClimbPosition : lowerRedPreClimbPosition;
        } else
        {
            return robotPose;
        }
    }

    @Override
    public Pose2d getNearestClimbPosition(Pose2d robotPose)
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
