package frc.robot.util;

import java.lang.reflect.Field;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.BuildConstants;

public class NFRLog extends DogLog
{
    public static void log(String key, Status status)
    {
        switch (status.getConflictingSeverity())
        {
        case ERROR:
            logFault(status.getName(), AlertType.kError);
            break;
        default:
            break;
        }

        log(key + "/Severity", status.getConflictingSeverity());
        log(key + "/Name", status.getName());
        log(key + "/Message", status.getMessage());

        var substatuses = status.getSubStatuses();
        for (int i = 0; i < substatuses.length; i++)
        {
            log(String.format("%s/Sub-%d", key, i), substatuses[i]);
        }
    }

    public static void log(String key, SwerveDriveState state)
    {
        log(key + "/Pose", state.Pose);
        log(key + "/Speeds", state.Speeds);
        log(key + "/ModulePositions", state.ModulePositions);
        log(key + "/ModuleStates", state.ModuleStates);
        log(key + "/ModuleTargets", state.ModuleTargets);
        log(key + "/OdometryPeriod", state.OdometryPeriod);
        log(key + "/SuccessfulDaqs", state.SuccessfulDaqs);
        log(key + "/FailedDaqs", state.FailedDaqs);
        log(key + "/RawHeading", state.RawHeading);
    }

    public static void log(String key, TalonFX motor)
    {
        log(key + "/DeviceID", motor.getDeviceID());
        log(key + "/Connected", motor.isConnected());
        log(key + "/Description", motor.getDescription());
        log(key + "/Rotations", motor.getPosition().getValueAsDouble());
        log(key + "/RotationRate", motor.getVelocity().getValueAsDouble());
        log(key + "/Current", motor.getStatorCurrent().getValueAsDouble());
    }

    public static void log(String key, CANcoder encoder)
    {
        log(key + "/DeviceID", encoder.getDeviceID());
        log(key + "/Connected", encoder.isConnected());
        log(key + "/Rotations", encoder.getPosition().getValueAsDouble());
        log(key + "/RotationRate", encoder.getVelocity().getValueAsDouble());
    }

    public static void log(String key, SwerveModule<TalonFX, TalonFX, CANcoder> module)
    {
        log(key + "/DriveMotor", module.getDriveMotor());
        log(key + "/SteerMotor", module.getSteerMotor());
        log(key + "/Encoder", module.getEncoder());
    }

    public static void publishMetadata() {
        for (Field field : BuildConstants.class.getFields())
        {
            try
            {
                // we must publish to NT because DogLog doesn't support logging to root topic yet
                NetworkTableInstance.getDefault().getStringTopic("/Metadata/" + field.getName()).publish()
                        .set(field.get(null).toString());
            } catch (Exception e)
            {
                e.printStackTrace();
            }
        }
    }
}
