package frc.robot.subsystems.flywheelshooter.Flywheel;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;

import frc.robot.lobby.LobbyConstants;

public class FlywheelIOFXS implements FlywheelIO
{

    private final TalonFXS motor;

    public FlywheelIOFXS()
    {
        motor = new TalonFXS(LobbyConstants.FlywheelShooterConstants.Flywheel.kMotorID);
        TalonFXSConfiguration config = new TalonFXSConfiguration();

        var slot0Configs = config.Slot0;

        slot0Configs.kS = LobbyConstants.FlywheelShooterConstants.Flywheel.kS;
        slot0Configs.kV = LobbyConstants.FlywheelShooterConstants.Flywheel.kV;
        slot0Configs.kA = LobbyConstants.FlywheelShooterConstants.Flywheel.kA;
        slot0Configs.kP = LobbyConstants.FlywheelShooterConstants.Flywheel.kP;
        slot0Configs.kI = LobbyConstants.FlywheelShooterConstants.Flywheel.kI;
        slot0Configs.kD = LobbyConstants.FlywheelShooterConstants.Flywheel.kD;
        slot0Configs.kG = LobbyConstants.FlywheelShooterConstants.Flywheel.kG;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    }
}