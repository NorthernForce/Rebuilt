package frc.robot.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import dev.doglog.DogLog;

public class TunablePID
{
    public static void create(String key, TalonFX motor, TalonFXConfiguration defaultConfig)
    {
        DogLog.tunable(key + "/kP", defaultConfig.Slot0.kP, newP ->
        {
            motor.getConfigurator().apply(defaultConfig.Slot0.withKP(newP));
        });

        DogLog.tunable(key + "/kI", defaultConfig.Slot0.kI, newI ->
        {
            motor.getConfigurator().apply(defaultConfig.Slot0.withKP(newI));
        });

        DogLog.tunable(key + "/kD", defaultConfig.Slot0.kD, newD ->
        {
            motor.getConfigurator().apply(defaultConfig.Slot0.withKP(newD));
        });

        DogLog.tunable(key + "/kV", defaultConfig.Slot0.kV, newV ->
        {
            motor.getConfigurator().apply(defaultConfig.Slot0.withKP(newV));
        });

        DogLog.tunable(key + "/kA", defaultConfig.Slot0.kA, newA ->
        {
            motor.getConfigurator().apply(defaultConfig.Slot0.withKP(newA));
        });
    }

    public static void create(String key, TalonFXS motor, TalonFXSConfiguration defaultConfig)
    {
        DogLog.tunable(key + "/kP", defaultConfig.Slot0.kP, newP ->
        {
            motor.getConfigurator().apply(defaultConfig.Slot0.withKP(newP));
        });

        DogLog.tunable(key + "/kI", defaultConfig.Slot0.kI, newI ->
        {
            motor.getConfigurator().apply(defaultConfig.Slot0.withKP(newI));
        });

        DogLog.tunable(key + "/kD", defaultConfig.Slot0.kD, newD ->
        {
            motor.getConfigurator().apply(defaultConfig.Slot0.withKP(newD));
        });

        DogLog.tunable(key + "/kV", defaultConfig.Slot0.kV, newV ->
        {
            motor.getConfigurator().apply(defaultConfig.Slot0.withKP(newV));
        });

        DogLog.tunable(key + "/kA", defaultConfig.Slot0.kA, newA ->
        {
            motor.getConfigurator().apply(defaultConfig.Slot0.withKP(newA));
        });
    }

    private TunablePID()
    {
    }
}
