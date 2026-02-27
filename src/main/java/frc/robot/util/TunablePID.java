package frc.robot.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import dev.doglog.DogLog;

public class TunablePID
{
    public static void createBasic(String key, TalonFX motor, TalonFXConfiguration defaultConfig)
    {
        DogLog.tunable(key + "/kP", defaultConfig.Slot0.kP, newP ->
        {
            motor.getConfigurator().apply(defaultConfig.Slot0.withKP(newP));
        });

        DogLog.tunable(key + "/kI", defaultConfig.Slot0.kI, newI ->
        {
            motor.getConfigurator().apply(defaultConfig.Slot0.withKI(newI));
        });

        DogLog.tunable(key + "/kD", defaultConfig.Slot0.kD, newD ->
        {
            motor.getConfigurator().apply(defaultConfig.Slot0.withKD(newD));
        });

        DogLog.tunable(key + "/kV", defaultConfig.Slot0.kV, newV ->
        {
            motor.getConfigurator().apply(defaultConfig.Slot0.withKV(newV));
        });

        DogLog.tunable(key + "/kA", defaultConfig.Slot0.kA, newA ->
        {
            motor.getConfigurator().apply(defaultConfig.Slot0.withKA(newA));
        });
    }

    public static void createBasic(String key, TalonFXS motor, TalonFXSConfiguration defaultConfig)
    {
        DogLog.tunable(key + "/kP", defaultConfig.Slot0.kP, newP ->
        {
            motor.getConfigurator().apply(defaultConfig.Slot0.withKP(newP));
        });

        DogLog.tunable(key + "/kI", defaultConfig.Slot0.kI, newI ->
        {
            motor.getConfigurator().apply(defaultConfig.Slot0.withKI(newI));
        });

        DogLog.tunable(key + "/kD", defaultConfig.Slot0.kD, newD ->
        {
            motor.getConfigurator().apply(defaultConfig.Slot0.withKD(newD));
        });

        DogLog.tunable(key + "/kV", defaultConfig.Slot0.kV, newV ->
        {
            motor.getConfigurator().apply(defaultConfig.Slot0.withKV(newV));
        });

        DogLog.tunable(key + "/kA", defaultConfig.Slot0.kA, newA ->
        {
            motor.getConfigurator().apply(defaultConfig.Slot0.withKA(newA));
        });
    }

    public static void createMotionMagic(String key, TalonFX motor, TalonFXConfiguration defaultConfig)
    {
        createBasic(key, motor, defaultConfig);

        DogLog.tunable(key + "/CruiseVelocity", defaultConfig.MotionMagic.MotionMagicCruiseVelocity,
                newCruiseVelocity ->
                {
                    motor.getConfigurator()
                            .apply(defaultConfig.MotionMagic.withMotionMagicCruiseVelocity(newCruiseVelocity));
                });

        DogLog.tunable(key + "/Acceleration", defaultConfig.MotionMagic.MotionMagicAcceleration, newAcceleration ->
        {
            motor.getConfigurator().apply(defaultConfig.MotionMagic.withMotionMagicAcceleration(newAcceleration));
        });

        DogLog.tunable(key + "/Jerk", defaultConfig.MotionMagic.MotionMagicJerk, newJerk ->
        {
            motor.getConfigurator().apply(defaultConfig.MotionMagic.withMotionMagicJerk(newJerk));
        });

        DogLog.tunable(key + "/Expo_kV", defaultConfig.MotionMagic.MotionMagicExpo_kV, newExpokV ->
        {
            motor.getConfigurator().apply(defaultConfig.MotionMagic.withMotionMagicExpo_kV(newExpokV));
        });

        DogLog.tunable(key + "/Expo_kA", defaultConfig.MotionMagic.MotionMagicExpo_kA, newExpokA ->
        {
            motor.getConfigurator().apply(defaultConfig.MotionMagic.withMotionMagicExpo_kA(newExpokA));
        });
    }

    public static void createMotionMagic(String key, TalonFXS motor, TalonFXSConfiguration defaultConfig)
    {
        createBasic(key, motor, defaultConfig);

        DogLog.tunable(key + "/CruiseVelocity", defaultConfig.MotionMagic.MotionMagicCruiseVelocity,
                newCruiseVelocity ->
                {
                    motor.getConfigurator()
                            .apply(defaultConfig.MotionMagic.withMotionMagicCruiseVelocity(newCruiseVelocity));
                });

        DogLog.tunable(key + "/Acceleration", defaultConfig.MotionMagic.MotionMagicAcceleration, newAcceleration ->
        {
            motor.getConfigurator().apply(defaultConfig.MotionMagic.withMotionMagicAcceleration(newAcceleration));
        });

        DogLog.tunable(key + "/Jerk", defaultConfig.MotionMagic.MotionMagicJerk, newJerk ->
        {
            motor.getConfigurator().apply(defaultConfig.MotionMagic.withMotionMagicJerk(newJerk));
        });

        DogLog.tunable(key + "/Expo_kV", defaultConfig.MotionMagic.MotionMagicExpo_kV, newExpokV ->
        {
            motor.getConfigurator().apply(defaultConfig.MotionMagic.withMotionMagicExpo_kV(newExpokV));
        });

        DogLog.tunable(key + "/Expo_kA", defaultConfig.MotionMagic.MotionMagicExpo_kA, newExpokA ->
        {
            motor.getConfigurator().apply(defaultConfig.MotionMagic.withMotionMagicExpo_kA(newExpokA));
        });

    }

    private TunablePID()
    {
    }
}
