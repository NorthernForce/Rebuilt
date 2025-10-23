package frc.robot.util;

import static edu.wpi.first.units.Units.Celsius;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import frc.robot.ralph.RalphConstants.MotorConstants.TalonFXConstants;

public class CTREUtil
{
    public static Status getTalonFXStatus(TalonFX talon)
    {
        Status isConnnected = new Status("TalonFX " + talon.getDeviceID() + " Connection Status",
                () -> talon.isConnected(),
                () -> ("TalonFX is " + (talon.isConnected() ? "connected" : "not connected")));
        Status isAlive = new Status("TalonFXAlive " + talon.getDeviceID() + " Status", () -> talon.isAlive(),
                () -> ("TalonFX is " + (talon.isAlive() ? "alive" : "not alive")));
        Status isTempOk = new Status("TalonFX " + talon.getDeviceID() + " Temperature Status",
                () -> (talon.getDeviceTemp().asSupplier().get().in(Celsius) < TalonFXConstants.kMaxTemperature
                        .in(Celsius)),
                () -> ("TalonFX is " + talon.getDeviceTemp().asSupplier().get().in(Celsius) + " °C, "
                        + "TalonFX max temperature is " + TalonFXConstants.kMaxTemperature.in(Celsius) + " °C"));
        return new Status("TalonFX " + talon.getDeviceID() + " Status", isConnnected, isAlive, isTempOk);
    }

    public static Status isTalonFXOk(TalonFXS talon)
    {
        Status isConnnected = new Status("TalonFX " + talon.getDeviceID() + " Connection Status",
                () -> talon.isConnected(),
                () -> ("TalonFX is " + (talon.isConnected() ? "connected" : "not connected")));
        Status isAlive = new Status("TalonFXAlive " + talon.getDeviceID() + " Status", () -> talon.isAlive(),
                () -> ("TalonFX is " + (talon.isAlive() ? "alive" : "not alive")));
        Status isTempOk = new Status("TalonFX " + talon.getDeviceID() + " Temperature Status",
                () -> (talon.getDeviceTemp().asSupplier().get().in(Celsius) < TalonFXConstants.kMaxTemperature
                        .in(Celsius)),
                () -> ("TalonFX is " + talon.getDeviceTemp().asSupplier().get().in(Celsius) + " °C, "
                        + "TalonFX max temperature is " + TalonFXConstants.kMaxTemperature.in(Celsius) + " °C"));
        return new Status("TalonFX " + talon.getDeviceID() + " Status", isConnnected, isAlive, isTempOk);
    }
}
