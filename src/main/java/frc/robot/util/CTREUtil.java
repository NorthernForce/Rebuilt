package frc.robot.util;

import static edu.wpi.first.units.Units.Celsius;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import frc.robot.ralph.RalphConstants.MotorConstants.TalonFXConstants;

public class CTREUtil
{
    /**
     * Gets the status of a TalonFX
     * 
     * @param talon the TalonFX to get the status of
     * @return the status of the TalonFX
     */
    public static Status getTalonFXStatus(TalonFX talon)
    {
        Status isConnected = new Status("TalonFX " + talon.getDeviceID() + " Connection Status",
                () -> talon.isConnected(),
                () -> ("TalonFX is " + (talon.isConnected() ? "connected" : "not connected")));
        Status isAlive = new Status("TalonFXAlive " + talon.getDeviceID() + " Status", () -> talon.isAlive(),
                () -> ("TalonFX is " + (talon.isAlive() ? "alive" : "not alive")));
        Status isTempOk = new Status("TalonFX " + talon.getDeviceID() + " Temperature Status",
                () -> (talon.getDeviceTemp().asSupplier().get().in(Celsius) < TalonFXConstants.kMaxTemperature
                        .in(Celsius)),
                () -> ("TalonFX is " + talon.getDeviceTemp().asSupplier().get().in(Celsius) + " °C, "
                        + "TalonFX max temperature is " + TalonFXConstants.kMaxTemperature.in(Celsius) + " °C"));
        return new Status("TalonFX " + talon.getDeviceID() + " Status", isConnected, isAlive, isTempOk);
    }

    /**
     * Gets the status of a TalonFXS (currently the same as TalonFX, but kept
     * separate for future changes)
     * 
     * @param talon the TalonFXS to get the status of
     * @return the status of the TalonFXS
     */

    public static Status getTalonFXSStatus(TalonFXS talon)
    {
        Status isConnected = new Status("TalonFX " + talon.getDeviceID() + " Connection Status",
                () -> talon.isConnected(),
                () -> ("TalonFX is " + (talon.isConnected() ? "connected" : "not connected")));
        Status isAlive = new Status("TalonFXAlive " + talon.getDeviceID() + " Status", () -> talon.isAlive(),
                () -> ("TalonFX is " + (talon.isAlive() ? "alive" : "not alive")));
        Status isTempOk = new Status("TalonFX " + talon.getDeviceID() + " Temperature Status",
                () -> (talon.getDeviceTemp().asSupplier().get().in(Celsius) < TalonFXConstants.kMaxTemperature
                        .in(Celsius)),
                () -> ("TalonFX is " + talon.getDeviceTemp().asSupplier().get().in(Celsius) + " °C, "
                        + "TalonFX max temperature is " + TalonFXConstants.kMaxTemperature.in(Celsius) + " °C"));
        return new Status("TalonFX " + talon.getDeviceID() + " Status", isConnected, isAlive, isTempOk);
    }

    /**
     * Gets the status of a CANCoder
     * 
     * @param cancoder the CANCoder to get the status of
     * @return the status of the CANCoder
     */
    public static Status getCANcoderStatus(CANcoder cancoder)
    {
        Status isConnected = new Status("CANCoder " + cancoder.getDeviceID() + " Connection Status",
                () -> cancoder.isConnected(),
                () -> ("CANCoder is " + (cancoder.isConnected() ? "connected" : "not connected")));
        return new Status("CANCoder " + cancoder.getDeviceID() + " Status", isConnected);
    }
}
