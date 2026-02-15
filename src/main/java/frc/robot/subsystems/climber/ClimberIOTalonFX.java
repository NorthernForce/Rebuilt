package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.lobby.LobbyConstants.ClimberConstants.ClimbLevels;

public class ClimberIOTalonFX implements ClimberIO {
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
    public ClimberIOTalonFX(ClimberParameters params) {
        motorID = params.motorID();
        sensorID = params.bottomLimitSwitchId();
        motor = new TalonFX(motorID);
        limitSwitch = new DigitalInput(sensorID);
        gearRatio = params.gearRatio();
        slowSpeed = params.slowSpeed();
        bottomLevel = params.bottomLevel();
        l1 = params.l1Height();
        l2 = params.l2Height();
        l3 = params.l3Height();
    }
    @Override
    public void runToPosition(ClimbLevels level) {
        double targetRotations = level.getHeight().in(Inches) * gearRatio;
        motor.setControl(positionRequest.withPosition(targetRotations));
        currentLevel = level;
    }

    /**
     * Runs to position with level number inputted
     * 
     * @param levelNum the level to go to, where 0 is the bottom of the elevator and 3 is the top rung.
     */
    @Override
    public void runToPosition(int levelNum) {
        ClimbLevels level = levelNum == 0 ? bottomLevel : (levelNum == 1 ? l1 : (levelNum == 2 ? l2 : l3));
        double targetRotations = level.getHeight().in(Inches) * gearRatio;
        motor.setControl(positionRequest.withPosition(targetRotations));
        currentLevel = level; 
    }

    @Override
    public void homeDown() {
        motor.set(-slowSpeed);
    }

    @Override
    public ClimbLevels getLevel() {
        return currentLevel;
    }

    @Override
    public boolean atBottom() {
        return limitSwitch.get();
    }
    
    @Override
    public void stopMotor() {
        motor.stopMotor();
    }
}
