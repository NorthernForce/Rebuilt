package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Distance;
import frc.robot.lobby.LobbyConstants.ClimberConstants.ClimbLevels;

public record ClimberParameters(int motorID, int bottomLimitSwitchId, ClimbLevels bottomLevel, ClimbLevels l1Height,
        ClimbLevels l2Height, ClimbLevels l3Height, double gearRatio, double slowSpeed, Distance maxHeight, double kP,
        double kI, double kD, double kV, double kG, double topRotations) {

}
