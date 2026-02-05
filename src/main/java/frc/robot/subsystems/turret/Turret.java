package frc.robot.subsystems.turret;

public class Turret extends SubsystemBase {
    private final TurretConstants constants;
    private final SuzieIO suzie;
    private final HoodIO hood;
    private final ShooterIO shooter;

    public Turret(SuzieIO suzie, HoodIO hood, ShooterIO shooter) {
        this.constants = constants;
        this.suzie = suzie;
        this.hood = hood;
        this.shooter = shooter;
    }
}