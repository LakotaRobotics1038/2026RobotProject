package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterHoodConstants;
import frc.robot.subsystems.ShooterHood;

public class RetractHoodCommand extends Command {
    private final ShooterHood shooterHoods = ShooterHood.getInstance();

    public RetractHoodCommand() {
        addRequirements(shooterHoods);
    }

    @Override
    public void initialize() {
        shooterHoods.setAngle(ShooterHoodConstants.SHOOTER_FULL_RETRACTION_ANGLE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
