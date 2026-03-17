package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterHoodsConstants;
import frc.robot.subsystems.ShooterHoods;

public class RetractHoodsCommand extends Command {
    private final ShooterHoods shooterHoods = ShooterHoods.getInstance();

    public RetractHoodsCommand() {
        addRequirements(shooterHoods);
    }

    @Override
    public void initialize() {
        shooterHoods.getNearHood().setAngle(ShooterHoodsConstants.SHOOTER_FULL_RETRACTION_ANGLE);
        shooterHoods.getFarHood().setAngle(ShooterHoodsConstants.SHOOTER_FULL_RETRACTION_ANGLE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
