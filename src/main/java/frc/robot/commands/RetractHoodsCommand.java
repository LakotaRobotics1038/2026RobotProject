package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class RetractHoodsCommand extends Command {
    private final Shooter shooter = Shooter.getInstance();

    public RetractHoodsCommand() {
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.getNearShooter().setAngle(ShooterConstants.SHOOTER_FULL_RETRACTION_ANGLE);
        shooter.getFarShooter().setAngle(ShooterConstants.SHOOTER_FULL_RETRACTION_ANGLE);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
