package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AcquisitionConstants.AcquisitionSetpoint;
import frc.robot.subsystems.Acquisition;

public class AcquisitionWiggleCommand extends Command {
    private final Acquisition acquisition = Acquisition.getInstance();
    private final Timer timer = new Timer();

    public AcquisitionWiggleCommand() {
        addRequirements(acquisition);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        if (timer.get() <= 0.5) {
            acquisition.setPivot(AcquisitionSetpoint.LOW_RAISE);
        } else if (timer.get() <= 1.0) {
            acquisition.setPivot(AcquisitionSetpoint.HIGH_RAISE);
        } else {
            timer.reset();
        }
    }

    @Override
    public void end(boolean interrupted) {
        acquisition.setPivot(AcquisitionSetpoint.LOWERED);
        timer.stop();
        timer.reset();
    }
}
