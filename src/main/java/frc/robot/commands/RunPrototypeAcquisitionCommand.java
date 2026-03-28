package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PrototypeAcq;

public class RunPrototypeAcquisitionCommand extends Command {
    private final PrototypeAcq prototypeAcq = PrototypeAcq.getInstance();

    public RunPrototypeAcquisitionCommand() {
        addRequirements(prototypeAcq);
    }

    @Override
    public void initialize() {
        prototypeAcq.start();
    }

    @Override
    public boolean isFinished() {
        return false;

    }

    @Override
    public void end(boolean interrupted) {
        prototypeAcq.stop();
    }
}
