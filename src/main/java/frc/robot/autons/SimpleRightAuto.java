package frc.robot.autons;

import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commands.*;
import frc.robot.constants.AcquisitionConstants;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.Optional;

public class SimpleRightAuto extends Auton {
    public SimpleRightAuto(Optional<DriverStation.Alliance> alliance) throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                new AcquisitionPivotCommand(AcquisitionConstants.AcquisitionSetpoint.LOWERED),
                followPathCommand(Paths.getRight1Path()),
                new AdjustHoodsCommand().raceWith(
                        new HubAlignCommand(() -> 0, () -> 0, null)
                                .andThen(new ShootCommand().withTimeout(5))));
    }
}
