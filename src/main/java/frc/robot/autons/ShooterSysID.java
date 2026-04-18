package frc.robot.autons;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Shooter;

public class ShooterSysID extends Auton {
    private final Shooter shooter = Shooter.getInstance();

    public ShooterSysID(Optional<Alliance> alliance) throws FileVersionException, IOException, ParseException {
        super(alliance);
        super.addCommands(
                Commands.runOnce(DataLogManager::start)
                        .andThen(shooter.quasistaticSysId(Direction.kForward))
                        .andThen(shooter.quasistaticSysId(Direction.kReverse))
                        .andThen(shooter.dynamicSysId(Direction.kForward))
                        .andThen(shooter.dynamicSysId(Direction.kReverse))
                        .andThen(Commands.runOnce(DataLogManager::stop)));
    }
}
