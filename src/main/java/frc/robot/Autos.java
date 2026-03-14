package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;

public class Autos {

    public static Command depotTest() {
       
        return Commands.sequence(
            followPath("Test-Left-Start"),
            RobotCommands.fireShooter().withTimeout(3),
            Commands.deadline(
                followPath("Test-Left-Depot").andThen(Commands.waitSeconds(2)),
                RobotCommands.intakeBalls()
            ),
            followPath("Test-Left-End"),
            RobotCommands.fireShooter()
        );
    }
    
    private static Command followPath(String pathName) {
        try {
            return AutoBuilder.followPath(PathPlannerPath.fromPathFile("Test-Left-Start"));
        } catch (FileVersionException | IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }
}
