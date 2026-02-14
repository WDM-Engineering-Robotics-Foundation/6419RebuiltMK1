package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Subsystems;

public class RobotCommands {
    
    public Command fireShooter() {
        return Commands.parallel(
            Subsystems.shooter().spinShooter(),
            Commands.waitUntil(Subsystems.shooter()::atSpeed).andThen(
                Commands.parallel(
                    Subsystems.kicker().runKicker(),
                    Subsystems.spindexer().spinHopper()
                )
            )
        );
    }

    public Command intakeBalls() {
        return Subsystems.intake().setOut(true);
    }

    public Command holdIntakeOut() {
        return Subsystems.intake().setOut(false);
    }
}
