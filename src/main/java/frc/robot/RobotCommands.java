package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.lang.reflect.Field;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.Subsystems;

public class RobotCommands {
    
    public static Command fireShooter() {
        return Commands.deadline(
            Commands.waitUntil(Subsystems.shooter()::atSpeed).andThen(
                Commands.parallel(
                    Subsystems.kicker().runKicker(),
                    Subsystems.spindexer().spinHopper()
                )
            ),
            Subsystems.shooter().spinShooter()
        ).withInterruptBehavior(InterruptionBehavior.kCancelSelf).asProxy();
    }

    public static Command fireShooterSetSpeed(double speed) {
        return Commands.deadline(
            Commands.waitUntil(Subsystems.shooter()::atSpeed).andThen(
                Commands.parallel(
                    Subsystems.kicker().runKicker(),
                    Subsystems.spindexer().spinHopper()
                )
            ),
            Subsystems.shooter().spinShooterSetSpeed(speed)
        ).withInterruptBehavior(InterruptionBehavior.kCancelSelf).asProxy();
    }


    public static Command intakeBalls() {
        return Subsystems.intake().setOut(true).asProxy();
    }

    public static Command holdIntakeIn() {
        return Subsystems.intake().setIn().asProxy();
    }

    public static Command holdIntakeOut() {
        return Subsystems.intake().setOut(false).asProxy();
    }

    public static Command holdIntakeFeed() {
        return Subsystems.intake().setFeed().asProxy();
    }
    private static final SwerveRequest.FieldCentricFacingAngle alignRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withHeadingPID(8,0,0)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withDeadband(5.12 * 0.1);
    public static Command alignToHub(CommandPS5Controller driveController) {
        return Subsystems.drivetrain().applyRequest(()->{
            Translation2d robotLoc = Subsystems.drivetrain().getState().Pose.getTranslation();
            Translation2d robotDiff = FieldCalculations.getTargetPose().minus(robotLoc);

            Rotation2d target = robotDiff.getAngle();

            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                target = target.plus(Rotation2d.k180deg);
                target = FieldCalculations.getAngleOffsetDegrees();
            }

            target = target.plus(FieldCalculations.getAngleOffsetDegrees().times(Constants.ShooterConstants.OFFSET_MULT_SIDE));
        
            return alignRequest
                    .withVelocityX(-driveController.getLeftY() * 5.12) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveController.getLeftX() * 5.12) // Drive left with negative X (left)
                    .withTargetDirection(target);
                    
        });
    }
}
