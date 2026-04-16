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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

    public static Command outtakeBalls() {
        return Subsystems.intake().holdOuttake().asProxy();
    }

    public static Command outtakeShake() {
        return Commands.sequence(
            Commands.deadline(
                Commands.waitSeconds(0.25),
                outtakeBalls()
            ),
            Commands.deadline(
                Commands.waitSeconds(0.25), 
                Subsystems.intake().holdOuttakeShake().asProxy()
            )
        ).repeatedly().handleInterrupt(()->CommandScheduler.getInstance().schedule(RobotCommands.holdIntakeIn()));
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
        .withHeadingPID(9,0,0)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withDeadband(5.12 * 0.1);
    public static Command alignToHub(CommandPS5Controller driveController) {
        return Subsystems.drivetrain().applyRequest(()->{
            Translation2d robotLoc = FieldCalculations.getTurretPose();
            Translation2d robotDiff = FieldCalculations.getTargetPose().minus(robotLoc);

            Rotation2d target = robotDiff.getAngle();

            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                target = target.plus(Rotation2d.k180deg);
            }

            double veloX = 0;
            double veloY = 0;
            if (driveController != null) {
                veloX = -driveController.getLeftY() * 5.12;
                veloY = -driveController.getLeftX() * 5.12;
            }
        
            return alignRequest
                    .withVelocityX(veloX) // Drive forward with negative Y (forward)
                    .withVelocityY(veloY) // Drive left with negative X (left)
                    .withTargetDirection(target.minus(new Rotation2d(Subsystems.drivetrain().getChassisSpeeds().vyMetersPerSecond*Constants.AimingConstants.VELO_MULT)));
                    
        });
    }

    // public static Command alignToFuel(CommandPS5Controller driveController) {
    //     return Subsystems.drivetrain().applyRequest(()->{
    //         Rotation2d targetRotation = Subsystems.drivetrain().getRotation3d().toRotation2d();
            
    //         double veloX = 0;
    //         double veloY = 0;
    //         if (driveController != null) {
    //             veloX = -driveController.getLeftY() * 5.12;
    //             veloY = -driveController.getLeftX() * 5.12;
    //         }
        
    //         return alignRequest
    //                 .withVelocityX(veloX) // Drive forward with negative Y (forward)
    //                 .withVelocityY(veloY) // Drive left with negative X (left)
    //                 .withTargetDirection(targetRotation.plus(Subsystems.vision().getTargetBallYaw()));
    //     });
    // }
}
