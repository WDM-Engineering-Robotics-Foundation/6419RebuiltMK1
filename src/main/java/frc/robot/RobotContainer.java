// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.Set;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Subsystems;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SwerveRequest.FieldCentricFacingAngle faceHubDrive = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withDeadband(MaxSpeed * 0.1)
        .withHeadingPID(10, 0, 0);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandPS5Controller joystick = new CommandPS5Controller(0);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        Subsystems.init();
        Subsystems.drivetrain().configurePathPlanner();
        SmartDashboard.putData(autoChooser = AutoBuilder.buildAutoChooser());
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        Subsystems.drivetrain().setDefaultCommand(
            // drivetrain will execute this command periodically
            Subsystems.drivetrain().applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            Subsystems.drivetrain().applyRequest(() -> idle).ignoringDisable(true)
        );

        // joystick.cross().whileTrue(Subsystems.drivetrain().applyRequest(() -> brake));
        // joystick.circle().whileTrue(Subsystems.drivetrain().applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.povDown().and(joystick.triangle()).whileTrue(Subsystems.drivetrain().sysIdDynamic(Direction.kForward));
        joystick.povDown().and(joystick.square()).whileTrue(Subsystems.drivetrain().sysIdDynamic(Direction.kReverse));
        joystick.povUp().and(joystick.triangle()).whileTrue(Subsystems.drivetrain().sysIdQuasistatic(Direction.kForward));
        joystick.povUp().and(joystick.square()).whileTrue(Subsystems.drivetrain().sysIdQuasistatic(Direction.kReverse));
        
        // Bind buttons for intake and shooter commands 
        //joystick.R1().whileTrue(Commands.either(RobotCommands.intakeBalls().finallyDo(()->CommandScheduler.getInstance().schedule(RobotCommands.holdIntakeOut())), Commands.none(), Subsystems.intake()::isOut));
        //joystick.R1().whileTrue(RobotCommands.intakeBalls()).whileFalse(RobotCommands.holdIntakeOut());
        // joystick.square().onTrue(Commands.defer(()->{
        //     if (Subsystems.intake().isOut()) {
        //         return Subsystems.intake().setIn();
        //     }
        //     return Subsystems.intake().setOut(true);
        // }, Set.of(Subsystems.intake())));
        joystick.R1().whileTrue(Subsystems.intake().setOut(true));
        joystick.L1().whileTrue(RobotCommands.fireShooter());

        joystick.circle().onTrue(RobotCommands.alignToHub(joystick).onlyWhile(joystick.axisMagnitudeGreaterThan(PS5Controller.Axis.kRightX.value, 0.1).negate()));

        Subsystems.drivetrain().registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
       return autoChooser.getSelected();
    }
}
