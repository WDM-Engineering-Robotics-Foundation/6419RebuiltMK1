package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldCalculations;

import static frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private TalonFX shooterLeft, shooterRight;

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private final NetworkTable shooterTable = inst.getTable("ShooterState");

    private final DoublePublisher shooterVelo = shooterTable.getDoubleTopic("Velocity").publish();

    private final DoublePublisher shooterError = shooterTable.getDoubleTopic("Velo Error").publish();

    private final BooleanPublisher shooterAtSpeed = shooterTable.getBooleanTopic("At Speed").publish();

    private double targetVelo = 0.0;

    ShooterSubsystem() {
        shooterLeft = new TalonFX(ShooterConstants.LEFT_SHOOTER_MOTOR_ID); {
            MotorOutputConfigs outConfig = new MotorOutputConfigs();

            outConfig.Inverted = ShooterConstants.LEFT_SHOOTER_REVERSED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
            ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
            closedLoopConfig.allowedClosedLoopError(ShooterConstants.VELO_TOLERANCE, ClosedLoopSlot.kSlot0);
    

            shooterLeft.getConfigurator().apply(outConfig);
            shooterLeft.getConfigurator().apply(ShooterConstants.SHOOTER_PID);
        }

        shooterRight = new TalonFX(ShooterConstants.RIGHT_SHOOTER_MOTOR_ID); {
            MotorOutputConfigs outConfig = new MotorOutputConfigs();

            outConfig.Inverted = ShooterConstants.RIGHT_SHOOTER_REVERSED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

            shooterRight.getConfigurator().apply(outConfig);
            shooterRight.getConfigurator().apply(ShooterConstants.SHOOTER_PID);
        }

        setDefaultCommand(Commands.run(() -> {
            targetVelo = 0;
            shooterLeft.setControl(new CoastOut());
            shooterRight.setControl(new Follower(ShooterConstants.LEFT_SHOOTER_MOTOR_ID, MotorAlignmentValue.Opposed));
        },this));
    }

    public boolean atSpeed() {
        return targetVelo != 0 && (Math.abs(shooterLeft.getVelocity().getValue().in(Units.RotationsPerSecond)-targetVelo) < ShooterConstants.VELO_TOLERANCE);
    }

    public Command spinShooter() {
        return Commands.run(()->{
            Translation2d robotLoc = Subsystems.drivetrain().getState().Pose.getTranslation();
            // double velo = (FieldCalculations.getTargetPose().getDistance(robotLoc)-ShooterConstants.BASE_DIST_METERS)*ShooterConstants.VELO_DIST_MULT + ShooterConstants.BASE_VELOCITY;
            // velo += Subsystems.drivetrain().getChassisSpeeds().vxMetersPerSecond*Constants.AimingConstants.VELO_MULT_DIST;
            // velo = MathUtil.clamp(velo, 0, ShooterConstants.VELO_MAX);
            double velo = ShooterConstants.BASE_VELOCITY;
            shooterLeft.setControl(new VelocityVoltage(targetVelo = velo).withFeedForward(ShooterConstants.VELO_FF));
            shooterRight.setControl(new Follower(ShooterConstants.LEFT_SHOOTER_MOTOR_ID, MotorAlignmentValue.Opposed));
        }, this);
    }

    @Override
    public void periodic() {
        shooterVelo.set(shooterLeft.getVelocity(true).getValue().in(Units.RotationsPerSecond));
        shooterError.set(shooterLeft.getClosedLoopError(true).getValue());
        shooterAtSpeed.set(atSpeed());
    }
   
    

    
}
