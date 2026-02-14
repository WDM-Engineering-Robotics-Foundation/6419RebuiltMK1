package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private TalonFX shooterLeft, shooterRight;

    ShooterSubsystem() {
        shooterLeft = new TalonFX(ShooterConstants.LEFT_SHOOTER_MOTOR_ID); {
            MotorOutputConfigs outConfig = new MotorOutputConfigs();

            outConfig.Inverted = ShooterConstants.LEFT_SHOOTER_REVERSED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
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
            shooterLeft.setControl(new CoastOut());
            shooterRight.setControl(new Follower(ShooterConstants.LEFT_SHOOTER_MOTOR_ID, MotorAlignmentValue.Opposed));
        },this));
    }

    public boolean atSpeed() {
        return shooterLeft.getClosedLoopError(true).getValueAsDouble() < ShooterConstants.VELO_TOLERANCE;
    }

    public Command spinShooter() {
        return Commands.run(()->{
            shooterLeft.setControl(new VelocityVoltage(ShooterConstants.BASE_VELOCITY));
            shooterRight.setControl(new Follower(ShooterConstants.LEFT_SHOOTER_MOTOR_ID, MotorAlignmentValue.Opposed));
        }, this);
    }
   
    

    
}
