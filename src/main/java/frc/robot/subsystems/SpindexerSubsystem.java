package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SpindexerConstants;

public class SpindexerSubsystem extends SubsystemBase {

    private final TalonFX spindexerMotor;


    SpindexerSubsystem() {

        spindexerMotor = new TalonFX(SpindexerConstants.SPINDEXER_MOTOR_ID); {
            MotorOutputConfigs outConfig = new MotorOutputConfigs();
            outConfig.Inverted = SpindexerConstants.SPINDEXER_REVERSED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
            spindexerMotor.getConfigurator().apply(outConfig);
        }

        setDefaultCommand(Commands.run(() ->{
            spindexerMotor.set(0);
        }, this));
    }
    
    public Command spinHopper() {
        return Commands.run(() -> {
            spindexerMotor.set(SpindexerConstants.SPINDEXER_DUTY_CYCLE);
        }, this);
    }
}
