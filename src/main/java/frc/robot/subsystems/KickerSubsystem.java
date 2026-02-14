package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.KickerConstants;

public class KickerSubsystem extends SubsystemBase {

    private TalonFX kickerMotor;

    KickerSubsystem() {
        kickerMotor = new TalonFX(KickerConstants.KICKER_MOTOR_ID); 

        MotorOutputConfigs outConfig = new MotorOutputConfigs();

        outConfig.Inverted = KickerConstants.KICKER_REVERSED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        kickerMotor.getConfigurator().apply(outConfig);

        setDefaultCommand(Commands.run(()->kickerMotor.setControl(new DutyCycleOut(0.0)), this));
    }

    public Command runKicker() {
        return Commands.run(()->{
            kickerMotor.setControl(new DutyCycleOut(KickerConstants.KICKER_DUTY_CYCLE));
        },this);
    }



}
