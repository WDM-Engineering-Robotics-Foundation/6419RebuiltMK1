package frc.robot.subsystems;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import static frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private SparkFlex posMotor, spinMotor;

    IntakeSubsystem() {
        posMotor = new SparkFlex(0, MotorType.kBrushless); {
            SparkFlexConfig config = new SparkFlexConfig();

            config.inverted(IntakeConstants.INTAKE_POS_REVERSED);

            posMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }

        spinMotor = new SparkFlex(0, MotorType.kBrushless); {
            SparkFlexConfig config = new SparkFlexConfig();

            config.inverted(IntakeConstants.INTAKE_SPIN_REVERSED);

            spinMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }
        setDefaultCommand(Commands.run(()->{
            posMotor.getClosedLoopController().setSetpoint(IntakeConstants.INTAKE_POS_HOME.in(Units.Radians), ControlType.kPosition);
            spinMotor.set(0.0);
        }, this));
        
    }

    public boolean atPosition() {
        return posMotor.getClosedLoopController().isAtSetpoint();
    }

    public Command setOut(boolean intake) {
        return Commands.run(()->{
            posMotor.getClosedLoopController().setSetpoint(IntakeConstants.INTAKE_POS_OUT.in(Units.Radians), ControlType.kPosition);
            spinMotor.set((intake && atPosition()) ? IntakeConstants.INTAKE_DUTY_CYCLE : 0.0);
        }, this).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    

   
}
