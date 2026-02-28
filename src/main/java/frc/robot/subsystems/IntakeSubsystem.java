package frc.robot.subsystems;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import static frc.robot.Constants.IntakeConstants;

import java.io.OutputStream;

public class IntakeSubsystem extends SubsystemBase {
    private SparkFlex posMotor, spinMotor;

    private SparkClosedLoopController posController;

    private boolean isOut = false;

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable intakeTable = inst.getTable("IntakeState");

    private final DoublePublisher dutyCyclePub = intakeTable.getDoubleTopic("Intake Speed").publish();
    private final DoublePublisher posActual = intakeTable.getDoubleTopic("Intake Pos").publish();
    private final DoublePublisher posTarget = intakeTable.getDoubleTopic("Intake Target Pos").publish();
    private final DoublePublisher intakeRotOut = intakeTable.getDoubleTopic("Intake Rotation Out").publish();
    private final BooleanPublisher outPub = intakeTable.getBooleanTopic("Is Out").publish();
    private final BooleanPublisher atTargetPub = intakeTable.getBooleanTopic("At Target").publish();


    IntakeSubsystem() {
        posMotor = new SparkFlex(IntakeConstants.INTAKE_POS_MOTOR_ID, MotorType.kBrushless); {
            SparkFlexConfig config = new SparkFlexConfig();
        
            config.inverted(IntakeConstants.INTAKE_POS_REVERSED);
            config.encoder.positionConversionFactor(IntakeConstants.POSITION_CONVERSION);
            //config.encoder.inverted(true);
            config.closedLoop.pid(IntakeConstants.POS_KP, IntakeConstants.POS_KI, IntakeConstants.POS_KD);
            config.closedLoop.outputRange(-1, 1);
            config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

            posMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            posController = posMotor.getClosedLoopController();

        }

        spinMotor = new SparkFlex(IntakeConstants.INTAKE_SPIN_MOTOR_ID, MotorType.kBrushless); {
            SparkFlexConfig config = new SparkFlexConfig();

            config.inverted(IntakeConstants.INTAKE_SPIN_REVERSED);

            spinMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }
        setDefaultCommand(Commands.run(()->{
            isOut = false;
            posController.setSetpoint(IntakeConstants.INTAKE_POS_HOME, ControlType.kPosition);
            spinMotor.set(0.0);
        }, this).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        
    }

    @Override
    public void periodic() {
        dutyCyclePub.set(spinMotor.get());
        //136
        atTargetPub.set(posController.isAtSetpoint());
        //posController.setSetpoint(IntakeConstants.INTAKE_POS_OUT, ControlType.kPosition);
        posTarget.set(posController.getSetpoint());
        posActual.set(posMotor.getEncoder().getPosition());
        intakeRotOut.set(posMotor.getAppliedOutput());
        outPub.set(isOut);
    }

    public boolean atPosition() {
        return posController.isAtSetpoint();
    }

    public Command setOut(boolean intake) {
        return Commands.run(()->{
            isOut = true;
            System.out.println("Setting setpoint: " + IntakeConstants.INTAKE_POS_OUT);
            posController.setSetpoint(IntakeConstants.INTAKE_POS_OUT, ControlType.kPosition, ClosedLoopSlot.kSlot0);
            spinMotor.set((intake) ? IntakeConstants.INTAKE_DUTY_CYCLE : 0.0); // && atPosition()
        }, this).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public boolean isOut() {
        return isOut;
    }

    public Command setIn() {
        return getDefaultCommand();
    }



   
}
