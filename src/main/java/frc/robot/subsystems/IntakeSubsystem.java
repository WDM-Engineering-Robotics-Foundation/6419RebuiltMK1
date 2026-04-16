package frc.robot.subsystems;


import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.encoder.SplineEncoder;
import com.revrobotics.encoder.config.DetachedEncoderAccessor;
import com.revrobotics.encoder.config.DetachedEncoderConfig;
import com.revrobotics.jni.DetachedEncoderJNI;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.KickerConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import static frc.robot.Constants.IntakeConstants;

import java.io.OutputStream;

public class IntakeSubsystem extends SubsystemBase {
    private SparkFlex posMotor;

    private TalonFX spinMotor;

    private SparkClosedLoopController posController;

    private SplineEncoder absEncoder;

    private boolean isOut = false;

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable intakeTable = inst.getTable("IntakeState");

    private final DoublePublisher dutyCyclePub = intakeTable.getDoubleTopic("Intake Speed").publish();
    private final DoublePublisher posActual = intakeTable.getDoubleTopic("Intake Pos").publish();
    private final DoublePublisher posTarget = intakeTable.getDoubleTopic("Intake Target Pos").publish();
    private final DoublePublisher intakeRotOut = intakeTable.getDoubleTopic("Intake Rotation Out").publish();
    private final BooleanPublisher outPub = intakeTable.getBooleanTopic("Is Out").publish();
    private final BooleanPublisher atTargetPub = intakeTable.getBooleanTopic("At Target").publish();

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Units.Volts.of(4),
            Units.Seconds.of(5),
            state -> SignalLogger.writeString("SysIdIntakeState", state.toString())
        ), 
        new SysIdRoutine.Mechanism(
            (volts)->{
                spinMotor.setControl(new VoltageOut(volts));
            }, 
            null, 
            this
        )
    );


    IntakeSubsystem() {
       
        absEncoder = new SplineEncoder(IntakeConstants.INTAKE_ABS_ENCODER_ID); 

        posMotor = new SparkFlex(IntakeConstants.INTAKE_POS_MOTOR_ID, MotorType.kBrushless); {
            SparkFlexConfig config = new SparkFlexConfig();
        
            config.inverted(IntakeConstants.INTAKE_POS_REVERSED);
            config.idleMode(IdleMode.kBrake);
        
            config.closedLoop.pid(IntakeConstants.POS_KP, IntakeConstants.POS_KI, IntakeConstants.POS_KD);
            config.closedLoop.outputRange(-1, 1);
            config.closedLoop.positionWrappingInputRange(0, 1.0);
            config.closedLoop.positionWrappingEnabled(true);
            config.closedLoop.feedbackSensor(FeedbackSensor.kDetachedAbsoluteEncoder, absEncoder);
            //config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
            config.encoder.positionConversionFactor(IntakeConstants.POSITION_CONVERSION);
            

            posMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            posController = posMotor.getClosedLoopController();

        }

        spinMotor = new TalonFX(IntakeConstants.INTAKE_SPIN_MOTOR_ID); {
            MotorOutputConfigs outConfig = new MotorOutputConfigs();

            outConfig.Inverted = IntakeConstants.INTAKE_SPIN_REVERSED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

            spinMotor.getConfigurator().apply(outConfig);
            spinMotor.getConfigurator().apply(IntakeConstants.INTAKE_PID);
            
        }
        setDefaultCommand(Commands.run(()->{
            isOut = false;
            posController.setSetpoint(IntakeConstants.INTAKE_POS_OUT, ControlType.kPosition); //HOME
            spinMotor.set(0.0);
        }, this).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        
    }

    @Override
    public void periodic() {
        dutyCyclePub.set(spinMotor.get());
        //136
        atTargetPub.set(atPosition());
        //posController.setSetpoint(IntakeConstants.INTAKE_POS_OUT, ControlType.kPosition);
        posTarget.set(posController.getSetpoint());
        
        posActual.set(absEncoder.getAngle());
        //posActual.set(posMotor.getEncoder().getPosition());
        intakeRotOut.set(posMotor.getAppliedOutput());
        outPub.set(isOut);
    }

    public Command sysIdDynamic(Direction dir) {
        return sysIdRoutine.dynamic(dir);
    }

    public Command sysIdQuasistatic(Direction dir) {
        return sysIdRoutine.quasistatic(dir);
    }

    public boolean atPosition() {
        return Math.abs(posController.getSetpoint()-absEncoder.getAngle()) <= IntakeConstants.POS_TOLERANCE;
    }

    public Command setOut(boolean intake) {
        return Commands.run(()->{
            isOut = true;
            posController.setSetpoint(IntakeConstants.INTAKE_POS_OUT, ControlType.kPosition, ClosedLoopSlot.kSlot0);
            spinMotor.setControl(new DutyCycleOut((intake) ? IntakeConstants.INTAKE_DUTY_CYCLE : 0.0)); // && atPosition()
        }, this).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public Command holdOuttake() {
        return Commands.run(()->{
            isOut = true;
            posController.setSetpoint(IntakeConstants.INTAKE_POS_OUT, ControlType.kPosition, ClosedLoopSlot.kSlot0);
            spinMotor.setControl(new DutyCycleOut(IntakeConstants.OUTTAKE_DUTY_CYCLE)); // && atPosition()
        }, this).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public Command holdOuttakeShake() {
        return Commands.run(()->{
            isOut = true;
            posController.setSetpoint(IntakeConstants.INTAKE_POS_OUT_SHAKE, ControlType.kPosition, ClosedLoopSlot.kSlot0);
            spinMotor.setControl(new DutyCycleOut(IntakeConstants.OUTTAKE_DUTY_CYCLE)); // && atPosition()
        }, this).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public Command setFeed() {
        return Commands.run(()->{
            posController.setSetpoint(IntakeConstants.INTAKE_POS_FEED, ControlType.kPosition);
            spinMotor.set(0);
        }, this).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public Command setDepot() {
        return Commands.run(()->{
            posController.setSetpoint(IntakeConstants.INTAKE_POS_DEPOT, ControlType.kPosition);
            spinMotor.set(IntakeConstants.INTAKE_DUTY_CYCLE);
        }, this).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public boolean isOut() {
        return isOut;
    }

    public Command setIn() {
        return Commands.run(()->{
            isOut = false;
            posController.setSetpoint(IntakeConstants.INTAKE_POS_HOME, ControlType.kPosition);
            spinMotor.set(0.0);
        }, this).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public Command setZero() {
        return Commands.run(()->{
            posController.setSetpoint(IntakeConstants.INTAKE_POS_INSIDE, ControlType.kPosition);
            spinMotor.set(0.0);
        }, this);
    }



   
}
