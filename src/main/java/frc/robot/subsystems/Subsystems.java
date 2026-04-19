package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.vision.OrinSubsystem;

public class Subsystems {

    private static CommandSwerveDrivetrain drivetrain;

    private static ShooterSubsystem shooter;

    private static SpindexerSubsystem spindexer;

    private static KickerSubsystem kicker;

    private static IntakeSubsystem intake;

    private static VisionSubsystem vision;

    private static OrinSubsystem orin;

    public static void init() {
        drivetrain = TunerConstants.createDrivetrain();
        shooter = new ShooterSubsystem();
        spindexer = new SpindexerSubsystem();
        kicker = new KickerSubsystem();
        intake = new IntakeSubsystem();
        vision = new VisionSubsystem();

        orin = new OrinSubsystem();
        
    }

    public static CommandSwerveDrivetrain drivetrain() {
        throwIfNull("Drivetrain", drivetrain);
        return drivetrain;
    }
    
    public static ShooterSubsystem shooter() {
        throwIfNull("Shooter", shooter);
        return shooter;
    }

    public static SpindexerSubsystem spindexer() {
        throwIfNull("Spindexer", spindexer);
        return spindexer;
    }
    public static KickerSubsystem kicker() {
        throwIfNull("Kicker", kicker);
        return kicker;
    }

    public static IntakeSubsystem intake() {
        throwIfNull("Intake", intake);
        return intake;
    }

    public static VisionSubsystem vision() {
        throwIfNull("Vision", vision);
        return vision;
    }

    public static OrinSubsystem orin() {
        throwIfNull("Orin", orin);
        return orin;
    }

    private static void throwIfNull(String name, Object obj) {
        if (obj == null) throw new IllegalStateException("Subsystem: " + name + " accessed before init()");
    }

}
