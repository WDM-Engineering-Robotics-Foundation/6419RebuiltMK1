package frc.robot.subsystems;

import frc.robot.generated.TunerConstants;

public class Subsystems {

    private static CommandSwerveDrivetrain drivetrain;

    private static ShooterSubsystem shooter;

    private static SpindexerSubsystem spindexer;

    private static KickerSubsystem kicker;

    private static IntakeSubsystem intake;

    private static VisionSubsystem vision;

    public static void init() {
        drivetrain = TunerConstants.createDrivetrain();
        shooter = new ShooterSubsystem();
        spindexer = new SpindexerSubsystem();
        kicker = new KickerSubsystem();
        intake = new IntakeSubsystem();
        vision = new VisionSubsystem();
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

    private static void throwIfNull(String name, Object obj) {
        if (obj == null) throw new IllegalStateException("Subsystem: " + name + " accessed before init()");
    }

}
