package frc.robot.subsystems;

import frc.robot.generated.TunerConstants;

public class Subsystems {

    private static CommandSwerveDrivetrain drivetrain;

    public static void init() {
        drivetrain = TunerConstants.createDrivetrain();
    }

    public static CommandSwerveDrivetrain drivetrain() {
        throwIfNull("Drivetrain", drivetrain);
        return drivetrain;
    }

    private static void throwIfNull(String name, Object obj) {
        if (obj == null) throw new IllegalArgumentException("Subsystem: " + name + " accessed before init()");
    }

}
