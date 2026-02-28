package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class Constants {

    public static final class ShooterConstants {

        public static final int LEFT_SHOOTER_MOTOR_ID = 15;
        public static final int RIGHT_SHOOTER_MOTOR_ID = 16;

        public static final boolean LEFT_SHOOTER_REVERSED = false;
        public static final boolean RIGHT_SHOOTER_REVERSED = true;

        public static final double BASE_VELOCITY = 40;
        public static final double VELO_FF = 4.6;

        public static final double VELO_TOLERANCE = 4;

        public static final double VELO_MAX = 75;

        public static final double VELO_DIST_MULT = 10;

        public static final double BASE_DIST_METERS = 2.54;

        public static final Slot0Configs SHOOTER_PID = new Slot0Configs().withKP(0.75);
    }

    public static final class SpindexerConstants {

        public static final int SPINDEXER_MOTOR_ID = 18;

        public static final boolean SPINDEXER_REVERSED = false;

        public static final double SPINDEXER_DUTY_CYCLE = 1;
    }

    public static final class KickerConstants {

        public static final int KICKER_MOTOR_ID = 17;

        public static final boolean KICKER_REVERSED = true;

        public static final double KICKER_DUTY_CYCLE = 1;
    }

    public static final class IntakeConstants {
        public static final int INTAKE_POS_MOTOR_ID = 19;
        public static final int INTAKE_SPIN_MOTOR_ID = 20;

        public static final boolean INTAKE_POS_REVERSED = false;
        public static final boolean INTAKE_SPIN_REVERSED = false;

        public static final double INTAKE_POS_HOME = 0.05;
        // 136
        public static final double INTAKE_POS_OUT = 0.35;

        public static final double POSITION_CONVERSION = 1.0/20.0 * 24.0/56.0;

        public static final double INTAKE_DUTY_CYCLE = 0.5;

        public static final double POS_KP = 1.4;
        public static final double POS_KI = 0.0;
        public static final double POS_KD = 0.0;
    }

    public static final class VisionConstants {
        public static final String FRONT_ELEMENT_CAMERA_NAME = "Front Element Camera";

        public static final String FRONT_LOCALIZATION_CAMERA_NAME = "Front Local Camera";
        public static final Transform3d FRONT_LOCALIZATION_CAMERA_OFFSET = new Transform3d(
            Units.Inches.of(9.5),
            Units.Inches.of(-3.4),
            Units.Inches.of(27.1),
            new Rotation3d(
                Units.Degrees.of(0),
                Units.Degrees.of(-10),
                Units.Degrees.of(0)
            )
        );
    }

    public static final class AimingConstants {
        public static final Translation2d RED_BASE_HUB_LOC = new Translation2d(
            Units.Meters.of(11.91),
            Units.Meters.of(4.025)
        );

        public static final Translation2d BLUE_BASE_HUB_LOC = new Translation2d(
            Units.Meters.of(4.625),
            Units.Meters.of(4.025)
        );

        public static final double VELO_MULT = 0.1;

        public static final double VELO_MULT_DIST = 0.1;

        public static final double SHOOTER_ANGLE_RAD = Units.Degrees.of(54).in(Units.Radians);

        public static final Distance SHOOTER_HEIGHT = Units.Inches.of(24.72);

        public static final Distance HUB_HEIGHT = Units.Inches.of(72);

        public static final double ANGLE_COS_CONST = 2 * (HUB_HEIGHT.minus(SHOOTER_HEIGHT).in(Units.Meters)) * Math.pow(Math.cos(SHOOTER_ANGLE.in(Units.Radians)),2);

        

        
    }
}
