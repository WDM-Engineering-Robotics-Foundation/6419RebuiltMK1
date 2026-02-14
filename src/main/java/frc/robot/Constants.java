package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public class Constants {

    public static final class ShooterConstants {

        public static final int LEFT_SHOOTER_MOTOR_ID = 15;
        public static final int RIGHT_SHOOTER_MOTOR_ID = 16;

        public static final boolean LEFT_SHOOTER_REVERSED = false;
        public static final boolean RIGHT_SHOOTER_REVERSED = true;

        public static final double BASE_VELOCITY = 0;
        public static final double VELO_FF = 0;

        public static final double VELO_TOLERANCE = 0;

        public static final Slot0Configs SHOOTER_PID = new Slot0Configs().withKP(0);
    }

    public static final class SpindexerConstants {

        public static final int SPINDEXER_MOTOR_ID = 18;

        public static final boolean SPINDEXER_REVERSED = false;

        public static final double SPINDEXER_DUTY_CYCLE = 0;
    }

    public static final class KickerConstants {

        public static final int KICKER_MOTOR_ID = 17;

        public static final boolean KICKER_REVERSED = false;

        public static final double KICKER_DUTY_CYCLE = 0.0;
    }

    public static final class IntakeConstants {
        public static final int INTAKE_POS_MOTOR_ID = 19;
        public static final int INTAKE_SPIN_MOTOR_ID = 20;

        public static final boolean INTAKE_POS_REVERSED = false;
        public static final boolean INTAKE_SPIN_REVERSED = false;

        public static final Angle INTAKE_POS_HOME = Units.Degrees.of(0);
        public static final Angle INTAKE_POS_OUT = Units.Degrees.of(0);

        public static final double INTAKE_DUTY_CYCLE = 0;
    }
}
