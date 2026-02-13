package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;

public class Constants {

    public static final class ShooterConstants {

        public static final int LEFT_SHOOTER_MOTOR_ID = 15;
        public static final int RIGHT_SHOOTER_MOTOR_ID = 16;
        public static final int FEEDER_MOTOR_ID = 17;

        public static final boolean LEFT_SHOOTER_REVERSED = false;
        public static final boolean RIGHT_SHOOTER_REVERSED = true;
        public static final boolean FEEDER_REVERSED = false;

        public static final double BASE_VELOCITY = 0;
        public static final double VELO_FF = 0;
        public static final double FEEDER_DUTY_CYCLE = 0;

        public static final double VELO_TOLERANCE = 0;

        public static final Slot0Configs SHOOTER_PID = new Slot0Configs().withKP(0);
    }

    public static final class SpindexerConstants {

        public static final int SPINDEXER_MOTOR_ID = 18;

        public static final boolean SPINDEXER_REVERSED = false;

        public static final double SPINDEXER_DUTY_CYCLE = 0;
    }
}
