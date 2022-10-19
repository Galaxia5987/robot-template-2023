package frc.robot;


public final class Constants {
    public static final int TALON_TIMEOUT = 10;
    public static final double NOMINAL_VOLTAGE = 10.0;
    public static final double LOOP_PERIOD = 0.02; // [s]

    public static class Intake {
        public static final double DEFAULT_POWER = 0.6;
    }

    public static class Shooter {
        public static final double Kp = 1;
        public static final double Ki = 1;
        public static final double Kd = 1;
        public static final double Kf = 1;

        public static final double TICKS_PER_ROTATION = 2048;
    }
}
