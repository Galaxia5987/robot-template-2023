package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

public final class Ports {

    public static class Intake {
        public static final int MOTOR = 0;
        public static final int SOLENOID = 0;
        public static final TalonFXInvertType INVERSION = TalonFXInvertType.Clockwise;
    }
}
