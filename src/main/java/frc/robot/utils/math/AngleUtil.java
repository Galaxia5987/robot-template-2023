package frc.robot.utils.math;

import edu.wpi.first.math.geometry.Rotation2d;

public class AngleUtil {

    public static class CoordinateSystem {
        public static CoordinateSystem ABSOLUTE = of(false, false, true);

        public XDirection xDirection;
        public YDirection yDirection;
        public ThetaDirection thetaDirection;

        public CoordinateSystem(XDirection xDirection, YDirection yDirection, ThetaDirection thetaDirection) {
            this.xDirection = xDirection;
            this.yDirection = yDirection;
            this.thetaDirection = thetaDirection;
        }

        public static CoordinateSystem of(boolean invertX, boolean invertY, boolean clockwise) {
            return new CoordinateSystem(
                    XDirection.of(invertX),
                    YDirection.of(invertY),
                    ThetaDirection.of(!clockwise)
            );
        }
    }

    public static class Angle {
        public CoordinateSystem coordinateSystem;
        public double value;

        public Angle(CoordinateSystem coordinateSystem, double value) {
            this.coordinateSystem = coordinateSystem;
            this.value = value;
        }

        public Angle(CoordinateSystem coordinateSystem, Rotation2d value) {
            this.coordinateSystem = coordinateSystem;
            this.value = value.getDegrees();
        }

        public Angle getAbsoluteValue() {
            return changeCoordinateSystem(CoordinateSystem.ABSOLUTE);
        }

        public Angle changeCoordinateSystem(CoordinateSystem newCoordinateSystem) {
            double val = this.value;
            CoordinateSystem coordinateSystem = this.coordinateSystem;
            val *= coordinateSystem.thetaDirection.get() *
                    newCoordinateSystem.thetaDirection.get() *
                    coordinateSystem.yDirection.get() *
                    newCoordinateSystem.yDirection.get();
            if (newCoordinateSystem.xDirection.invert ^ coordinateSystem.xDirection.invert) {
                val = 180 - val;
            }
            return new Angle(newCoordinateSystem, val);
        }

        public double minus(Angle other) {
            return getAbsoluteValue().value - other.getAbsoluteValue().value;
        }
    }

    public enum XDirection {
        RIGHT(false),
        LEFT(true);

        public final boolean invert;

        XDirection(boolean invert) {
            this.invert = invert;
        }

        public int get() {
            return invert ? -1 : 1;
        }

        public static XDirection of(boolean invert) {
            return invert ? LEFT : RIGHT;
        }
    }

    public enum YDirection {
        UP(false),
        DOWN(true);

        public final boolean invert;

        YDirection(boolean invert) {
            this.invert = invert;
        }

        public int get() {
            return invert ? -1 : 1;
        }

        public static YDirection of(boolean invert) {
            return invert ? DOWN : UP;
        }
    }

    public enum ThetaDirection {
        COUNTER_CLOCKWISE(false),
        CLOCKWISE(true);

        public final boolean invert;

        ThetaDirection(boolean invert) {
            this.invert = invert;
        }

        public int get() {
            return invert ? -1 : 1;
        }

        public static ThetaDirection of(boolean invert) {
            return invert ? CLOCKWISE : COUNTER_CLOCKWISE;
        }
    }
}
