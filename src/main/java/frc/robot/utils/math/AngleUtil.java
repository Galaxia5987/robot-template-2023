package frc.robot.utils.math;

import edu.wpi.first.math.geometry.Rotation2d;

public class AngleUtil {

    public static class CoordinateSystem {
        public static CoordinateSystem ABSOLUTE = of(0, false);

        public XDirection xDirection;
        public ThetaDirection thetaDirection;

        public CoordinateSystem(XDirection xDirection, ThetaDirection thetaDirection) {
            this.xDirection = xDirection;
            this.thetaDirection = thetaDirection;
        }

        public static CoordinateSystem of(int xDirection, boolean clockwise) {
            return new CoordinateSystem(
                    XDirection.of(xDirection),
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

        public Angle getAbsoluteAngle() {
            double absoluteAngle;
            if (coordinateSystem.thetaDirection.clockwise) {
                absoluteAngle = coordinateSystem.xDirection.zeroVal - value;
            } else {
                absoluteAngle = coordinateSystem.xDirection.zeroVal + value;
            }
            return new Angle(CoordinateSystem.ABSOLUTE, absoluteAngle);
        }

        public double minus(Angle other) {
            return getAbsoluteAngle().value - other.getAbsoluteAngle().value;
        }

        @Override
        public String toString() {
            return "Angle: \n" +
                    "   Coordinate System: " +
                        coordinateSystem.xDirection.name() + ", " +
                        coordinateSystem.thetaDirection.name() + "\n" +
                    "   Value: " + value;
        }
    }

    public enum XDirection {
        RIGHT(0),
        UP(90),
        LEFT(180),
        DOWN(270);

        public final int zeroVal;

        XDirection(int zeroVal) {
            this.zeroVal = zeroVal;
        }

        public static XDirection of(int zeroVal) {
            switch (zeroVal) {
                case 0:
                    return RIGHT;
                case 90:
                    return UP;
                case 180:
                    return LEFT;
            }
            return DOWN;
        }
    }

    public enum ThetaDirection {
        COUNTER_CLOCKWISE(false),
        CLOCKWISE(true);

        public final boolean clockwise;

        ThetaDirection(boolean clockwise) {
            this.clockwise = clockwise;
        }

        public int get() {
            return clockwise ? -1 : 1;
        }

        public static ThetaDirection of(boolean invert) {
            return invert ? CLOCKWISE : COUNTER_CLOCKWISE;
        }
    }
}
