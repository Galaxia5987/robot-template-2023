package frc.robot.utils.math;

/**
 * A Double that can be interpolated using the InterpolatingTreeMap.
 *
 * @see InterpolatingTreeMap
 */
public class InterpolatingDouble implements Interpolable<InterpolatingDouble>, InverseInterpolable<InterpolatingDouble>,
        Comparable<InterpolatingDouble> {
    public double value;

    public InterpolatingDouble(double val) {
        value = val;
    }

    @Override
    public InterpolatingDouble interpolate(InterpolatingDouble other, double x) {
        double dydx = other.value - this.value;
        double searchY = dydx * x + value;
        return new InterpolatingDouble(searchY);
    }

    @Override
    public double inverseInterpolate(InterpolatingDouble upper, InterpolatingDouble query) {
        double upper_to_lower = upper.value - value;
        if (upper_to_lower <= 0) {
            return 0;
        }
        double query_to_lower = query.value - value;
        if (query_to_lower <= 0) {
            return 0;
        }
        return query_to_lower / upper_to_lower;
    }

    @Override
    public int compareTo(InterpolatingDouble other) {
        return Double.compare(value, other.value);
    }
}
