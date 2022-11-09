package frc.robot.subsystems.gyroscope;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.LoggedSubsystem;

public class Gyroscope extends LoggedSubsystem {
    private final AHRS navx;

    private final GyroscopeIO io;
    private final DoubleLogEntry angleLog;
    private final DoubleLogEntry rawAngleLog;
    private final DoubleLogEntry zeroAngleLog;

    public Gyroscope() {
        navx = new AHRS(SPI.Port.kMXP);
        io = new GyroscopeIO();
        io.zeroAngle = new Rotation2d();

        angleLog = new DoubleLogEntry(DataLogManager.getLog(), getLogDirectory("angle"));
        rawAngleLog = new DoubleLogEntry(DataLogManager.getLog(), getLogDirectory("rawAngle"));
        zeroAngleLog = new DoubleLogEntry(DataLogManager.getLog(), getLogDirectory("zeroAngle"));
    }

    @Override
    public void log() {
        io.rawAngle = navx.getRotation2d();
        io.angle = getAngle();

        angleLog.append(io.angle.getDegrees());
        rawAngleLog.append(io.rawAngle.getDegrees());
        zeroAngleLog.append(io.zeroAngle.getDegrees());
    }

    @Override
    public String getSubsystemName() {
        return "Gyroscope";
    }

    /**
     * Resets the angle of the navx to the current angle.
     */
    public void resetAngle() {
        resetAngle(new Rotation2d());
    }

    /**
     * Resets the angle of the navx to the current angle.
     *
     * @param angle the angle in -180 to 180 degrees coordinate system.
     */
    public void resetAngle(Rotation2d angle) {
        io.zeroAngle = getRawAngle().minus(angle);
    }

    /**
     * Gets the current angle of the robot in respect to the start angle.
     *
     * @return the current angle of the robot in respect to the start angle.
     */
    public Rotation2d getAngle() {
        return getRawAngle().minus(io.zeroAngle);
    }

    /**
     * Gets the raw angle from the navx.
     *
     * @return the angle of the robot in respect to the angle of the robot initiation time.
     */
    public Rotation2d getRawAngle() {
        return io.rawAngle;
    }

    public static class GyroscopeIO {
        public Rotation2d angle;
        public Rotation2d rawAngle;
        public Rotation2d zeroAngle;
    }
}
