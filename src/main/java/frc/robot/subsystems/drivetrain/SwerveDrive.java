package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.utils.Utils;

import static frc.robot.Constants.*;

public class SwerveDrive extends LoggedSubsystem {
    private static SwerveDrive INSTANCE = null;

    private final SwerveDriveKinematics mKinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Rear left
            new Translation2d(-DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Rear right
            new Translation2d(-DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));
    private final SwerveDriveOdometry mOdometry = new SwerveDriveOdometry(mKinematics, new Rotation2d(),
            new Pose2d());

    private final AHRS mNavx = new AHRS();

    private final SwerveModule mFrontLeft;
    private final SwerveModule mFrontRight;
    private final SwerveModule mRearLeft;
    private final SwerveModule mRearRight;

    private ChassisSpeeds mChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    
    private final SwerveDriveLogInputs inputs;

    private SwerveDrive() {
        super(SwerveDriveLogInputs.getInstance());
        inputs = SwerveDriveLogInputs.getInstance();

        mFrontLeft = new SwerveModule(
                0,
                FRONT_LEFT_MODULE_DRIVE_MOTOR_ID,
                FRONT_LEFT_MODULE_STEER_MOTOR_ID,
                OFFSETS[0],
                FRONT_LEFT_DRIVE_INVERTED,
                FRONT_LEFT_ANGLE_INVERTED,
                FRONT_LEFT_ANGLE_SENSOR_PHASE,
                FRONT_LEFT_MOTION_MAGIC_CONFIGS);

        mFrontRight = new SwerveModule(
                1,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID,
                FRONT_RIGHT_MODULE_STEER_MOTOR_ID,
                OFFSETS[1],
                FRONT_RIGHT_DRIVE_INVERTED,
                FRONT_RIGHT_ANGLE_INVERTED,
                FRONT_RIGHT_ANGLE_SENSOR_PHASE,
                FRONT_RIGHT_MOTION_MAGIC_CONFIGS);

        mRearLeft = new SwerveModule(
                2,
                REAR_LEFT_MODULE_DRIVE_MOTOR_ID,
                REAR_LEFT_MODULE_STEER_MOTOR_ID,
                OFFSETS[2],
                REAR_LEFT_DRIVE_INVERTED,
                REAR_LEFT_ANGLE_INVERTED,
                REAR_LEFT_ANGLE_SENSOR_PHASE,
                REAR_LEFT_MOTION_MAGIC_CONFIGS);

        mRearRight = new SwerveModule(
                3,
                REAR_RIGHT_MODULE_DRIVE_MOTOR_ID,
                REAR_RIGHT_MODULE_STEER_MOTOR_ID,
                OFFSETS[3],
                REAR_RIGHT_DRIVE_INVERTED,
                REAR_RIGHT_ANGLE_INVERTED,
                REAR_RIGHT_ANGLE_SENSOR_PHASE,
                REAR_RIGHT_MOTION_MAGIC_CONFIGS);
    }

    public static SwerveDrive getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new SwerveDrive();
        }
        return INSTANCE;
    }

    @Override
    public void updateInputs() {
        inputs.speeds = mKinematics.toChassisSpeeds(
                mFrontLeft.getState(),
                mFrontRight.getState(),
                mRearLeft.getState(),
                mRearRight.getState());
        inputs.pose = Utils.pose2dToArray(getPose());
    }

    @Override
    public String getSubsystemName() {
        return "SwerveDrive";
    }

    public void zeroNavx() {
        mNavx.reset();
    }

    public Rotation2d getNavxRotation() {
        return Rotation2d.fromDegrees(mNavx.getYaw());
    }

    public SwerveDriveKinematics getKinematics() {
        return mKinematics;
    }

    public void updateOdometry() {
        mOdometry.update(getNavxRotation(), mKinematics.toSwerveModuleStates(getSpeeds()));
    }

    public Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        mChassisSpeeds = chassisSpeeds;
    }

    public void drive(double vx, double vy, double theta) {
        drive(new ChassisSpeeds(vx, vy, theta));
    }

    public void setStates(SwerveModuleState[] states) {
        mChassisSpeeds = mKinematics.toChassisSpeeds(states);
    }

    public ChassisSpeeds getSpeeds() {
        return inputs.speeds;
    }

    @Override
    public void periodic() {
        SwerveModuleState[] states = mKinematics.toSwerveModuleStates(mChassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        mFrontLeft.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND,
                states[0].angle);
        mFrontRight.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND,
                states[1].angle);
        mRearLeft.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND,
                states[2].angle);
        mRearRight.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND,
                states[3].angle);
    }
}
