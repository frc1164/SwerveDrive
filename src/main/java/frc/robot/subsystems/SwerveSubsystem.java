package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
// import com.kauailabs.navx.AHRSProtocol;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final AHRS gyro = new AHRS(SerialPort.Port.kUSB);

    // This is no longer needed since odometry was moved outside this subsystem. Delete if successful
    // private final Pose2d poseThis = new Pose2d();

    //private final SwerveDrivePoseEstimator odometer = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
    //        new Rotation2d(0), Position, poseThis);

    // Get new singleton odometry class for tracking robot pose
    RobotOdometry odometer = RobotOdometry.getInstance();

    //This is now defined and used in the periodic() method and can possibly be deleted
    //private final SwerveModulePosition[] positions = {frontLeft.getPosition(), frontRight.getPosition(), 
    //  backLeft.getPosition(), backRight.getPosition()};

    // Create a new Field2d object for plotting pose
    private final Field2d m_field = new Field2d();

    // Create two new SimpleMotorFeedforwards (one right and one left) with gains kS, kV, and kA from SysID characterization
    private SimpleMotorFeedforward feedforwardRight = new SimpleMotorFeedforward(DriveConstants.kSRight, DriveConstants.kVRight, DriveConstants.kARight);
    private SimpleMotorFeedforward feedforwardLeft = new SimpleMotorFeedforward(DriveConstants.kSLeft, DriveConstants.kVLeft, DriveConstants.kALeft);

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
            //This is new. Added to initialize the odometry. Not sure about the getPose() call... It feels a little recursive...
            resetOdometry(getPose());
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    // Gets the robot heading in degrees
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    // Gets the robot heading as a Rotation2d
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    // Gets the estimated pose from the odometer
    public Pose2d getPose() {
        return odometer.getPoseEstimator().getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        SwerveModulePosition[] state = {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};
        odometer.getPoseEstimator().resetPosition(getRotation2d(), state, pose);
    }

    @Override
    public void periodic() {
        getChassisPitchError();

        SwerveModulePosition[] positions = {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};
        odometer.getPoseEstimator().update(getRotation2d(), positions);

        SmartDashboard.putString("Alliance Color", DriverStation.getAlliance().name());
        m_field.setRobotPose(this.getPose());
        SmartDashboard.putData(m_field);
        
        // SmartDashboard.putNumber("Robot Heading", getHeading());
        // SmartDashboard.putString("Robot Rotation", getPose().getRotation().toString());
        // SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    
        // SmartDashboard.putNumber("Pitch", gyro.getPitch());
        // SmartDashboard.putNumber("Yaw", gyro.getYaw());
        // SmartDashboard.putNumber("Roll", gyro.getRoll());
        // SmartDashboard.putNumber("Pitch Rate", gyro.getRawGyroX());
        // SmartDashboard.putNumber("Yaw Rate", gyro.getRawGyroY());
        // SmartDashboard.putNumber("Roll Rate", gyro.getRawGyroZ());
        // SmartDashboard.putNumber("X Acceleration", gyro.getWorldLinearAccelX());
        // SmartDashboard.putNumber("Y Acceleration", gyro.getWorldLinearAccelY());

    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0], feedforwardLeft);
        frontRight.setDesiredState(desiredStates[1], feedforwardRight);
        backLeft.setDesiredState(desiredStates[2], feedforwardLeft);
        backRight.setDesiredState(desiredStates[3], feedforwardRight);
    }
    
    public float getChassisPitch() {
        return gyro.getPitch();
    }

    public float getChassisPitchError() {
        double pitch = (double)gyro.getPitch();
        SmartDashboard.putNumber("Pitch", pitch);
        pitch = pitch * Math.PI/180;
        double g = 1.0;
        double a = gyro.getWorldLinearAccelX();
        pitch = pitch - Math.asin(((Math.sin(pitch) * 2.0 * g * Math.cos(pitch) + a) / Math.sqrt(Math.pow(g, 2.0) + Math.pow(Math.sin(pitch), 2.0) + 2.0 * g * Math.sin(pitch) + Math.pow(g, 2.0) * Math.pow(Math.cos(pitch), 2.0)))*(180.0/Math.PI));
        SmartDashboard.putNumber("Corrected Pitch", pitch);
        return (float)pitch;
    }
}
