package frc.robot;

import org.opencv.core.Mat.Atable;

import edu.wpi.first.hal.simulation.SpiReadAutoReceiveBufferCallback;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.HashMap;


public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1/6.75;
        public static final double kTurningMotorGearRatio = 7.00 / 150.00;//150.00 / 7.00;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio *  kWheelDiameterMeters * Math.PI;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2.0 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;
        public static final double kPTurning = 0.5;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(24.75);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(24.75);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), /* Left front */
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), /* Right front */
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), /* Left rear */
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); /* Right rear */

        public static final int kFrontLeftDriveMotorPort = 10;
        public static final int kBackLeftDriveMotorPort = 40;
        public static final int kFrontRightDriveMotorPort = 20;
        public static final int kBackRightDriveMotorPort = 30;

        public static final int kFrontLeftTurningMotorPort = 11;
        public static final int kBackLeftTurningMotorPort = 41;
        public static final int kFrontRightTurningMotorPort = 21;
        public static final int kBackRightTurningMotorPort = 31;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 12;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 42;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 22;
        public static final int kBackRightDriveAbsoluteEncoderPort = 32;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 167.695 * Math.PI/180.0;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 45.0 * Math.PI/180.0;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 284.15 * Math.PI/180.0;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 245.25 * Math.PI/180.0;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        /* Feedforward constants from SysID run on 2/9/23 */
        public static final double kSLeft = 0.097576;
        public static final double kVLeft = 2.6933;
        public static final double kALeft = 0.26236;

        public static final double kSRight = 0.099437;
        public static final double kVRight = 2.6173;
        public static final double kARight = 0.11195;

        // Drive/Rotation gain
        public static final double kRotGain = 2;
        public static final double kDriveGain = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 0.1;
        public static final double kPYController = 0.1;
        public static final double kPThetaController = 10;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 5;
        public static final int kDriverFieldOrientedButtonIdx = 5;

        public static final double kDeadband = 0.1;
    }

    public static class GripperC {
        public static final int rightMotor = 62;
        public static final int leftMotor = 61;
        public static final int GripperMotor = 60;
        public static final int GripperCANifier = 59;
        public static boolean leftMotorReversed = false;
        public static boolean rightMotorReversed = true;
        public static final int TimeOfFlightSensor = 63;
    }
     
    public static class OperatorConstants {
     public static final int kOperatorControllerPort = 1;
    }

    public static final class ArmConstants {
        //CAN Values for Arm Hardware
        public static final int armCANifier = 53;
        public static final int armShoulderMotor = 50;
        public static final int armExtensionMotor = 51;
        public static final int CANCoderid = 52;

        //Shoulder Soft Limit Stuff
        public static final double ShoulderEncoderOffset = 263 * Math.PI/180;
        //public static final double TopShoulderLimit = -0.05;
        public static final double TopShoulderLimit = 0.22;
        public static final double TopShoulderSoftStop = 0.1;
        public static final double BottomShoulderLimit = -1.12;
        public static final double BottomShoulderSoftStop = -1.17;
        public static final boolean ShoulderEncoderRevsersed = false;

        //Telescope Limits
        public static final double TopTelescopeLimit = 125 * 360 * Math.PI/180;
        public static final double BottomTelescopeLimit = -0.95 /* 10 * 360 * Math.PI/180 */ ;
    
        //Telescope Lenghts
        public static final double armR0 = 46;
        public static final double armRetractedSoftStop = 46.5;
        public static final double maxArmLength = 72;
        public static final double armExtendedSoftStop = 71.5;
        public static final double extensionPerRotation = 0.206;

        // Arm control PID. Yay!
        public static final double thetaP = 0.1;
        public static final double thetaI = 0.05;
        public static final double thetaD = 0;
        public static final double radiusP = 0.01;
        public static final double radiusI = 0.05;
        public static final double radiusD = 0;

        // Arm Soft Limit Locations
        public static final double radiusOutputMax = 20;
        public static final double thetaOutputMax = 20;

        public static final double thetaBumper = -0.88;
        public static final double yFloor = -41;
        public static final double yBumper = -36.0;
 
    }

    public static final class BalanceConstants {
        public static final double balanceP = 0.0275;
        public static final double balanceI = 0;
        public static final double balanceD = .01;
    }
}