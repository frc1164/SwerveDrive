package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotOdometry;

public class Vision extends SubsystemBase {

    NetworkTable LimelightTable;
    String limelightName;
    double tv;
    double ta;
    double tl;

    LimelightHelpers.LimelightResults results;
    RobotOdometry odometer = RobotOdometry.getInstance();

    public Vision(String limelightName) {
        this.limelightName = limelightName;

        LimelightTable = NetworkTableInstance.getDefault().getTable(limelightName);

        LimelightOn(false);
    }

    public boolean LimelightOn(boolean on) {
        if (on) {
            LimelightTable.getEntry("ledMode").setNumber(3);
            return true;
        } else {
            LimelightTable.getEntry("ledMode").setNumber(1);
            return false;
        }
    }

    public void updateValues() {

        //TODO: Maybe switch these to use the limelight helper sub
        tv = LimelightTable.getEntry("tv").getDouble(0);
        ta = LimelightTable.getEntry("ta").getDouble(0);
        tl = LimelightTable.getEntry("tl").getDouble(40);

        results = LimelightHelpers.getLatestResults(limelightName);

    }

    public boolean isGoodTarget() {
        return ta >= 0.5  || results.targetingResults.targets_Fiducials.length > 1 && ta>0.4;
    }

    public boolean hasTargets() {
        return tv == 1;
    }

    public Pose2d getVisionEstimatedPose() {

        double[] bot_pose;

        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            bot_pose = LimelightTable
                    .getEntry("botpose_wpiblue")
                    .getDoubleArray(new double[1]);
        } else {
            bot_pose = LimelightTable
                    .getEntry("botpose_wpired")
                    .getDoubleArray(new double[1]);
        }

        double bot_x = bot_pose[0];
        double bot_y = bot_pose[1];
        double rotation_z = (bot_pose[5] + 360) % 360;


        return new Pose2d(
                new Translation2d(bot_x, bot_y),
                Rotation2d.fromDegrees(rotation_z));
    }

    public double getLatency() {
        return Timer.getFPGATimestamp() - Units.millisecondsToSeconds(tl);

        // maybe need camera_latency?
        // TODO: TEST 
        // return results.targetingResults.latency_capture;

        // TODO: TEST this breaks it for some reason
        // return llresults.targetingResults.latency_pipeline; 
    }

    @Override
    public void periodic() {

        updateValues();

        SmartDashboard.putBoolean("Vision Identified Tag", hasTargets());

        // if the limelight has a target
        if (hasTargets() && isGoodTarget()) {
            // grab data off network tables and clean it up a bit

            Pose2d currentPosition = odometer.getPoseEstimator().getEstimatedPosition();
            Pose2d visionPose = getVisionEstimatedPose();

            // if the data is good, use it to update the pose estimator
            if (visionPose.getX() != 0 && visionPose.getY() != 0 &&

            // these check if vision is within a meter of our estimated pose otherwise we
            // ignore it
            Math.abs(currentPosition.getX() - visionPose.getX()) < 1 &&    
            Math.abs(currentPosition.getY() - visionPose.getY()) < 1) {
      
            // Add the vision measurement to the singleton PoseEstimator
            odometer.getPoseEstimator().addVisionMeasurement(visionPose, getLatency());
                
            }
        }
    }
}
