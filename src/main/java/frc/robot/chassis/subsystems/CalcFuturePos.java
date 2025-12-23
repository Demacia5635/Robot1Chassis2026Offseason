// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.Camera;

public class CalcFuturePos extends SubsystemBase {
    private final Supplier<ChassisSpeeds> speedsSupplier;
    private final Supplier<Rotation2d> robotAngleSupplier;
    private final Supplier<Pose2d> currentPoseSupplier;
    private final double dtSeconds;
    private final NetworkTable table;

    /**
     * Constructor for position tracking with vision latency compensation.
     * @param camera The camera used for vision measurements
     * @param speeds Supplier that provides current chassis speeds
     * @param getRobotAngle Supplier that provides current robot angle
     * @param currentPose2d Supplier that provides current robot pose
     * @param dtseconds The time step in seconds for periodic updates
     */
    public CalcFuturePos(Camera camera, Supplier<ChassisSpeeds> speeds, 
                                     Supplier<Rotation2d> getRobotAngle, 
                                     Supplier<Pose2d> currentPose2d, 
                                     double dtseconds) {
        this.speedsSupplier = speeds;
        this.robotAngleSupplier = getRobotAngle;
        this.currentPoseSupplier = currentPose2d;
        this.dtSeconds = dtseconds;
        this.table = NetworkTableInstance.getDefault().getTable(camera.getTableName());
    }

    /**
     * Computes the change in position accounting for vision latency.
     * The latency is the sum of target latency (tl) and capture latency (cl).
     * This method extrapolates where the robot was when the vision data was captured.
     * 
     * @return Translation2d representing the change in position in field coordinates
     */
    public Translation2d computeDeltaPosition() {

        // Get latency in milliseconds and convert to seconds
        double latencyMs = table.getEntry("tl").getDouble(0.0) + 
                          table.getEntry("cl").getDouble(0.0);
        double latencySeconds = latencyMs / 1000.0;

        // Calculate total time including periodic update time and vision latency
        double totalTime = dtSeconds + latencySeconds;

        // Get current chassis speeds
        ChassisSpeeds currentSpeeds = speedsSupplier.get();

        Pose2d currentPose = currentPoseSupplier.get();

        // Calculate movement in robot-relative coordinates
        Translation2d relativeMovement = new Translation2d(
            currentPose.getX() + currentSpeeds.vxMetersPerSecond * totalTime,
            currentPose.getY() + currentSpeeds.vyMetersPerSecond * totalTime
        );

        Rotation2d relativeAngle = robotAngleSupplier.get().rotateBy(new Rotation2d(currentSpeeds.omegaRadiansPerSecond * totalTime));
        
        // Rotate to field-relative coordinates
        Translation2d futurePose = relativeMovement.rotateBy(relativeAngle);
        return futurePose;
    }

    /**
     * Gets the estimated pose of the robot at the time the vision measurement was taken.
     * This accounts for the latency by subtracting the movement that occurred during the delay.
     * 
     * @return Pose2d representing where the robot was when vision data was captured
     */
    public Pose2d getLatencyCompensatedPose() {
        Pose2d currentPose = currentPoseSupplier.get();

        // Subtract the delta to get where we were when the measurement was taken
        Translation2d delta = computeDeltaPosition();
        return new Pose2d(
            currentPose.getX() - delta.getX(),
            currentPose.getY() - delta.getY(),
            currentPose.getRotation()
        );
    }


  @Override
  public void periodic() {
    computeDeltaPosition();
  }
}
