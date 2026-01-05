package frc.robot.vision.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import static frc.robot.vision.utils.VisionConstants.*;

public class Quest extends SubsystemBase {
  private Field2d field;
  private QuestNav questNav;
  private Pose3d currentQuestPose;
  private boolean isCalibrated;
  private double timestamp;

  public Quest() {
    isCalibrated = false;
    timestamp = 0;
    questNav = new QuestNav();
    field = new Field2d();
    currentQuestPose = new Pose3d(); // Initialize to origin - IMPORTANT!
    
    SmartDashboard.putData("Quest Field", field);
  }
  public boolean isCalibrated(){
    return isCalibrated;
  }
  
  // Set robot pose (transforms to Quest frame and sends to QuestNav)
  public void setQuestPose(Pose3d currentBotpose){
    currentQuestPose = currentBotpose.transformBy(ROBOT_TO_QUEST);
    questNav.setPose(currentQuestPose);
    isCalibrated = true;// i know it is inefficent
  }

  // Get robot pose (transforms from Quest frame to robot frame)
  public Pose2d getRobotPose() { 
    return currentQuestPose.transformBy(ROBOT_TO_QUEST.inverse()).toPose2d();
  }
  
  // Check if Quest is connected
  public boolean isConnected() {
    return questNav.isConnected();
  }
  
  // Check if Quest is tracking
  public boolean isTracking() {
    return questNav.isTracking();
  }
  

  @Override
  public void periodic() {
    questNav.commandPeriodic();

    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
    
    if(poseFrames.length > 0 && poseFrames[poseFrames.length - 1].isTracking()){
      currentQuestPose = poseFrames[poseFrames.length - 1].questPose3d();
      timestamp = poseFrames[poseFrames.length - 1].dataTimestamp();
      // Display Quest pose
      SmartDashboard.putNumber("Quest X", currentQuestPose.getX());
      SmartDashboard.putNumber("Quest Y", currentQuestPose.getY());
      SmartDashboard.putNumber("Quest Rotation", currentQuestPose.getRotation().getZ());

      field.setRobotPose(currentQuestPose.toPose2d());
    }
    
    // Diagnostics - helpful for debugging!
    SmartDashboard.putBoolean("Quest Connected", questNav.isConnected());
    SmartDashboard.putBoolean("Quest Tracking", questNav.isTracking());
    SmartDashboard.putNumber("Quest Latency (ms)", questNav.getLatency());
    
    // Battery monitoring
    questNav.getBatteryPercent().ifPresent(
      battery -> SmartDashboard.putNumber("Quest Battery %", battery)
    );
  }
  // gives me the timestamp of the newst frame
  public double getTimestamp(){
    return timestamp;
  }
  
  public void questReset() {
    questNav.setPose(new Pose3d(new Pose2d(0, 0, Rotation2d.kZero)));
  }
}