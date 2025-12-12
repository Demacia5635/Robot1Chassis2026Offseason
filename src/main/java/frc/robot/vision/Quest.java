
package frc.robot.vision;



import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class Quest extends SubsystemBase {
  private Field2d field;
  private QuestNav questNav;
  private Pose2d currentPose;
  private PoseFrame[] poseFrames;

  public Quest() {
    questNav = new QuestNav();
    poseFrames = questNav.getAllUnreadPoseFrames();

if (poseFrames.length > 0) {
    // Get the most recent Quest pose
    currentPose = poseFrames[poseFrames.length - 1].questPose();
}
    field = new Field2d();
    if(currentPose != null){
      field.setRobotPose(currentPose);
      
    }
    SmartDashboard.putData("Quest Field", field);

    

    SmartDashboard.putNumber("Quest X", 0);
    SmartDashboard.putNumber("Quest Y", 0);
  }

  public Pose2d getPose() {
    return currentPose;
  }
  

  @Override
  public void periodic() {
      
      // Connection status
      poseFrames = questNav.getAllUnreadPoseFrames();

      if (poseFrames.length > 0) {
        // Get the most recent Quest pose
        currentPose = poseFrames[poseFrames.length - 1].questPose();
    }
      
      // Position data
      
    if(currentPose != null){
      SmartDashboard.putNumber("Quest X",   currentPose.getX());
      SmartDashboard.putNumber("Quest Y", currentPose.getY());
      
      field.setRobotPose(currentPose);
    }
    else{
      SmartDashboard.putNumber("Quest X",   0.0);
      SmartDashboard.putNumber("Quest Y", 0.0);
    }


  }
}