
package frc.robot.vision.subsystem;



import edu.wpi.first.math.geometry.Pose2d;
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
  private Pose2d currentPose;
  private PoseFrame[] poseFrames;

  public Quest() {
    questNav = new QuestNav();
    poseFrames = questNav.getAllUnreadPoseFrames();

if (poseFrames.length > 0) {
    // Get the most recent Quest pose
    currentPose = poseFrames[poseFrames.length - 1].questPose().plus(OFFSET_ROBOT_TO_QUEST);
}
    field = new Field2d();
    if(currentPose != null){
      field.setRobotPose(currentPose);
      questNav.setPose(currentPose);
      
    }
    SmartDashboard.putData("Quest Field", field);

    

  }

  public Pose2d getPose() {
    return currentPose;
  }
  public void questReset(){
    questNav.setPose(new Pose2d(0,0, new Rotation2d()).plus(OFFSET_ROBOT_TO_QUEST));
  }
  

  @Override
  public void periodic() {
    questNav.commandPeriodic();
      
      // Connection status
    poseFrames = questNav.getAllUnreadPoseFrames();

      if (poseFrames.length > 0) {
        // Get the most recent Quest pose
        currentPose = poseFrames[poseFrames.length - 1].questPose().plus(OFFSET_ROBOT_TO_QUEST);
    }
      
      // Position data
      
    if(currentPose != null){
      SmartDashboard.putNumber("Quest X", currentPose.getX());
      SmartDashboard.putNumber("Quest Y", currentPose.getY());
      
      field.setRobotPose(currentPose);
    }


  }
}