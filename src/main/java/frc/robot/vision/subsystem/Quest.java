// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.subsystem;


import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import static frc.robot.vision.utils.VisionConstants.OFFSET_ROBOT_TO_QUEST;


public class Quest extends SubsystemBase {
  QuestNav questNav;
  /** Creates a new Quest. */
  public Quest() {
    questNav = new QuestNav();
  }
  public Pose2d questPose2d(){
    Pose2d questPose = new Pose2d();
    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
    if (poseFrames.length > 0) {
      // Get the most recent Quest pose
      questPose = poseFrames[poseFrames.length - 1].questPose().plus(OFFSET_ROBOT_TO_QUEST);
  }
    return questPose;
  }

  @Override
  public void periodic() {
    questNav.commandPeriodic();//Cleans up QuestNav responses after processing on the headset
    // and if we don't have data or for some reason the response we got isn't for the command we sent, skip for this loop
    super.periodic();
    // This method will be called once per scheduler run


  }
  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    SmartDashboard.putNumber("quest x",questPose2d().getX());

    SmartDashboard.putNumber("Quest Y", questPose2d().getY());
  }
}

