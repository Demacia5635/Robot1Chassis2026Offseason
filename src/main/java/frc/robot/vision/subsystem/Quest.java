// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.subsystem;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

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
      questPose = poseFrames[poseFrames.length - 1].questPose();
  }
    return questPose;
  }

  @Override
  public void periodic() {
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

