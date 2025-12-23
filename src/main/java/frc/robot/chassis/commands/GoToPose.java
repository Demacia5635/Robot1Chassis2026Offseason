// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.commands;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.vision.subsystem.Quest;

public class GoToPose extends Command {
  double dKp = 0.9;
  double rKp = 1.2;
  Chassis chassis;
  Quest quest;
  Pose2d wantedPose = new Pose2d();
  public GoToPose() {
    this.chassis = RobotContainer.chassis;
    this.quest = RobotContainer.quest;
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Translation2d diffVector = quest.getPose().getTranslation();
    ChassisSpeeds speeds = new ChassisSpeeds(diffVector.getY() * dKp, -diffVector.getX() * dKp, 0);
    if(Math.abs(chassis.getPose().getRotation().getDegrees()) <= 2) speeds = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0);
    chassis.setVelocities(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(quest.getPose().getTranslation().getX()) < 0.01 && Math.abs(quest.getPose().getTranslation().getY()) < 0.01;
  }
}
