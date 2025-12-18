// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Path.Trajectory.ChangeReefToClosest;
import frc.robot.Path.Trajectory.FollowTrajectory;
import frc.robot.Path.Utils.PathPoint;
import frc.robot.chassis.commands.Drive;
import frc.robot.chassis.commands.GoToPose;
import frc.robot.chassis.commands.auto.FieldTarget.ELEMENT_POSITION;
import frc.robot.chassis.commands.auto.FieldTarget.FEEDER_SIDE;
import frc.robot.chassis.commands.auto.FieldTarget.LEVEL;
import frc.robot.chassis.commands.auto.FieldTarget.POSITION;
import frc.robot.chassis.commands.auto.AlgaeL3;
import frc.robot.chassis.commands.auto.AlgaeL3L3;
import frc.robot.chassis.commands.auto.AutoUtils;
import frc.robot.chassis.commands.auto.FieldTarget;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.robot1.arm.commands.ArmCommand;
import frc.robot.robot1.arm.commands.ArmDrive;
import frc.robot.robot1.CheckElectronicsRobot;
import frc.robot.robot1.RobotCoastOrBrake;
import frc.robot.robot1.arm.commands.ArmCalibration;
import frc.robot.robot1.arm.constants.ArmConstants.ARM_ANGLE_STATES;
import frc.robot.robot1.arm.subsystems.Arm;
import frc.robot.robot1.climb.command.ClimbUntilSensor;
import frc.robot.robot1.climb.command.JoyClimeb;
import frc.robot.robot1.climb.command.OpenClimber;
import frc.robot.robot1.climb.subsystem.Climb;
import frc.robot.robot1.gripper.commands.GrabOrDrop;
import frc.robot.robot1.gripper.commands.GripperDrive;
import frc.robot.robot1.gripper.subsystems.Gripper;
import frc.robot.leds.Robot1Strip;
import frc.robot.leds.subsystems.LedManager;
import frc.robot.practice.AllOffsets;
import frc.robot.utils.CommandController;
import frc.robot.utils.Elastic;
import frc.robot.utils.LogManager;
import frc.robot.utils.CommandController.ControllerType;
import frc.robot.utils.Elastic.Notification;
import frc.robot.utils.Elastic.Notification.NotificationLevel;
import frc.robot.vision.Quest;
import frc.robot.vision.subsystem.ObjectPose;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Sendable{

  public static ObjectPose objectPose;
  public static RobotContainer robotContainer;
  public static LedManager ledManager;
  public static CommandController driverController;
  // public static CommandController operatorController;
  public static boolean isComp = DriverStation.isFMSAttached();
  private static boolean hasRemovedFromLog = false;
  public static boolean isRed;
  public static Trigger allianceTrigger;

  public static Chassis chassis;  
  public static Arm arm;
  public static Gripper gripper;
  public static Climb climb;
  public static Robot1Strip robot1Strip;
  
  public static FieldTarget scoringTarget = new FieldTarget(POSITION.A, ELEMENT_POSITION.CORAL_LEFT, LEVEL.L3);
  public static FEEDER_SIDE currentFeederSide;

  public SendableChooser<AutoMode> autoChooser;
  public enum AutoMode {
    LEFT, MIDDLE, RIGHT
  }

  public static Command leftAuto;
  public static Command middleAuto;
  public static Command rightAuto;
  public final Timer timer = new Timer();

  private Trigger userButtonTrigger;
  public static Quest quest;


  public static int N_CYCLE = 0;
  public static double CYCLE_TIME = 0.02;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    quest = new Quest();
    // WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    robotContainer = this;
    new LogManager();
    ledManager = new LedManager();

    driverController = new CommandController(OperatorConstants.DRIVER_CONTROLLER_PORT, ControllerType.kXbox);
    // operatorController = new CommandController(OperatorConstants.OPERATOR_CONTROLLER_PORT, ControllerType.kXbox);
    // allianceTrigger = new Trigger(() -> isRed);

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    SmartDashboard.putData("RC", this);
    LogManager.addEntry("Timer", ()-> DriverStation.getMatchTime());
    SmartDashboard.putData("Reef", ReefWidget.getInstance());
    // SmartDashboard.putData("PDH", new PowerDistribution(PowerDistributionConstants.POWER_DISTRIBUTION_ID, PowerDistributionConstants.MODULE_TYPE));
    SmartDashboard.putData("Offsets/Practice", new AllOffsets().ignoringDisable(true));
    // SmartDashboard.putData("Check Electronics", new CheckElectronicsRobot());
    Elastic.sendNotification(new Notification(NotificationLevel.INFO, "Start Robot Code", ""));
    
    configureSubsytems();
    new AutoUtils();
    configureDefaultCommands();
    configureBindings();
    configureAuto();

    // allianceTrigger.onChange(new InstantCommand(() ->
    //   FieldTarget.REEF_POINTS = new PathPoint[]{
    //     new FieldTarget(POSITION.A, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER).getReefAvoidPoint(),
    //     new FieldTarget(POSITION.B, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER).getReefAvoidPoint(),
    //     new FieldTarget(POSITION.C, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER).getReefAvoidPoint(),
    //     new FieldTarget(POSITION.D, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER).getReefAvoidPoint(),
    //     new FieldTarget(POSITION.E, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER).getReefAvoidPoint(),
    //     new FieldTarget(POSITION.F, ELEMENT_POSITION.FEEDER_MIDDLE, LEVEL.FEEDER).getReefAvoidPoint(),
    //   }
    // ).ignoringDisable(true));

    currentFeederSide = FEEDER_SIDE.MIDDLE;
  }

  /**
   * This function start all the subsytems.
   * Put here all the subsystems you want to use.
   * This function is called at the robot container constractor.
   */
  private void configureSubsytems() {
    Ultrasonic.setAutomaticMode(true);
    // UsbCamera povCam = CameraServer.startAutomaticCapture("POV Cam", 0);
    // povCam.setResolution(80, 60);
    // povCam.setFPS(30);

    chassis = new Chassis();
    objectPose = new ObjectPose(chassis.getTag("reefRight").getCamera(),()-> chassis.getGyroAngle(),()-> chassis.getPoseVisionEstimation());
    // arm = new Arm();
    // gripper = new Gripper();
    // climb = new Climb();
    // robot1Strip = new Robot1Strip(chassis, arm, gripper);
  }

  /**
   * This function set all the default commands to the subsystems.
   * set all the default commands of the subsytems.
   * This function is called at the robot container constractor
   */
  private void configureDefaultCommands() {
    chassis.setDefaultCommand(new Drive(chassis, driverController));
    // arm.setDefaultCommand(new ArmCommand(arm));
  }

  private void configureBindings() {
    // userButtonTrigger = new Trigger(() -> RobotController.getUserButton() && !DriverStation.isEnabled());
    // userButtonTrigger.onTrue(new RobotCoastOrBrake(chassis, arm));

    driverController.getLeftStickMove().onTrue(new Drive(chassis, driverController));
    driverController.downButton().onTrue(new GoToPose());
  
  }

  private void configureAuto() {
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("LEFT", AutoMode.LEFT);
    autoChooser.addOption("MIDDLE", AutoMode.MIDDLE);
    autoChooser.addOption("RIGHT", AutoMode.RIGHT);
    SmartDashboard.putData("AutoChooser", autoChooser);
    // leftAuto = new ArmCommand(arm).alongWith(new AlgaeL3L3(chassis, arm, gripper, isRed, false)).finallyDo((boolean interrupted) -> Elastic.selectTab("Teleoperated")).withName("Left Auto");
    // middleAuto = new ArmCommand(arm).alongWith(new AlgaeL3(chassis, arm, gripper)).finallyDo((boolean interrupted) -> Elastic.selectTab("Teleoperated")).withName("Middle Auto");
    // rightAuto = new ArmCommand(arm).alongWith(new AlgaeL3L3(chassis, arm, gripper, isRed, true)).finallyDo((boolean interrupted) -> Elastic.selectTab("Teleoperated")).withName("Right Auto");

    // allianceTrigger.onChange(new InstantCommand(() -> {
      // leftAuto = new ArmCommand(arm).alongWith(new AlgaeL3L3(chassis, arm, gripper, isRed, false)).finallyDo((boolean interrupted) -> Elastic.selectTab("Teleoperated")).withName("Left Auto");
      // middleAuto = new ArmCommand(arm).alongWith(new AlgaeL3(chassis, arm, gripper)).finallyDo((boolean interrupted) -> Elastic.selectTab("Teleoperated")).withName("Middle Auto");
      // rightAuto = new ArmCommand(arm).alongWith(new AlgaeL3L3(chassis, arm, gripper, isRed, true)).finallyDo((boolean interrupted) -> Elastic.selectTab("Teleoperated")).withName("Right Auto");
    // }).ignoringDisable(true));
  }

  public static boolean isRed() {
    return isRed;
  }

  public static void setIsRed(boolean isRed) {
    RobotContainer.isRed = isRed;
  }

  public static boolean isComp() {
    return isComp;
  }

  public static void setIsComp(boolean isComp) {
    RobotContainer.isComp = isComp;
    if(!hasRemovedFromLog && isComp) {
      hasRemovedFromLog = true;
      LogManager.removeInComp();
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("isRed", RobotContainer::isRed, RobotContainer::setIsRed);
    builder.addBooleanProperty("isComp", RobotContainer::isComp, RobotContainer::setIsComp);
  }

  /**
   * This command is schedules at the start of teleop.
   * Look in {@link Robot} for more details.
   * @return the ommand that start at the start at enable
   */
  public Command getEnableInitCommand() {
    return null;
    // return new ArmCalibration(arm);
  }

  /**
   * This command is schedules at the start of disable.
   * Put here all the stop functions of all the subsytems and then add them to the requirments
   * This insures that the motors do not keep their last control mode earlier and moves uncontrollably.
   * Look in {@link Robot} for more details.
   * @return the command that runs at disable
   */
  public Command getDisableInitCommand() {
    return new InstantCommand(()-> {
      chassis.stop();
      // arm.stop();
      // gripper.stop();
      // climb.stopClimb();
      timer.stop();
    }, chassis
    // , arm, gripper, climb
    ).withName("initDisableCommand").ignoringDisable(true);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    timer.reset();
    timer.start();
    return null;
    // switch (autoChooser.getSelected()) {
    //   case LEFT:
    //     return leftAuto;

    //   case MIDDLE:
    //     return middleAuto;
      
    //   case RIGHT: 
    //     return rightAuto;
    
    //   default:
    //     return new RunCommand(()-> chassis.setRobotRelVelocities(new ChassisSpeeds(2, 0, 0)), chassis).withTimeout(1);
    // }
  }
}
