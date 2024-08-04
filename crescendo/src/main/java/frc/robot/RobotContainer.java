// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArmPivot.AMPScoringPos;
import frc.robot.commands.ArmPivot.ArmPivotErrected;
import frc.robot.commands.ArmPivot.ArmPivotHumanFeeder;
import frc.robot.commands.ArmPivot.ArmPivotLineScoring;
import frc.robot.commands.ArmPivot.ArmPivotShooting;
import frc.robot.commands.ArmPivot.ArmPivotStore;
import frc.robot.commands.ArmPivot.ArmPivotTrap;
import frc.robot.commands.ArmPivot.ShootingWithoutCameras;
import frc.robot.commands.ArmPivot.ShootingWithoutCameras2ndStageLeg;
import frc.robot.commands.ArmPivot.ShootingWithoutCamerasN1;
import frc.robot.commands.ArmPivot.ShootingWithoutCamerasStageLeg;
import frc.robot.commands.Hybrid.DriveTest;
import frc.robot.commands.Hybrid.SelfDrivingShot;
import frc.robot.commands.Hybrid.LineBreakArmPivot.ReadyToShoot;
import frc.robot.commands.Hybrid.LineBreakFullShooter.FeedToShoot3;
import frc.robot.commands.Hybrid.LineBreakFullShooter.HasNote1;
import frc.robot.commands.Hybrid.LineBreakFullShooter.HasNote2;
import frc.robot.commands.Intake.FloorFeederTest;
import frc.robot.commands.Intake.ReverseIntake;
import frc.robot.commands.ShooterFeederWheels.ShooterFeederAMP;
import frc.robot.commands.ShooterFeederWheels.ShooterFeederFire;
import frc.robot.commands.ShooterFeederWheels.ShooterFeederHuman;
import frc.robot.commands.ShooterFeederWheels.ShooterFeederPickUp;
import frc.robot.commands.ShooterWheels.ShooterAMP;
import frc.robot.commands.ShooterWheels.ShooterCrossField;
import frc.robot.commands.ShooterWheels.ShooterTest;
import frc.robot.commands.ShooterWheels.ShooterTrapScoring;
import frc.robot.commands.Swerve.AlignPoseSpeaker;
import frc.robot.commands.Swerve.LaneLogicIn;
import frc.robot.commands.Swerve.LaneLogicOut;
import frc.robot.commands.Swerve.MoveToNote;
import frc.robot.commands.Swerve.MoveToPose;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;

import java.util.function.BiFunction;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static SwerveBase m_swerveBase = new SwerveBase();
  //public static SwerveModule m_SwerveModule = SwerveModule();
  public static AprilTagSubsystem m_aprilTag = new AprilTagSubsystem();
  // do not uncomment public static ArmPivotSubsystem m_armPivotSubsystem = new ArmPivotSubsystem(m_swerveBase);
  public static IntakeSubsystem m_intake = new IntakeSubsystem();
  public static LineBreak m_lineBreak = new LineBreak();
  public ShooterFeederSubsystem m_shooterFeeder = new ShooterFeederSubsystem(this);
  public static ShooterSubsystem m_shooter = new ShooterSubsystem();
  public static LEDSubsystem m_led = new LEDSubsystem();
  public static SelfDriving m_selfDriving = new SelfDriving();
  public static NoteDetection m_note_detection = new NoteDetection();
  
 
  public static ArmPivotSubsystem m_ArmPivotSubsystem;

  //Defines all mChooser that will display in suffle board
  private SendableChooser<String> mChooser;
  private SendableChooser<String> mChooser1;
  private SendableChooser<String> mChooser2;
  private SendableChooser<String> mChooser3;
  private SendableChooser<String> mChooser4;
  private SendableChooser<String> mChooser5;
  private SendableChooser<String> mChooser6;
  

  // public PowerDistributionPanel newPower = new PowerDistributionPanel(0);
  // public ClimberSubsystem m_climber = new ClimberSubsystem();
  // private ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  /* Human interfaces */
  private final Joystick driverJoystick;
  private final Joystick streamdeck;
  
  //Defines all buttons that will be used to control the robot
  // private JoystickButton btn_arm_pivot_down;
  private JoystickButton btn_arm_pivot_up;
  private JoystickButton btn_shooter_feeder;
  private JoystickButton btn_floor_feeder;
  private JoystickButton btn_armP_pivot_stop;
  // private JoystickButton btn_amp_scoring_pos;
  private JoystickButton btn_shooting;
  private JoystickButton btn_human_feeder;
  private JoystickButton btn_store;
  private JoystickButton btn_shooting_without_cameras;
  private JoystickButton btn_shooting_without_cameras_stage_leg;
  private JoystickButton btn_shooting_without_cameras_2nd_stage_leg;
  private JoystickButton btn_amp_scoring_pos;
  private JoystickButton btn_reset_yaw;
  private JoystickButton btn_aim_speaker;
  private JoystickButton btn_aim_amp;
  private JoystickButton btn_aim_human_feeder;
  private JoystickButton btn_led_win;
  private JoystickButton btn_shooter;
  private JoystickButton btn_defence;
  private JoystickButton btn_errected;
  private JoystickButton btn_line_scoring;
  private JoystickButton btn_cross_field;
  private JoystickButton btn_reverse_feeder;
  private JoystickButton btn_aim_line;
  private JoystickButton btn_driver_N1;
  private JoystickButton btn_trap_scoring;
  private JoystickButton btn_far_feeder;

  private JoystickButton btn_auto_pickup;
  private JoystickButton btn_more_amps;
  private JoystickButton btn_faster_swerve;
  private JoystickButton btn_slower_swerve;
  private JoystickButton btn_self_driving_shoot;
  private JoystickButton btn_pickup_note_to_shoot;

  private JoystickButton btn_shooting_with_driver;  
  private JoystickButton btn_driver_fire;  
  private JoystickButton btn_pose_note;


  private DoubleSupplier limit;
  private DoubleSupplier stopRotation;
  private DoubleSupplier stopMainualDriving;
  private BiFunction<Double, Double, Double> Clamp;
  
  public static PIDController angleController;

  public static boolean Camera1_InAuto;
  public static boolean Camera2_InAuto;
  public static boolean Camera3_InAuto;
  public static boolean Camera4_InAuto;
  public static boolean Camera5_InAuto;
  

//  public static boolean isRed= true;
 
  /* Subsystems */
  // to bring back arm pivot


  
  /* Parent Class */
  private final Robot m_robot;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = 
  new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(Robot robot) {
    m_robot = robot;

    //Defines what port of the compter each controler will be located in
    driverJoystick = new Joystick(0);
    streamdeck = new Joystick(1);


    //Put auto in here and they will show up in smart dash board do no forget to select auto before match
    mChooser = new SendableChooser<>();
    SmartDashboard.putData("Auto Choices",  mChooser);
    mChooser.setDefaultOption("Default Auto", "StraitLineTest");
    mChooser.addOption("6 Piece", "C-N3-Shoot6-N7-Shoot6");
    mChooser.addOption("Test Aim", "AlignShooterTest");
    mChooser.addOption("A-N1", "A-N1");
    mChooser.addOption("Hellos", "Hellos");
    mChooser.addOption("B-Shoot2-N2-Shoot5", "B-Shoot2-N2-Shoot5");
    mChooser.addOption("StraitLineTest", "StraitLineTest");
    mChooser.addOption("Note Stop Test", "Note Stop Test");
    mChooser.addOption("NotePickupTest", "NotePickupTest");
    mChooser.addOption("NotePickupTest2", "NotePickupTest2");
    mChooser.addOption("103 Auto", "103 Auto");
    mChooser.addOption("RotationTest", "RotationTest");
    

    //Used to turn Cameras on and off in auto
    mChooser1 = new SendableChooser<>();
    mChooser1.setDefaultOption("Yes", "Yes");
    mChooser1.addOption("No", "No");
    SmartDashboard.putData("Camera-1 in Auto" ,  mChooser1);

    mChooser2 = new SendableChooser<>();
    mChooser2.setDefaultOption("Yes", "Yes");
    mChooser2.addOption("No", "No");
    SmartDashboard.putData("Camera-2 in Auto" ,  mChooser2);

    mChooser3 = new SendableChooser<>();
    mChooser3.setDefaultOption("Yes", "Yes");
    mChooser3.addOption("No", "No");
    SmartDashboard.putData("Camera-3 in Auto" ,  mChooser3);

    mChooser4 = new SendableChooser<>();
    mChooser4.setDefaultOption("Yes", "Yes");
    mChooser4.addOption("No", "No");
    SmartDashboard.putData("Camera-4 in Auto" ,  mChooser4);

    mChooser5 = new SendableChooser<>();
    mChooser5.setDefaultOption("Yes", "Yes");
    mChooser5.addOption("No", "No");
    SmartDashboard.putData("Camera-5 in Auto" ,  mChooser5);

    

    //Used to make isRed true or false to inverts what side the robot is on
    // mChooser6 = new SendableChooser<>();
    // mChooser6.setDefaultOption("Red", "Red");
    // mChooser6.addOption("Blue", "Blue");
    // SmartDashboard.putData("Aliance Color",  mChooser6);

 
    // Controles rotaion Whne Auto Targeting
     angleController = new PIDController(1.0, 0.0, 0.0);
     angleController.enableContinuousInput(-Math.PI, Math.PI);
    
         //Added by Spencer to ramp power by lever
    //limit = () -> 0.55 - 0.45 * driverJoystick.getRawAxis(SwerveConstants.sliderAxis);
    /* maps sliderAxis to be between 0.1 and 1.0 */
   // Clamp  = (val, lim) -> (Math.abs(val) < lim) ? val : Math.copySign(lim, val);

   //Old controler interface that used Spencer clamp which had now been moved to control max speed thought TeleOp Swerve
     // m_swerveBase = new SwerveBase();
    // m_swerveBase.setDefaultCommand(
    //     new TeleopSwerve(
    //         m_swerveBase,
    //         () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.translationAxis) , limit.getAsDouble()),
    //         () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.strafeAxis), limit.getAsDouble()),
    //         () -> -Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.rotationAxis) ,limit.getAsDouble() * stopRotation.getAsDouble()),
    //         () -> !driverJoystick.getRawButton(1) // inverted=fieldCentric, non-inverted=RobotCentric
            
    //     ));

    //Stops the robot from reciving any rotation command
   stopRotation = () -> driverJoystick.getRawButton(9) ? 0.0 : 1.0;
    //New driver interface without clamp and new lever ramp range from 20%-100% commanded max power
  
  
    m_swerveBase.setDefaultCommand(
        new TeleopSwerve(
            m_swerveBase,
            () -> (driverJoystick.getRawAxis(SwerveConstants.translationAxis)),
            () -> (driverJoystick.getRawAxis(SwerveConstants.strafeAxis)),
            () -> -(driverJoystick.getRawAxis(SwerveConstants.rotationAxis)* stopRotation.getAsDouble()),
            () -> (-(driverJoystick.getRawAxis(SwerveConstants.sliderAxis)-1)/2.5+0.2),
            () -> !driverJoystick.getRawButton(1) // inverted=fieldCentric, non-inverted=RobotCentric
          
        ) );
 
    // to bring back arm pivot
    m_ArmPivotSubsystem = new ArmPivotSubsystem(m_swerveBase);

    // Configure the trigger bindings
    configureBindings();
  }


  public Joystick getJoystick() {
    return driverJoystick;
  }

  //
  public Joystick getstreamdeck() {
    return streamdeck;
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // btn_example = new Trigger(m_exampleSubsystem::exampleCondition)
    // btn_example.onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());



    //The following commands should be transfered year to you

    //ReZeros pigeon if intale calibration was not correct
    btn_reset_yaw = new JoystickButton(driverJoystick, 7);
    btn_reset_yaw.onTrue(new InstantCommand(() -> m_swerveBase.setNeedPigeonReset(true)));

    //Sets the robot max amp draw to more then normal
     btn_more_amps = new JoystickButton(driverJoystick, 8);
     btn_more_amps.whileTrue(new RunCommand(() -> m_swerveBase.setNeedMoreAmps(true)));
     btn_more_amps.onFalse(new RunCommand(() -> m_swerveBase.setNeedMoreAmps(false)));
      
     //This will change max swerve speed thought SwerveBase to slower
     btn_slower_swerve = new JoystickButton(driverJoystick, 8);
     btn_slower_swerve.whileTrue(new RunCommand(() -> m_swerveBase.setSlowerSwerve(true)));
     btn_slower_swerve.onFalse(new RunCommand(() -> m_swerveBase.setSlowerSwerve(false)));

    //This will change max swerve speed thought SwerveBase to faster
     btn_faster_swerve = new JoystickButton(driverJoystick, 5);
     btn_faster_swerve.whileTrue(new RunCommand(() -> m_swerveBase.setFasterSwerve(true)));
     btn_faster_swerve.onFalse(new RunCommand(() -> m_swerveBase.setFasterSwerve(false)));
    


     //The following are the hybrid commands that use multiple systems in one button

    // // //Auto Dirves to note
    // btn_auto_pickup = new JoystickButton(driverJoystick, 4);
    // btn_auto_pickup.whileTrue(new MoveToNote(m_swerveBase));
    // btn_auto_pickup.whileTrue(new FloorFeederTest(m_intake));
    // btn_auto_pickup.whileTrue(new ArmPivotErrected(m_ArmPivotSubsystem));
    // btn_auto_pickup.whileTrue(new ShooterFeederPickUp(m_shooterFeeder));
    // //btn_auto_pickup.whileTrue(new RunCommand(() -> m_note_detection.FindTheNotes()));

    //Auto drive to estimated note pose
    // btn_auto_pickup = new JoystickButton(driverJoystick, 4);
    // btn_auto_pickup.whileTrue(new MoveToNotePose(m_swerveBase));
    // btn_auto_pickup.whileTrue(new FloorFeederTest(m_intake));
    // btn_auto_pickup.whileTrue(new ArmPivotErrected(m_ArmPivotSubsystem));
    // btn_auto_pickup.whileTrue(new ShooterFeederPickUp(m_shooterFeeder));
    // btn_auto_pickup.whileTrue(new RunCommand(() -> m_note_detection.FindTheNotes()));

    
     btn_self_driving_shoot = new JoystickButton(driverJoystick, 3);
     btn_self_driving_shoot.whileTrue(new RunCommand(() -> m_selfDriving.setTargetPose(10)));
     btn_self_driving_shoot.onFalse(new RunCommand(() -> m_selfDriving.setTargetPose(0)));
     btn_self_driving_shoot.whileTrue(new LaneLogicIn(m_swerveBase));


    btn_pickup_note_to_shoot = new JoystickButton(driverJoystick, 4);
    btn_pickup_note_to_shoot.whileTrue(new SelfDrivingShot(m_shooter, m_ArmPivotSubsystem,  m_shooterFeeder, m_lineBreak, m_selfDriving, m_swerveBase, m_intake)/*.repeatedly()*/);

    // btn_pickup_note_to_shoot = new JoystickButton(driverJoystick, 4);
    // btn_pickup_note_to_shoot.whileTrue(new DriveTest(m_swerveBase));

    btn_shooter_feeder = new JoystickButton(driverJoystick, 11);
    btn_shooter_feeder.whileTrue(new ShooterFeederAMP(m_shooterFeeder));
   
    btn_driver_fire = new JoystickButton(driverJoystick, 12);
    btn_driver_fire.whileTrue(new ShooterFeederFire(m_shooterFeeder));

    // btn_driver_N1 = new JoystickButton(driverJoystick, 10);
    // btn_driver_N1.whileTrue(new ShootingWithoutCamerasN1(m_ArmPivotSubsystem));


    // btn_arm_pivot_up = new JoystickButton(driverJoystick, 3);
    // btn_aim_speaker.whileTrue(new ArmPivotShooting(m_ArmPivotSubsystem));

    // btn_aim_speaker = new JoystickButton(streamdeck, 1);
    // btn_aim_speaker.whileTrue(
    //     new TeleopSwerve(
    //         m_swerveBase,
    //         () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.translationAxis), limit.getAsDouble()),
    //         () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.strafeAxis), limit.getAsDouble()),
    //         () -> angleController.calculate(m_swerveBase.getPose().getRotation().getRadians(), m_swerveBase.getAngleToSpeaker().getRadians()),
    //         () -> !driverJoystick.getRawButton(1) // inverted=fieldCentric, non-inverted=RobotCentric
    //     ));
    // btn_aim_speaker.whileTrue(new ArmPivotShooting(m_ArmPivotSubsystem));

    // btn_aim_speaker = new JoystickButton(streamdeck, 6);
    // btn_aim_speaker.whileTrue(new ArmPivotShooting(m_ArmPivotSubsystem));
   
  //  btn_aim_speaker = new JoystickButton(driverJoystick, 6);
  //   btn_aim_speaker.whileTrue(new ArmPivotShooting(m_ArmPivotSubsystem));
  //   btn_aim_speaker.whileTrue(
  //       new TeleopSwerve(
  //           m_swerveBase,
  //           () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.translationAxis), limit.getAsDouble()),
  //           () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.strafeAxis), limit.getAsDouble()),
  //           () -> angleController.calculate(m_swerveBase.getPose().getRotation().getRadians(), m_swerveBase.getAngleToSpeaker().getRadians()),
  //           () -> !driverJoystick.getRawButton(1) // inverted=fieldCentric, non-inverted=RobotCentric
  //       ));

//Currently not working
  // btn_pose_note = new JoystickButton(driverJoystick, 6);
  //   // btn_pose_note.whileTrue(new MoveToNoteByPose(m_swerveBase));
  //   btn_pose_note.whileTrue(new MoveToPose(m_swerveBase));
  //   btn_pose_note.whileTrue(new RunCommand(() -> m_selfDriving.setTargetPose(3)));
  //   btn_pose_note.onFalse(new RunCommand(() -> m_selfDriving.setTargetPose(0)));
  //   btn_pose_note.whileTrue(new RunCommand(() -> m_selfDriving.DriveCalculation()));


        // btn_aim_line = new JoystickButton(streamdeck, 1);
        // btn_aim_line.whileTrue(
        // new TeleopSwerve(
        //     m_swerveBase,
        //     () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.translationAxis), limit.getAsDouble()),
        //     () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.strafeAxis), limit.getAsDouble()),
        //     () -> angleController.calculate(m_swerveBase.getPose().getRotation().getRadians(), m_ArmPivotSubsystem.getLineAngle().getRadians()),
        //    () -> !driverJoystick.getRawButton(1) // inverted=fieldCentric, non-inverted=RobotCentric
        // ));
   
    // btn_aim_amp = new JoystickButton(streamdeck, 10);
    // btn_aim_amp.whileTrue(
    //     new TeleopSwerve(
    //         m_swerveBase,
    //         () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.translationAxis), limit.getAsDouble()),
    //         () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.strafeAxis), limit.getAsDouble()),
    //         () -> angleController.calculate(m_swerveBase.getPose().getRotation().getRadians(), m_ArmPivotSubsystem.getAmpAngle().getRadians()),
    //         () -> !driverJoystick.getRawButton(1) // inverted=fieldCentric, non-inverted=RobotCentric
    //     ));

    btn_aim_human_feeder = new JoystickButton(driverJoystick, 9);
    //btn_aim_human_feeder.whileTrue(
        // new TeleopSwerve(
        //     m_swerveBase,
        //     () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.translationAxis), limit.getAsDouble()),
        //     () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.strafeAxis), limit.getAsDouble()),
        //     () -> angleController.calculate(m_swerveBase.getPose().getRotation().getRadians(), m_ArmPivotSubsystem.getHumanFeederAngle().getRadians()),
        //    () -> !driverJoystick.getRawButton(1) // inverted=fieldCentric, non-inverted=RobotCentric
        // ));
    
    // Intakes Note From Floor And Uses Line Breaks To Stop Note At Specific Position
    btn_floor_feeder = new JoystickButton(streamdeck, 5);
    btn_floor_feeder.whileTrue(new FloorFeederTest(m_intake));
    btn_floor_feeder.whileTrue(new ArmPivotErrected(m_ArmPivotSubsystem));
    btn_floor_feeder.whileTrue(new ShooterFeederPickUp(m_shooterFeeder));

    btn_reverse_feeder = new JoystickButton(streamdeck, 4);
    btn_reverse_feeder.whileTrue(new ReverseIntake(m_intake));
    btn_reverse_feeder.whileTrue(new ArmPivotErrected(m_ArmPivotSubsystem));
  


// // Note intake no robot centric
//     btn_floor_feeder = new JoystickButton(streamdeck, 15);
//     btn_floor_feeder.whileTrue(new FloorFeederTest(m_intake));
//     btn_floor_feeder.whileTrue(new ArmPivotErrected(m_ArmPivotSubsystem));
//     btn_floor_feeder.whileTrue(new ShooterFeederPickUp(m_shooterFeeder));
    // Turns on Shooter Wheels when held
    // btn_arm_pivot_up = new JoystickButton(streamdeck, 2);
    // btn_arm_pivot_up.whileTrue(new ShooterTest(m_robot.getShooterSubsystem()));

    // // Will Push The Note At Full Speed Into Shooter
    // btn_arm_pivot_up = new JoystickButton(driverJoystick, 1);
    // btn_arm_pivot_up.whileTrue(new ShooterFeederFire(m_shooterFeeder));

    btn_shooter = new JoystickButton(streamdeck, 6);
    btn_shooter.whileTrue(new ShooterTest(m_shooter));

    // Feeds Note At Slower Speed For AMP
    btn_shooter_feeder = new JoystickButton(streamdeck, 8);
    btn_shooter_feeder.whileTrue(new ShooterFeederAMP(m_shooterFeeder));

    // btn_led_win = new JoystickButton(streamdeck, 2);
    // btn_led_win.whileFalse(new LEDWinYes(m_led));
    // btn_led_win.whileTrue(new LEDWinNo(m_led));

    //Buton auto intakes and shootes
    btn_far_feeder = new JoystickButton(driverJoystick, 2);
    btn_far_feeder.toggleOnTrue(new FeedToShoot3(m_shooter, m_ArmPivotSubsystem,  m_shooterFeeder, m_lineBreak).repeatedly());
  //btn_far_feeder.whileTrue(new Commands.repeatedly(() -> (new FeedToShoot(m_shooter, m_ArmPivotSubsystem, m_intake,  m_shooterFeeder, m_swerveBase))));
   


    // Gets Note From Human Feeder And Uses Line Breaks To Stop Note At Specific Position
    btn_human_feeder = new JoystickButton(streamdeck, 9);
    btn_human_feeder.whileTrue(new ArmPivotHumanFeeder (m_ArmPivotSubsystem));
    btn_human_feeder.whileTrue(new ShooterFeederHuman(m_shooterFeeder));

    
    // Moves Arm Into Travel Position
    btn_store = new JoystickButton(streamdeck, 10);
    btn_store.whileTrue(new ArmPivotStore (m_ArmPivotSubsystem));
  
    // Shoots Note From Directly In Front Of Speaker
    btn_shooting_without_cameras = new JoystickButton(streamdeck, 3);
    btn_shooting_without_cameras.whileTrue(new ShootingWithoutCameras(m_ArmPivotSubsystem));
    btn_shooting_without_cameras.whileTrue(new ShooterTest(m_shooter));

    //Shoots Note Cross Field
    btn_cross_field = new JoystickButton(streamdeck, 11);
    btn_cross_field.whileTrue(new ShootingWithoutCameras(m_ArmPivotSubsystem));
    btn_cross_field.whileTrue(new ShooterCrossField(m_shooter));

    //Shoots Note Behind Black Line
    btn_line_scoring = new JoystickButton(streamdeck, 12);
    btn_line_scoring.whileTrue(new ArmPivotLineScoring(m_ArmPivotSubsystem));
    btn_line_scoring.whileTrue(new ShooterTest(m_shooter));

    btn_trap_scoring = new JoystickButton(streamdeck, 1);
    btn_trap_scoring.whileTrue(new ArmPivotTrap(m_ArmPivotSubsystem));
    btn_trap_scoring.whileTrue(new ShooterTrapScoring(m_shooter));

    // // Shoots Note From Front Leg Of Stage
    // btn_shooting_without_cameras_stage_leg = new JoystickButton(streamdeck,8);
    // // btn_shooting_without_cameras_stage_leg.whileFalse(new ArmPivotStore(m_robot.getArmPivotSubsystem()));
    // btn_shooting_without_cameras_stage_leg.whileTrue(new ShootingWithoutCamerasStageLeg(m_ArmPivotSubsystem));
    // btn_shooting_without_cameras_stage_leg.whileTrue(new ShooterTest(m_shooter));

    // // Shoots Note From Right Leg Of Stage
    // btn_shooting_without_cameras_2nd_stage_leg = new JoystickButton(streamdeck, 9);
    // // btn_shooting_without_cameras_2nd_stage_leg.whileFalse(newArmPivotStore(m_robot.getArmPivotSubsystem()));
    // btn_shooting_without_cameras_2nd_stage_leg.whileTrue(new ShootingWithoutCameras2ndStageLeg(m_ArmPivotSubsystem));
    // btn_shooting_without_cameras_2nd_stage_leg.whileTrue(new ShooterTest(m_shooter));

    // Shoots Note To Go Into AMP
    btn_amp_scoring_pos = new JoystickButton(streamdeck, 7);
    btn_amp_scoring_pos.whileTrue(new AMPScoringPos(m_ArmPivotSubsystem));
    btn_amp_scoring_pos.whileTrue(new ShooterAMP(m_shooter));

    // btn_defence = new JoystickButton(streamdeck, 13);
    // btn_defence.whileTrue(new ShootingDefencePos(m_ArmPivotSubsystem));
    // btn_defence.whileTrue(new ShooterDefence(m_shooter));
    // btn_defence.whileTrue(new FloorFeederTest(m_intake));
    // btn_floor_feeder.whileTrue(new ShooterFeederPickUp(m_shooterFeeder));

    // Lets Pathplanner acsess commands 
    NamedCommands.registerCommand("Shoot", (new ArmPivotShooting(m_ArmPivotSubsystem)));
    NamedCommands.registerCommand("Store", (new ArmPivotStore(m_ArmPivotSubsystem)));
    NamedCommands.registerCommand("ShootFrontofSpeaker", (new ShootingWithoutCameras (m_ArmPivotSubsystem)));
    NamedCommands.registerCommand("ShootN1", (new ShootingWithoutCamerasN1(m_ArmPivotSubsystem)));
    NamedCommands.registerCommand("ShootLeg1", (new ShootingWithoutCamerasStageLeg (m_ArmPivotSubsystem)));
    NamedCommands.registerCommand("ShootLeg2", (new ShootingWithoutCameras2ndStageLeg (m_ArmPivotSubsystem)));
    NamedCommands.registerCommand("Errected", (new ArmPivotErrected(m_ArmPivotSubsystem)));
    NamedCommands.registerCommand("Shoot", (new ShooterTest (m_shooter)));
    NamedCommands.registerCommand("Feed", (new ShooterFeederFire(m_shooterFeeder)));
    NamedCommands.registerCommand("Intake", (new FloorFeederTest (m_intake)));
    NamedCommands.registerCommand("FeederIntake", (new ShooterFeederPickUp(m_shooterFeeder)));
    NamedCommands.registerCommand("Align", (new AlignPoseSpeaker(m_swerveBase)));
    NamedCommands.registerCommand("AlignShooter", (new ArmPivotShooting(m_ArmPivotSubsystem)));
    NamedCommands.registerCommand("HasNote1", (new HasNote1(m_lineBreak)));
    NamedCommands.registerCommand("HasNote2", (new HasNote2(m_lineBreak)));
    NamedCommands.registerCommand("MovetoNote", (new MoveToNote(m_swerveBase)));
    NamedCommands.registerCommand("BlackLine", (new ArmPivotLineScoring(m_ArmPivotSubsystem)));
    NamedCommands.registerCommand("ReadyToShoot", (new ReadyToShoot(m_lineBreak)));
    NamedCommands.registerCommand("Store", (new ArmPivotStore(m_ArmPivotSubsystem)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    return AutoBuilder.buildAuto(mChooser.getSelected());//mChooser.getSelected() will get the auto selected from smart dashboard
    //if you want hard coded auto do "AutoName"
  //   Command autonomousCommand = new AutoTest(
  //   m_shooter,
  //   m_ArmPivotSubsystem,
  //   m_intake,
  //   m_shooterFeeder,
  //   m_swerveBase,
  //   m_lineBreak
  //  );

    //return autonomousCommand;
  }

  public SwerveBase getSwerveSubsytem() {
    return m_swerveBase;
  }


  public LineBreak getLineBreakSubsystem() {
    return m_lineBreak;
  }

  public ShooterSubsystem getShooterSubsystem() {
    return m_shooter;
  }

  
  //Take what mChooser says and makes isRed true or false only work beacuse method is called in robot(telop perodic)
  public void SmartDashboardtoCommands() {
  // if (mChooser6.getSelected() == "Blue") {
  //  isRed = false;
  // }
  // if (mChooser6.getSelected() == "Red"){
  //  isRed = true;
  //   }

  if(mChooser.getSelected() == "Test Aim"){

    new RunCommand(() -> m_selfDriving.setTargetPose(4));
  }

  if (mChooser1.getSelected() == "No") {
    Camera1_InAuto = false;
  }
  if (mChooser1.getSelected() == "Yes"){
   Camera1_InAuto = true;
  }
  
  if (mChooser2.getSelected() == "No") {
    Camera2_InAuto = false;
  }
  if (mChooser2.getSelected() == "Yes"){
    Camera2_InAuto = true;
  }
  
  if (mChooser3.getSelected() == "No") {
    Camera3_InAuto = false;
  }
  if (mChooser3.getSelected() == "Yes"){
    Camera3_InAuto = true;
  }

  if (mChooser4.getSelected() == "No") {
    Camera4_InAuto = false;
  }
  if (mChooser4.getSelected() == "Yes"){
   Camera4_InAuto = true;
  }
  
  if (mChooser5.getSelected() == "No") {
   Camera5_InAuto = false;
  }
  if (mChooser5.getSelected() == "Yes"){
    Camera5_InAuto = true;
  }
  
  }
  
}