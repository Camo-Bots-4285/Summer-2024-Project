// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.VisionConstants.FIELD_LENGTH_METERS;
import static frc.robot.Constants.VisionConstants.FIELD_WIDTH_METERS;

import org.ejml.equation.ManagerFunctions.Input1;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.RobotContainer;
import frc.robot.subsystems.NoteDetection;
import frc.robot.subsystems.SelfDriving;
import frc.robot.subsystems.SwerveBase;

 

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
 public final class Constants {
  public static boolean isRed= true;

  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
 
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class AprilTagConstants {
  }

  public static class ArmPivotConstants {
    /*
     * These numbers must match the Spark connecting to this motor.
     */

    // 2 NEO Motor
    public static final int MOTOR_ARM_PIVOT_A = 20;
    public static final int MOTOR_ARM_PIVOT_B = 21;

    // PID Locations
     public static final int POSITION_STARTING = 0;
    public static final int POSITION_INTAKE_FLOOR = 1;
    public static final int POSITION_INTAKE_FEEDER = 2;
    public static final int POSITION_TRAVEL = 3;
    public static final int POSITION_AMP_SCORING = 4;
    public static final int POSITION_SHOOTING = 5;
    public static final int POSITION_HUMAN_FEEDER = 6;
    public static final int POSITION_STORE = 7;
    public static final int POSITION_ERRECTED = 8;
    public static final int POSITION_SHOOTING_WITHOUT_CAMERAS = 9;
    public static final int POSITION_SHOOTING_WITHOUT_CAMERAS_STAGE_LEG = 10;
    public static final int POSITION_SHOOTING_WITHOUT_CAMERAS_2ND_STAGE_LEG = 11;
    public static final int POSITION_AMP_SCORING_POS = 12;
    public static final int POSITION_SHOOTING_WITHOUT_CAMERAS_N1 = 13;
    public static final int POSITION_SHOOTING_DEFENCE = 14;
    public static final int POSITION_LINE_SCORING = 15;
    public static final int POSITION_TRAP_SCORING = 16;
    public static final int POSITION_FAR_FEEDER = 17;

    public static final double POSITION_PID_STARTING = 0;// Robot Will go here on start
    public static final double POSITION_PID_INTAKE_FLOOR = POSITION_STARTING;
    public static final double POSITION_PID_INTAKE_FEEDER = 1.31;
    public static final double POSITION_PID_AMP_SCORING = -2.3333;
    public static final double POSITION_PID_HUMAN_FEEDER = -2.499999; //-2.595237 //-2.666665
    //-2.499999 Far Human Feeder
    public static final double POSITION_PID_STORE = 6.214284;
    public static final double POSITION_PID_ERRECTED = 0;
    public static final double POSITION_PID_SHOOTING_WITHOUT_CAMERAS =  2.1666 ;//2.1666   2.5   2.833331  2.880950
    public static final double POSITION_PID_SHOOTING_WITHOUT_CAMERAS_2ND_STAGE_LEG = 3.41; 
    public static final double POSITION_PID_SHOOTING_WITHOUT_CAMERAS_STAGE_LEG = 3.15;
    public static final double POSITION_PID_AMP_SCORING_POS = -1.9; //-1.738;
    public static final double POSITION_PID_SHOOTING_WITHOUT_CAMERAS_N1 = 3.25; //1.07126
    public static final double POSITION_PID_SHOOTING_DEFENCE = .1;
    public static final double POSITION_PID_LINE_SCORING = 2.7;//2.85
    public static final double POSITION_PID_SHOOTING_TRAP = 1.761905;
    public static final double POSITION_PID_FAR_FEEDER = 2.833331;

    // 36 inch -- 1.31
    // 42 inch -- 1.336
    // 48 inch -- 1.37
    // 54 inch -- 1.35
    // 60 inch -- 1.36
    // 66 inch -- 1.382
    // 72 inch -- 1.388
    // 78 inch -- 1.395
    // 84 inch -- 1.42
    // 90 inch -- 1.44
    // 96 inch -- 1.45
    // 102 inch -- 1.47
    // 108 inch -- 1.48
    // 114 inch -- 1.48
    // 120 inch -- 1.485
    // 126 inch -- 1.49
    // 132 inch -- 1.495
    // 138 inch -- 1.495
    // 144 inch -- 1.50
    // 150 inch -- 1.50
    // 156 inch -- 1.505
    // 162 inch -- 1.51
    // 168 inch -- 1.515
    // 174 inch -- 1.53
    // 180 inch -- 1.5375
    // 186 inch -- 1.5425
    // 192 inch -- 1.54525
    // 198 inch -- 1.54
    // 204 inch -- 1.535
    // 216 inch --1.5375 This value shoot good disk high and bad disk low

    // public static final double POSITION_PID_INTAKE_FEEDER = 1.4;
    public static final double POSITION_PID_TRAVEL = 0.22;

  }

  public static class ClimberConstants {
    /*
     * These numbers must match the Spark connecting to this motor.
     */
  }

  public static class IntakeConstants {
    /*
     * These numbers must match the Spark connecting to this motor.
     */

    // NEO Motor
    public static final int MOTOR_INTAKE_FLOOR = 13;
    public static final int MOTOR_INTAKE_FLOOR2 = 14;

  }

  public static class ShooterConstants {
    /*
     * These numbers must match the Spark connecting to this motor.
     */

    // Falcon Motor
    public static final int MOTOR_SHOOTER_A = 15;

    // Falcon Motor
    public static final int MOTOR_SHOOTER_B = 16;
  }

  public static class ShooterFeederConstants {
    /*
     * These numbers must match the Spark connecting to this motor.
     */

    // NEO Motor
    public static final int MOTOR_SHOOTER_FEEDER = 11;

    public static final int SHOOTER_FEEDER_STATE_FLOOR_FEEDING = 0;
    public static final int SHOOTER_FEEDER_STATE_HUMAN_FEEDING = 1;
    public static final int SHOOTER_FEEDER_REVERSE = 3;
    public static final int SHOOTER_FEEDER_FAR_FEEDER = 4;

  }

  public static final class SwerveConstants {
    /* Drive Controls */
    public static final int translationAxis = 1;
    public static final int strafeAxis = 0;
    public static final int rotationAxis = 2; // was 4 on Xbox
    public static final int sliderAxis = 3;

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(21.5); // measured from center of each module
    public static final double wheelBase = Units.inchesToMeters(21.5);

    // nominal (real) divided by fudge factor
    public static final double wheelDiameter = Units.inchesToMeters(4.0 / 1.0); // was 1.04085   was 4
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double driveGearRatio = 5.14; // Mk4 drive ratio
    public static final double angleGearRatio = 12.8; // Mk4 steer ratio (does this need encoder stuff??)

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0), // front left, ++ quadrant
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // front right, +- quadrant
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // rear left, -+ quadrant
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // rear right, -- quadrant
    );

    public static Translation2d mDriveRadius = new Translation2d(trackWidth/2, wheelBase / 2);

    //Swerve Drive Calibration Factor used to make robot go commanded speed should be tuned with each bot
    //Find the value by deviding the actual speed by the commanded speed recommend doing multiple test with lower speeds
    //Units are meter per second
    public static double calibrationFactorSB = 1.5;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 6.03504;//In meter/Second 
    public static final double maxAngularVelocity = 2.0;//Is in radian

    public static final int frontLeftRotationMotorId = 2;//21
    public static final int frontLeftDriveMotorId = 1;//11

    public static final int frontRightRotationMotorId = 8;//24
    public static final int frontRightDriveMotorId = 7;//14

    public static final int rearLeftRotationMotorId = 4;//22
    public static final int rearLeftDriveMotorId = 3;//12

    public static final int rearRightRotationMotorId = 6;//23
    public static final int rearRightDriveMotorId = 5;//13

    public static final int frontLeftRotationEncoderId = 1;
    public static final int frontRightRotationEncoderId = 4;
    public static final int rearLeftRotationEncoderId = 2;
    public static final int rearRightRotationEncoderId = 3;

    public static  double kTeleDriveMaxSpeedMetersPerSecond;  
    public static double kTeleDriveMaxAccelerationUnitsPerSecond = 1.5;
    
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 3.1415/4;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3.0;

    public static final int PIGEON_SENSOR_ID = 0;

  }

  public static final class VisionConstants {

    //TODO Calculate and get these values, Units need to be in meters and radians
    /**
     * Physical location of the apriltag camera on the robot, relative to the center
     * of the robot.
     */

    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_1 = new Transform3d(
        new Translation3d(-0.063, -0.3125, 0.562),// Get from CAD Model In meters-0.063, -0.3125, 0.562
        new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(-45.0)));

    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_2 = new Transform3d(
        new Translation3d(0.063, -0.3125, 0.562),//0.063, -0.3125, 0.562
        new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(-135)));

    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_3 = new Transform3d(
        new Translation3d(0.063, 0.3125, 0.562),//0.063, 0.3125, 0.562
        new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(135)));

    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_4 = new Transform3d(
        new Translation3d(-0.063, 0.3125, 0.562),//-0.063, 0.3125, 0.562
        new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(45)));
    
    //Lime Light 
    //          CAD             RealLife   OldVaules 
    // Height  0.6673348        0.684      0.669
    //Front    0.792804         0.072      0.059
    //Right    0.3028135        0.311      0.303
    //Angle    29.7169722 Deg   30 Deg     30 deg
    // public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_4 = new Transform3d(
    //     new Translation3d(0.3109483, 0.0631137, -0.567547072),
    //     new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(-135.0)));
    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_5 = new Transform3d(
        new Translation3d(0.072, 0.311, 0.669),//0.072, -0.311, 0.669
        new Rotation3d(30.0, Units.degreesToRadians(0.0), Units.degreesToRadians(0)));
    
        //Main Note Camera Assumed to be center 
     public static final Transform3d Note_Camera_Main_To_Robot = new Transform3d(
        new Translation3d(0.18, 0.0, 0.38),//0.072, -0.311, 0.669
        new Rotation3d(Units.degreesToRadians(0.0),15.0, Units.degreesToRadians(0.0)));

      public static final Transform3d Note_Camera_Assistant1_To_Robot = new Transform3d(
        new Translation3d(0.072, 0.311, 0.669),//0.072, -0.311, 0.669
        new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(0.0), Units.degreesToRadians(0.0)));

      public static final Transform3d Note_Camera_Assistant2_To_Robot = new Transform3d(
        new Translation3d(0.072, 0.311, 0.669),//0.072, -0.311, 0.669
        new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(0.0), Units.degreesToRadians(0.0)));
    
    
    public static final double FIELD_LENGTH_METERS = 16.542;
    public static final double FIELD_WIDTH_METERS = 8.2042;

    public static final Pose2d FLIPPING_POSE = new Pose2d(
      new Translation2d(FIELD_LENGTH_METERS , FIELD_WIDTH_METERS),
      new Rotation2d(Math.PI)
    );

    

    /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
    //Raise this value to reject less accurate poses 0.2 recommended by photon vision
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
  }

  public static class LEDConstants {
      public static final int DIO_LED_IS_BLUE = 4;
      public static final int DIO_LED_IS_RED = 5;
      public static final int DIO_LED_WIN = 6;

      // public static int DIO_One = 1;
      // public static int DIO_Ten = 2;
      // public static int DIO_Hundred = 3;
      // public static int DIO_Thousand = 4;
      public static final Boolean DIO_ENABLE = false;
      public static final Boolean DIO_DISABLE = true;

      
    }


public static class LineBreakConstants {

    public static int DIO_BOTTOM_SENSOR = 8;
    public static int DIO_TOP_SENSOR = 7;

    public static final boolean LINEBREAK_BLOCKED = false;
    public static final boolean LINEBREAK_OPEN = true;
  }

  public static class BuildConstants {
    public static final String MAVEN_GROUP = "";
    public static final String MAVEN_NAME = "crescendo";
    public static final String VERSION = "unspecified";
    public static final int GIT_REVISION = 6;
    public static final String GIT_SHA = "cc013129e3c7396cb75b6440b939e763719c4f9a";
    public static final String GIT_DATE = "2024-05-28 11:17:52 EDT";
    public static final String GIT_BRANCH = "main";
    public static final String BUILD_DATE = "2024-07-16 15:15:26 EDT";
    public static final long BUILD_UNIX_TIME = 1721157326492L;
    public static final int DIRTY = 1;
  }

 }
