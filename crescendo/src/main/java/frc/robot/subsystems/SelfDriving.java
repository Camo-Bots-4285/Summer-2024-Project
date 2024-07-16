// ToDo Get pose of amp and speacker on each side
//ToDo  optimise roation like a serve module
//ToDo Test to see if Order of subtraction is right
//ToDo make command to run
//ToDo tune PIDs for smoother travel


//Test roation first 
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.google.flatbuffers.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveBase;

public class SelfDriving extends  SubsystemBase{
   
public static double TagetPoseX;
public static double TagetPoseY;
public static double TagetPoseRotation;

// public static double currentPoseX;
// public static double currentPoseY;
//public static double currentPoseRotation;

public static double PoseDifferenceX;
public static double PoseDifferenceY;
public static double PoseDifferenceRotation;
public static double PoseDifferenceRotationRaw;

public static double XSpeed;
public static double YSpeed;
public static double RotationSpeed;

public static double MaxTranslationSpeed = 0.055;//0.055
public static double MaxRotationSpeed = 0.1;

public static boolean CloseEnough;
public static double HowFar;
public static double HowStraght;
public static boolean TranslationClose;
public static boolean RotonalClose;

    // PIDs to Contol Drivig
public static PIDController NoteRotation = new PIDController(0.0025, 0, 0);
public static PIDController NoteTranslation = new PIDController(0.0025, 0, 0);
public static PIDController AutoNoteRotation = new PIDController(0.025, 0, 0);
public static PIDController AutoNoteTranslation = new PIDController(0.0175, 0, 0);
    
public static PIDController MoveToPoseTranslation = new PIDController(5.0, 0, 0);
public static PIDController MoveToPoseRotation = new PIDController(0.0025, 0, 0.0);


public void setTargetPose(int set) {
if (set == 0){}

//Shotting Infront of Speacker
if (set == 1){ 
    //Pose and Angle For Red
    if (RobotContainer.isRed == true){
    TagetPoseX = 15.0;
    TagetPoseY = 5.5;
    TagetPoseRotation = 0.0;
        }
    //Pose and Angle For Blue
    if (RobotContainer.isRed == false){
    TagetPoseX = 1.275;
    TagetPoseY = 5.5;
    TagetPoseRotation = 0.0;
        }
    }
//Amp Pose 
if (set == 2){ 
    //Pose and Angle For Red
    if (RobotContainer.isRed == true){
    TagetPoseX = 14.7;
    TagetPoseY = 7.6;
    TagetPoseRotation = 90.0;//270
        }
    //Pose and Angle For Blue
    if (RobotContainer.isRed == false){
    TagetPoseX = 1.80;
    TagetPoseY = 7.60;
    TagetPoseRotation = 90;
        }

    }
}
   
public void DriveCalculation() {



}



    @Override
  public void periodic() {
    //currentPoseX = SwerveBase.m_pose.getX();
    // currentPoseX = getPose().getX();
    // currentPoseY = SwerveBase.m_pose.getY() * SwerveBase.SCALE_Y;
    //SwerveBase.currentPoseRotation = SwerveBase.pigeonSensor.getYaw();   

     
    //Because we are driving in feild centric for this command the number will change signs depeding on allaince
   if (RobotContainer.isRed == true){
        PoseDifferenceX = TagetPoseX - SwerveBase.currentPoseX;
        PoseDifferenceY = TagetPoseY - SwerveBase.currentPoseY;// Need to test might not be right
        PoseDifferenceRotationRaw =  SwerveBase.currentPoseRotation - TagetPoseRotation; // Need to test might not be right   
    }
        if (RobotContainer.isRed == false){
        PoseDifferenceX = SwerveBase.currentPoseX - TagetPoseX;
        PoseDifferenceY = SwerveBase.currentPoseY - TagetPoseY;// Need to test might not be right
        PoseDifferenceRotationRaw = SwerveBase.currentPoseRotation - TagetPoseRotation;// Need to test might not be right
    }
//Roation should be the same reguardless of alliance
//The following will optimize the rotation to go left or right
        if (PoseDifferenceRotationRaw  > 180){
        PoseDifferenceRotation = PoseDifferenceRotationRaw - 360;
    }
        else {
        PoseDifferenceRotation =  PoseDifferenceRotationRaw;
    }

    XSpeed = MoveToPoseTranslation.calculate(PoseDifferenceX , 0.0);
    YSpeed = MoveToPoseTranslation.calculate(PoseDifferenceY, 0.0);
    RotationSpeed = MoveToPoseRotation.calculate(PoseDifferenceRotation, 0.0);

 // Sets a max out put for variable
    XSpeed =  XSpeed < MaxTranslationSpeed ?  XSpeed : MaxTranslationSpeed;
    YSpeed =  YSpeed < MaxTranslationSpeed ?  YSpeed : MaxTranslationSpeed;
    RotationSpeed =  RotationSpeed < MaxRotationSpeed ?  RotationSpeed : MaxRotationSpeed; 

    // When CloseEnough == Fasle MoveToPose will end
    // This is where you will specify hoe close it must be for what you are doing to work


    // HowFar = PoseDifferenceY * PoseDifferenceY + PoseDifferenceX * PoseDifferenceX;
    //  HowStraght = Math.abs(PoseDifferenceRotation);


    //  if(HowFar < 0.05){
    //     TranslationClose = true;
    //  }
    //  else{ TranslationClose = false;}
     
    //  if(HowStraght < 2.5){
    //     RotonalClose = true;
    //  }
    //  else{ RotonalClose = false;}


    if(TranslationClose == true && RotonalClose == true ){//&&  RotonalClose == true){
        CloseEnough = true;
    }
    else{
        CloseEnough = false;
    }

    //  System.out.println(SwerveBase.currentPoseX);  
    //  System.out.println(SwerveBase.currentPoseY);  
       

  }
}
