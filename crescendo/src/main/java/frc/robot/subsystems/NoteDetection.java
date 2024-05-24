// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 import frc.robot.subsystems.SwerveBase;


public class NoteDetection extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */


public static double x;
public static double y;
public static double area;
public static boolean seesTarget;

  public NoteDetection() {
   
}


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.

   // drive.drive(1.0, 0.0, 0.0, true);
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
   
    //System.out.println(y);

NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-notes");
NetworkTableEntry tx = table.getEntry("tx");
NetworkTableEntry ty = table.getEntry("ty");
NetworkTableEntry ta = table.getEntry("ta");

//read values periodically
 x = tx.getDouble(0.0);
 y = ty.getDouble(0.0);
 area = ta.getDouble(0.0);

//post to smart dashboard periodically
SmartDashboard.putNumber("Limelightx", x);
SmartDashboard.putNumber("LimelightY", y);
SmartDashboard.putNumber("LimelightArea", area);

if (area < 0.01){
  seesTarget = false;
  }
  else{
    seesTarget = true;
  }
  }

  
//   public void LimelightinAuto() {
   
//     System.out.println(x);

// NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-notes");
// NetworkTableEntry tx = table.getEntry("tx");
// NetworkTableEntry ty = table.getEntry("ty");
// NetworkTableEntry ta = table.getEntry("ta");

// //read values periodically
//  x = tx.getDouble(0.0);
// double y = ty.getDouble(0.0);
// double area = ta.getDouble(0.0);

// //post to smart dashboard periodically
// SmartDashboard.putNumber("Limelightx", x);
// SmartDashboard.putNumber("LimelightY", y);
// SmartDashboard.putNumber("LimelightArea", area);
 // }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
