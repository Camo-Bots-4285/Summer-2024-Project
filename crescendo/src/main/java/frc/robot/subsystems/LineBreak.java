// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.JsonSerializable.Base;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LineBreakConstants;


public class LineBreak extends SubsystemBase {

      public static boolean resting_bottom_bitch_state;
      public static boolean top_toe_hoe_state;
      public static boolean HasNote;
      
      DigitalInput bottom_sensor = new DigitalInput(LineBreakConstants.DIO_BOTTOM_SENSOR);  
      DigitalInput top_sensor = new DigitalInput(LineBreakConstants.DIO_TOP_SENSOR);


  public LineBreak() {

  }
 
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
  return runOnce(
    ()-> {

    });
  }

  @Override
  public void periodic() {
    //System.out.println(LineBreak.HasNote);
    //System.out.println(LineBreakConstants.LINEBREAK_BLOCKED);
    //System.out.println(LineBreakConstants.LINEBREAK_OPEN);
    //System.out.println(top_sensor.get());
    //This method will be called once per scheduler run
    if (bottom_sensor.get() == true){
      HasNote = true;
  }
      else{
      HasNote = false;
  }   
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

  

  public Boolean getBottomState() {
    return bottom_sensor.get();
  }
  
  public Boolean getTopState() {
    return top_sensor.get();
  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
}
