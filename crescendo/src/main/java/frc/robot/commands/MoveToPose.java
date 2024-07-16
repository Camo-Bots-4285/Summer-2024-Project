// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LineBreak;
import frc.robot.subsystems.NoteDetection;
import frc.robot.subsystems.SelfDriving;
import frc.robot.subsystems.SwerveBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveBase;
import frc.robot.subsystems.NoteDetection;
import frc.robot.subsystems.SelfDriving;


/** An example command that uses an example subsystem. */
public class MoveToPose extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private SelfDriving m_self_Driving;
 
  private final SwerveBase drive;
  
  


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveToPose(SwerveBase swerveBase) {
drive = swerveBase;
 }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
//drive.drive(0.0, 0.0, SelfDriving.RotationSpeed, true);
//drive.drive(SelfDriving.XSpeed, SelfDriving.YSpeed, SelfDriving.RotationSpeed, true);
drive.drive(0.25, 0.0, 0.0, true);
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   //if (SelfDriving.CloseEnough == false){
        return false;  
    //     }
    
    // else{
    //     return true;
    //     }
    }   
}
