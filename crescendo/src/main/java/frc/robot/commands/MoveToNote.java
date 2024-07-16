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


/** An example command that uses an example subsystem. */
public class MoveToNote extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private NoteDetection m_note_Detection;
 
  private final SwerveBase drive;
  public static double NoteSpeed;
  


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveToNote(SwerveBase swerveBase) {
drive = swerveBase;
 }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //  if (NoteDetection.x > 2.5 & NoteDetection.x < 4.0){
  //   drive.drive(0.02, 0.0, -0.002, false);
  //  }
  //   if (NoteDetection.x < -2.5 & NoteDetection.x > -4.0){
  //   drive.drive(0.02, 0.0, 0.002, false);
  //   }
  //   if (NoteDetection.x > 4.0){
  //   drive.drive(0.0, 0.0, -0.035, false);
  //  }
  //   if (NoteDetection.x < -4.0){
  //   drive.drive(0.0, 0.0, 0.035, false);
  //   }
  //   if (NoteDetection.x > -2.5 & NoteDetection.x < 2.5){
  //   drive.drive(0.07, 0.0, 0.0, false);
  //  }
  //  if (NoteDetection.x > -0.01 & NoteDetection.x < 0.01){
  //   drive.drive(0.0, 0.0, 0.0, false);
  //  }


//drive.drive((SwerveBase.NoteTranslation.calculate(NoteDetection.y, -20.0)), 0.0, (SwerveBase.NoteRotation.calculate(NoteDetection.x, 0.0)), false);


NoteSpeed = SelfDriving.AutoNoteTranslation.calculate(NoteDetection.y, 0.0);
NoteSpeed = NoteSpeed < 0.015 ? NoteSpeed : 0.015; 
if (NoteDetection.seesTarget == true){
drive.drive(-NoteSpeed, 0.0, (SelfDriving.AutoNoteRotation.calculate(NoteDetection.x, 0.0)), false);
//(what goes here).set(SwerveBase.NoteRotation.calculate(NoteDetection.x, 0.0));
//(what goes here).set(SwerveBase.NoteTranslation.calculate(NoteDetection.x, 0.0));
}
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if (LineBreak.HasNote == true){
        return false;
    }
    else{
        return true;
    }
  }

}
