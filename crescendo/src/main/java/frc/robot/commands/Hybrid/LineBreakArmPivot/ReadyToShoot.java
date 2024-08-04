// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hybrid.LineBreakArmPivot;

import frc.robot.Constants.ArmPivotConstants;
import frc.robot.Constants.LineBreakConstants;
import frc.robot.subsystems.ArmPivotSubsystem;
import frc.robot.subsystems.LEDSubsystem;
//import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LineBreak;
import frc.robot.Constants.LEDConstants;
/** An example command that uses an example subsystem. */
public class ReadyToShoot extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LineBreak m_subsystem;

 private static double AngleToShoot;
 public static boolean ReadyToShoot;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ReadyToShoot(LineBreak subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  AngleToShoot = ArmPivotSubsystem.ArmAngle - 2.7;
  if(Math.abs(AngleToShoot)<0.1){
ReadyToShoot = true;
  }
  else{
    ReadyToShoot = false;
  }
  System.out.println("sup");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
  }

 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (ReadyToShoot == true){
        return true;
    }
    else{
        return false;
    }
  }
}
