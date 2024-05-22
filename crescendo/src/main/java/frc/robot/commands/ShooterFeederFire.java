// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ArmPivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ArmPivotSubsystem;
import frc.robot.subsystems.ShooterFeederSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

//private int active_mode;

/** An example command that uses an example subsystem. */
public class ShooterFeederFire extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterFeederSubsystem m_subsystem;
  public static ArmPivotSubsystem m_ArmPivotSubsystem;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterFeederFire(ShooterFeederSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if (MODE == ArmPivotConstants.POSITION_STARTING ) { m_subsystem.shoot(0.075, true);}
    //if else {}
   // Was working on makeing one command for shooter feeder speed
    m_subsystem.shoot(1.00, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
