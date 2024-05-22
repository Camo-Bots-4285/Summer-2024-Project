// //Copyed from team 4829
// //
// //


package frc.robot.commands;


import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
<<<<<<< HEAD


public class FeedToShot extends Command {
    

    private final ShooterSubsystem shoot;
    private final ArmPivotSubsystem aim;
    private final ShooterFeederSubsystem feed;
    private final LineBreak breakline;



   // public class FeedToShot extends repeatingSequenceCommandGroup{
  public FeedToShot(ShooterSubsystem m_shooter, ArmPivotSubsystem m_ArmPivotSubsystem, ShooterFeederSubsystem m_shooterFeeder,LineBreak m_lineBreak) {
     shoot = m_shooter;
     aim = m_ArmPivotSubsystem;
     feed = m_shooterFeeder;
     breakline = m_lineBreak;
     
  addRequirements(shoot,aim,feed,breakline);
      }

      @Override
      public void execute () {
    new SequentialCommandGroup(
        
        // 1. intake till linebreck is broken
        new ParallelDeadlineGroup(
            new HasNote(breakline),
            new ArmPivotFarHumanFeeder(aim),
            new ShootingFarHumanFeeder(shoot)
=======
/**
 * This class is for the 5 Ball Auto Command
 */

public class FeedToShot extends SequentialCommandGroup {
   // public class FeedToShot extends repeatingSequenceCommandGroup{
  public FeedToShot(ShooterSubsystem m_shooter, ArmPivotSubsystem m_ArmPivotSubsystem,
      IntakeSubsystem m_intake, ShooterFeederSubsystem m_shooterFeeder,SwerveBase m_swerveBase,LineBreak m_lineBreak) {

     addCommands(
        
        // 1. intake till linebreck is broken
        new ParallelDeadlineGroup(
            new HasNote(m_lineBreak),
            new ArmPivotFarHumanFeeder(m_ArmPivotSubsystem),
            new ShootingFarHumanFeeder(m_shooter)
>>>>>>> 69ecc61cc605f7b726e728ef9956c8542595dd05
        ),
        
        // 2. Then continues intaking while moving to shot position
        new ParallelCommandGroup(
<<<<<<< HEAD
            new ShootingWithoutCameras(aim),
            new ShootingFarHumanFeeder(shoot)
=======
            new ShootingWithoutCameras(m_ArmPivotSubsystem),
            new ShootingFarHumanFeeder(m_shooter)
>>>>>>> 69ecc61cc605f7b726e728ef9956c8542595dd05
        ).withTimeout(0.5),

        // 3. Contine Moving to fire postion and revs shooter
        new ParallelCommandGroup(
<<<<<<< HEAD
            new ShootingWithoutCameras(aim),
            new ShooterCrossField(shoot)
=======
            new ShootingWithoutCameras(m_ArmPivotSubsystem),
            new ShooterCrossField(m_shooter)
>>>>>>> 69ecc61cc605f7b726e728ef9956c8542595dd05
        ).withTimeout(0.5),
       
        // 4. Shots Note
        new ParallelCommandGroup(
<<<<<<< HEAD
            new ShootingWithoutCameras(aim),
            new ShooterCrossField(shoot),
            new ShooterFeederFire(feed)
        ).withTimeout(0.25)
    );
   
      }

  
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

}
=======
            new ShootingWithoutCameras(m_ArmPivotSubsystem),
            new ShooterCrossField(m_shooter),
            new ShooterFeederFire(m_shooterFeeder)
        ).withTimeout(0.25)
    );
 

}
  
}
>>>>>>> 69ecc61cc605f7b726e728ef9956c8542595dd05
