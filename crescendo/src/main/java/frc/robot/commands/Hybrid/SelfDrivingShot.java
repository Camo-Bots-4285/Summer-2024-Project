package frc.robot.commands.Hybrid;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPivotConstants;
import frc.robot.subsystems.ArmPivotSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LineBreak;
import frc.robot.subsystems.NoteDetection;
import frc.robot.subsystems.SelfDriving;
import frc.robot.subsystems.ShooterFeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.commands.ArmPivot.ArmPivotErrected;
import frc.robot.commands.ArmPivot.ShootingWithoutCameras;
import frc.robot.commands.Hybrid.LineBreakArmPivot.ReadyToShoot;
import frc.robot.commands.Hybrid.LineBreakFullShooter.HasNote1;
import frc.robot.commands.Hybrid.LineBreakFullShooter.HasNote2;
import frc.robot.commands.Intake.FloorFeederTest;
import frc.robot.commands.ShooterFeederWheels.ShooterFeederFire;
import frc.robot.commands.ShooterFeederWheels.ShooterFeederPickUp;
import frc.robot.commands.ShooterWheels.ShooterTest;
import frc.robot.commands.Swerve.LaneLogicIn;
import frc.robot.commands.Swerve.LaneLogicOut;
import frc.robot.commands.Swerve.MoveToNote;
import frc.robot.commands.Swerve.MoveToPose;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * A complex auto command that drives forward, releases a hatch, and then drives backward.
 */
public class SelfDrivingShot extends SequentialCommandGroup{
// public static FeedToShoot3 repeatingSequence(){
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem shoot;
  private final ArmPivotSubsystem aim;
  private final ShooterFeederSubsystem feed;
  private final LineBreak breakline;
  private final SelfDriving selfdrive; 
  private final SwerveBase drive;
  private final IntakeSubsystem intake;
 //FeedToShoot3 repeats = FeedToShoot3.repeatedly();
 //public FeedToShoot3 repeatedly(ShooterSubsystem m_shooter, ArmPivotSubsystem m_ArmPivotSubsystem, ShooterFeederSubsystem m_shooterFeeder,LineBreak m_lineBreak){
  public SelfDrivingShot(ShooterSubsystem m_shooter, ArmPivotSubsystem m_ArmPivotSubsystem, ShooterFeederSubsystem m_shooterFeeder,
  LineBreak m_lineBreak, SelfDriving m_selfDriving, SwerveBase m_swerveBase,IntakeSubsystem m_intake) {
    shoot = m_shooter;
    aim = m_ArmPivotSubsystem;
    feed = m_shooterFeeder;
    breakline = m_lineBreak;
    selfdrive = m_selfDriving;
    drive = m_swerveBase;
    intake = m_intake;
 addRequirements(shoot,aim,feed,breakline,selfdrive,drive,intake);
   
    addCommands(

        new ParallelDeadlineGroup(
            new ParallelRaceGroup(
                new SeesNote(m_lineBreak), 
                new HasNote1(m_lineBreak)
            ),
        
        new RunCommand(() -> selfdrive.setTargetPose(10)),
        new LaneLogicOut(drive) 
        ),

       new ParallelDeadlineGroup(
                new HasNote1(m_lineBreak),
                new MoveToNote(m_swerveBase),
                new FloorFeederTest(intake), 
                new ShooterFeederPickUp(feed),
                new ArmPivotErrected(m_ArmPivotSubsystem)
            ),
  
        //The following Group auto picks up note
            new ParallelDeadlineGroup(
                new CloseEnough(m_lineBreak),
                new LaneLogicIn(drive),
                new RunCommand(() -> selfdrive.setTargetPose(10)),
                //new ShooterFeederPickUp(feed),
                new ShootingWithoutCameras(aim)
            ),

        // The following sequence is to drive back and shot the note
            // new ParallelDeadlineGroup(
            //     new ReadyToShoot(m_lineBreak),
            //     new ShootingWithoutCameras(aim)
            // ),

             new ParallelCommandGroup(
                new ShootingWithoutCameras(aim),
                new ShooterTest (m_shooter)
            ).withTimeout(.5),

              new ParallelCommandGroup(
                new ShootingWithoutCameras(aim),
                new ShooterTest (m_shooter),
                new ShooterFeederFire(m_shooterFeeder)
            ).withTimeout(0.25)




           
      
    );
   
            }

     
  }
  