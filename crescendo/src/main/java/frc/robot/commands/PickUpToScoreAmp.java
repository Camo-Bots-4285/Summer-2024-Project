package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPivotConstants;
import frc.robot.subsystems.ArmPivotSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LineBreak;
import frc.robot.subsystems.NoteDetection;
import frc.robot.subsystems.SelfDriving;
import frc.robot.subsystems.ShooterFeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveBase;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;

/**
 * A complex auto command that drives forward, releases a hatch, and then drives backward.
 */
public class PickUpToScoreAmp extends SequentialCommandGroup{
// public static FeedToShoot3 repeatingSequence(){
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem shoot;
  private final ArmPivotSubsystem aim;
  private final ShooterFeederSubsystem feed;
  private final LineBreak breakline;
  private final SwerveBase drive;
  private final SelfDriving autodrive;
  private final IntakeSubsystem intake;
  private final NoteDetection note;
 //FeedToShoot3 repeats = FeedToShoot3.repeatedly();
 //public FeedToShoot3 repeatedly(ShooterSubsystem m_shooter, ArmPivotSubsystem m_ArmPivotSubsystem, ShooterFeederSubsystem m_shooterFeeder,LineBreak m_lineBreak){
  public PickUpToScoreAmp(ShooterSubsystem m_shooter, ArmPivotSubsystem m_ArmPivotSubsystem, 
  ShooterFeederSubsystem m_shooterFeeder,LineBreak m_lineBreak, SwerveBase m_swerveBase,
  SelfDriving m_selfDriving,IntakeSubsystem m_intake, NoteDetection m_note_detection) {
    shoot = m_shooter;
    aim = m_ArmPivotSubsystem;
    feed = m_shooterFeeder;
    breakline = m_lineBreak;
    drive = m_swerveBase;
    autodrive = m_selfDriving;
    intake = m_intake;
    note = m_note_detection;
 addRequirements(shoot,aim,feed,breakline,drive,autodrive,intake,note);
 
 //if (NoteDetection.seesTarget == true & LineBreak.HasNote == false){
            addCommands(
                //Looks for note and gets Note
                new ParallelDeadlineGroup(
                    new MoveToNote(drive),
                    new FloorFeederTest(intake),
                    new ArmPivotErrected(aim),
                    new ShooterFeederPickUp(feed)
                ),
                // Moves to the front of speacker
                new ParallelDeadlineGroup(
                    new MoveToPose(drive),
                    new ArmPivotErrected(aim),
                    new ShooterFeederPickUp(feed)
                ),
                
                new ParallelCommandGroup(
                    new ShooterAMP(shoot),
                    new AMPScoringPos(aim)
                ).withTimeout(0.5),
               // shoots note

                new ParallelCommandGroup(
                    new ShooterAMP(shoot),
                    new AMPScoringPos(aim),
                    new ShooterFeederAMP(feed)   
                ).withTimeout(0.25)
              

            );
        // };

        //  if(LineBreak.HasNote == true){
        //      addCommands(
           
        //     // Moves to the front of the speacker
        //         new ParallelDeadlineGroup(
        //             new MoveToPose(m_swerveBase),
        //             new ShootingWithoutCameras(m_ArmPivotSubsystem),
        //             new ShooterTest(m_shooter)
        //         ),

        //        // 3. Contine Moving to fire postion and revs shooter
        //         new ParallelCommandGroup(
        //             new ShooterTest(m_shooter),
        //             new ShootingWithoutCameras(m_ArmPivotSubsystem),
        //             new ShooterCrossField(shoot),
        //             new ShooterFeederFire(m_shooterFeeder)   
        //         ).withTimeout(0.25)
              

        //     );
        // };
    }
}
  