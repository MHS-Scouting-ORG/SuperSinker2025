package frc.robot.commands.SwerveCmds;

import java.util.List;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveFollowTagCmd extends Command {

  private CommandSwerveDrivetrain swerveSubs; 
  private DoubleSupplier xSupp, ySupp, zSupp;
  private PhotonCamera rightArducam; 
  private List<PhotonPipelineResult> unreadResultList; 
  // private PhotonPipelineResult result; 
  private PIDController turningPid; 

  /** Creates a new SwerveFollowTagCmd. */
  public SwerveFollowTagCmd(CommandSwerveDrivetrain swerveSubs, DoubleSupplier xSupp, DoubleSupplier ySupp, DoubleSupplier zSupp) {
    this.swerveSubs = swerveSubs; 
    this.xSupp = xSupp; 
    this.ySupp = ySupp; 

    rightArducam = new PhotonCamera("Arducam_OV9281_USB_Camera");
    turningPid = new PIDController(0, 0, 0); 
    turningPid.setTolerance(2);

    addRequirements(swerveSubs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // unreadResultList = rightArducam.getAllUnreadResults(); 

    // if (!unreadResultList.isEmpty()) {
    //   PhotonPipelineResult result = unreadResultList.get(unreadResultList.size() - 1); 

    //   if (result.hasTargets()) {
    //     if (Driver)
    //   }
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
