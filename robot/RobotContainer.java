package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.SubSystem.Drive;
import frc.robot.SubSystem.Vision;
import frc.robot.SubSystem.BracinSub;
import frc.robot.command.Auto.AutonomousCommand;
import frc.robot.command.Drive.Loc;
import frc.robot.command.Drive.PidCommand;

public class RobotContainer {

  // Subsystems
  public final BracinSub baby = new BracinSub();
  private final Drive driveSubsystem = new Drive(baby);

  // Input
  public final Joystick joyDeliciu = new Joystick(Constants.joy);
  public final Joystick JoyDelicioso = new Joystick(Constants.JoyDelicioso);

  // Commands
  private final Loc locCommand;
  
  private final AutonomousCommand auto;

  private final Vision vision = new Vision(driveSubsystem,Constants.targetArea);

  private final PidCommand Pdiddy;
  


  public RobotContainer() { 

    Pdiddy = new PidCommand(baby,JoyDelicioso);

    CommandScheduler.getInstance().registerSubsystem(driveSubsystem);

    // Initialize Loc command with drive subsystem and joystick
    auto = new AutonomousCommand(driveSubsystem,vision,Constants.targetArea);

    locCommand = new Loc(driveSubsystem,joyDeliciu,baby,vision,JoyDelicioso);

    // Set default command
    driveSubsystem.setDefaultCommand(locCommand);

    baby.setDefaultCommand(Pdiddy);

  }

  public Command getAutonomousCommand(){
      return auto;
  
  }

  public Command getBracinCommand(){
    return Pdiddy;
  }

}
