// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swerve.SwerveDrive;

public class driveToTag extends Command {
  private SwerveDrive swerve;

  /** Creates a new driveToTag. */
  public driveToTag(SwerveDrive swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // Calculate the angular velocity based on Limelight aiming (proportional control)
    double rot = limelight_aim_proportional();

    // Calculate the forward speed based on Limelight ranging (proportional control)
    double xSpeed = limelight_range_proportional();

    // You can choose whether to drive field-relative or not (typically false for autonomous)
    boolean fieldRelative = false;

    // Drive the swerve drive system
    swerve.localizationDrive(xSpeed, 0.0, rot, fieldRelative, 0.02);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Simple proportional turning control with Limelight
    private double limelight_aim_proportional() {
        double kP = .035;  // Proportional constant
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight-shahzhu") * kP;

        // Convert to radians per second for our drive method
        targetingAngularVelocity *= Constants.SwerveConstants.maxChassisAngularVelocity;

        // Invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }

    // Simple proportional ranging control with Limelight's "ty" value
    private double limelight_range_proportional() {
        double kP = .1;  // Proportional constant
        double targetingForwardSpeed = LimelightHelpers.getTY("limelight-shahzhu") * kP;

        targetingForwardSpeed *= Constants.SwerveConstants.maxChassisTranslationalSpeed;
        targetingForwardSpeed *= -1.0;

        return targetingForwardSpeed;
    }
}

