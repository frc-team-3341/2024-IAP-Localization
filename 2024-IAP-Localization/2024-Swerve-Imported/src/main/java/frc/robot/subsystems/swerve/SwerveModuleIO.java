package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Creates a SwerveModuleIO interface. This is extendable and is used for
 * different module types and simulations.
 * 
 * @author Aric Volman
 */
public interface SwerveModuleIO {
   
   /**
    * Gets number of module
    */
    default int getNum() {
      return 0;
   }

   /**
    * Set swerve module's state in m/s and radians.
    * 
    * @param state State of module
    */
   default void setDesiredState(SwerveModuleState state) {
   }

   // /**
   // * Sets the desired state for the module.
   // *
   // * @param desiredState Desired state with speed and angle.
   // */
   // public void setDesiredState(SwerveModuleState desiredState) {
   //    var encoderRotation = new Rotation2d(m_turningEncoder.getDistance());

   //    // Optimize the reference state to avoid spinning further than 90 degrees
   //    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

   //    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
   //    // direction of travel that can occur when modules change directions. This results in smoother
   //    // driving.
   //    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

   //    // Calculate the drive output from the drive PID controller.
   //    final double driveOutput =
   //       m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

   //    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

   //    // Calculate the turning motor output from the turning PID controller.
   //    final double turnOutput =
   //       m_turningPIDController.calculate(m_turningEncoder.getDistance(), state.angle.getRadians());

   //    final double turnFeedforward =
   //       m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

   //    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
   //    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
   // }

   /**
    * Gets swerve module's set state
    */
   default SwerveModuleState getDesiredState() {
      return null;
   }

   /**
    * Gets swerve module's real state
    */
   default SwerveModuleState getActualModuleState() {
      return null;
   }

   /**
    * Sets voltage of swerve module's drive motor (-12.0 to 12.0).
    * 
    * @param volts Voltage to set
    */
   default void setDriveVoltage(double volts) {
   }

   /**
    * Sets voltage of swerve module's turn motor (-12.0 to 12.0).
    * 
    * @param volts Voltage to set
    */
   default void setTurnVoltage(double volts) {
   }

   /**
    * Sets brake mode of swerve module's drive motor.
    * 
    * @param enable Enables brake mode if true
    */
   default void setDriveBrakeMode(boolean enable) {
   }

   /**
    * Sets brake mode of swerve module's turn motor.
    * 
    * @param enable Enables brake mode if true
    */
   default void setTurnBrakeMode(boolean enable) {
   }

   /**
    * Gets swerve module's position as an object.
    */
   default SwerveModulePosition getPosition() {
      return null;
   }

   /**
    * Gets turn position in radians
    */
    default double getTurnPositionInRad() {
      return 0.0;
   }

   /**
    * Resets encoders of swerve drive motor.
    */
   default void resetEncoders() {
   }

   /** Updates all module telemetry. NOTE: please create variables for everything you want to update and display them in this method. */
   default void updateTelemetry() {
   }

   /**
    * Updates simulation motors by a constant dt (0.02)
    */
   default void updateSim() {
   }
}