// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class setArmPositionCmd extends Command {

   private final Shooter m_shooter;

   /** Creates a new SetArmPositionCmd. */
   public setArmPositionCmd(Shooter shooter) {
      m_shooter = shooter;

      addRequirements(m_shooter);
      // Use addRequirements() here to declare subsystem dependencies.
   }

   // Called when the command is initially scheduled.
   @Override
   public void initialize() {
      m_shooter.setArmPosition(ArmConstants.ARM_CLOSE_SHOT_SETPOINT);
   }

   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute() {
   }

   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
   }

   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
      return true;
   }
}
