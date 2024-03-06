// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.lang.Math;
import java.lang.constant.Constable;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class climbByXbox extends Command {
   Climber m_climber;
   BooleanSupplier m_unlock;
   DoubleSupplier m_rightSpeed;
   DoubleSupplier m_leftSpeed;
   BooleanSupplier m_lBump;
   BooleanSupplier m_rBump;

   /** Creates a new climbByXbox. */
   public climbByXbox(Climber climber, BooleanSupplier unlock, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed, BooleanSupplier lBump, BooleanSupplier rBump) {
      m_climber = climber;
      m_unlock = unlock;
      m_rightSpeed = rightSpeed;
      m_leftSpeed = leftSpeed;
      m_lBump = lBump;
      m_rBump = rBump;
      addRequirements(m_climber);
      // Use addRequirements() here to declare subsystem dependencies.
   }

   // Called when the command is initially scheduled.
   @Override
   public void initialize() {
   }

   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute() {
      if (m_unlock.getAsBoolean())
         m_climber.unlockBothClimbers();

      double lSpeed = 0.0;
      double rSpeed = 0.0;

      if (!m_rBump.getAsBoolean())
      {
         rSpeed = m_rightSpeed.getAsDouble();
      }
      else
      {
         rSpeed = Constants.ClimberConstants.CLIMBER_UP_SPEED;
      }
  

      if (!m_lBump.getAsBoolean())
      {
         lSpeed = m_leftSpeed.getAsDouble();
      }
      else
      {
         lSpeed = Constants.ClimberConstants.CLIMBER_UP_SPEED;
      }


      SmartDashboard.putNumber("Climber R Spd", rSpeed);
      SmartDashboard.putNumber("Climber L Spd", lSpeed);

      m_climber.moveClimber(rSpeed, true);
      m_climber.moveClimber(lSpeed, false);
   }

   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
      m_climber.moveClimber(0.0, true);
      m_climber.moveClimber(0.0, false);
   }

   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
      return false;
   }
}
