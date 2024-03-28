// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import java.lang.constant.Constable;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.Auton;
import edu.wpi.first.wpilibj.Timer;

public class intakeOneNote extends Command {
   /** Creates a new intakeOneNote. */
   Shooter m_shooter;
   private Timer time;

   public intakeOneNote(Shooter shooter)
   {
      // Use addRequirements() here to declare subsystem dependencies.
      m_shooter = shooter;
      time = new Timer();
      addRequirements(m_shooter);
   }

   // Called when the command is initially scheduled.
   @Override
   public void initialize()
   {
      m_shooter.setArmPosition(ArmConstants.ARM_DOWN_ENCODER_VALUE);
      time.reset();
      time.start();
   }

   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute()
   {
      //if (m_shooter.IsShooterSpinningTooFastForIntake()) {
      //   m_shooter.intakeNote(.40);
      //} else {
         m_shooter.intakeNote(.85);
      //}
   }

   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
   }

   // Returns true when the command should end.
   @Override
   public boolean isFinished()
   {
      if (m_shooter.CheckIfNoteAnywhere() == true)
      {
         return true;
      }

      if (time.hasElapsed(Auton.AUTO_INTAKE_MAX_DURATION) == true)
      {
         return true;
      }

      return false;
      //return (m_shooter.CheckIfNoteAnywhere() || time.hasElapsed(Auton.AUTO_INTAKE_MAX_DURATION));
   }
}
