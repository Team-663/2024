// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;

public class intakeBackupNote extends Command {
   private final Shooter m_shooter;
   private Timer time;

   /** Creates a new intakeBackupNote. */
   public intakeBackupNote(Shooter shooter) {
      m_shooter = shooter;
      time = new Timer();
      addRequirements(m_shooter);
      // Use addRequirements() here to declare subsystem dependencies.
   }

   // Called when the command is initially scheduled.
   @Override
   public void initialize() {
      time.reset();
      time.start();
   }

   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute() {
      m_shooter.intakeBackwardSlow(true);
   }

   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
      m_shooter.intakeBackwardSlow(false);
   }

   // Returns true when the command should end.
   @Override
   public boolean isFinished() 
   {
      return time.hasElapsed(0.25);
      //return !m_shooter.CheckIfNoteInIntake();
   }
}
