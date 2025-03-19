package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ElevatorTrapezoidProfile;
import frc.robot.subsystems.ElevatorTrapezoidProfile.ElevatorTrapezoidState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Arm.ArmState;

public class ReefCoralOuttakeCommand extends SequentialCommandGroup {
    public ReefCoralOuttakeCommand(Arm arm, ArmState arm_state, ElevatorTrapezoidProfile elevator, ElevatorTrapezoidState elevator_state, Intake intake, LED led) {
        addCommands(


            // Önce Arm'ı istedigi pozisyonuna getir
            new ArmAndElevatorCommand(arm, arm_state, elevator, elevator_state),

            // Intake başlat
            new StartIntake(intake),

            // 0.4 saniye bekle
            new WaitCommand(0.4),

            // Intake durdur
            new StopIntake(intake),

            // Intake'i eski pozisyonuna getir
            new ElevatorTrapezoidCommand(elevator, ElevatorTrapezoidState.NOTRL1, arm),
            
            new ArmCommand(arm, ArmState.NOTRINTAKEARM)

            

        );
    }
}
