package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ElevatorTrapezoidProfile;
import frc.robot.subsystems.ElevatorTrapezoidProfile.ElevatorTrapezoidState;
import frc.robot.subsystems.Arm.ArmState;

public class ArmAndElevatorCommand extends SequentialCommandGroup {
    public ArmAndElevatorCommand(Arm arm, ArmState arm_state, ElevatorTrapezoidProfile elevator, ElevatorTrapezoidState elevator_state) {
        addCommands(
    new ArmCommand(arm, arm_state),
    new ElevatorTrapezoidCommand(elevator, elevator_state,arm),
    new WaitCommand(0.4)
);
 }
}
