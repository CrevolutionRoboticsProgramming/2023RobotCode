package frc.robot.intake.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.intake.IntakeConfig;
import frc.robot.intake.IntakeRoller;
import frc.robot.intake.IntakeConfig.PivotState;

import java.util.function.DoubleSupplier;

public class IntakeCommands {
//     @Deprecated
//     public static Command setPivotState(IntakeConfig.PivotState state) {
// //        return new InstantCommand(() -> RobotContainer.intakePivot.setPivotState(state), RobotContainer.intakePivot);
//         return null;
//     }

//     public static Command runRollerManual(IntakeRoller roller, double supplier) {
//         return new InstantCommand(() -> roller.setOutput(supplier));
//     }

//     @Deprecated
//     public static Command setPivotOutput(DoubleSupplier supplier) {
//         return new InstantCommand(() -> RobotContainer.intakePivot.setOutput(supplier.getAsDouble()), RobotContainer.intakePivot);
//     }

//     public static Command setHoodState(IntakeConfig.HoodState state) {
//         return new InstantCommand(() -> RobotContainer.intakeHood.setState(state));
//     }

    // public static Command clearPivotDown() { 
    //     return new ConditionalCommand(new SetPivotState(RobotContainer.intakePivot, PivotState.kStowed), new WaitCommand(0), () -> {
    //         final var pos = RobotContainer.intakePivot.getAngleRads();
    //         final var lowerBound = Rotation2d.fromDegrees(90);
    //         final var upperBound = Rotation2d.fromDegrees(180);
    //         return (pos > lowerBound.getRadians() && pos < upperBound.getRadians());
    //     });
    // }
}
