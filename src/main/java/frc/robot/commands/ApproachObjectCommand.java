// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.objectDetection.ObjectDetection;
import frc.robot.subsystems.swerve.Drive;
import org.littletonrobotics.junction.Logger;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ApproachObjectCommand extends SequentialCommandGroup {
  /** Creates a new ApproachObjectCommand. */
  private Pose2d targetPose;

  public ApproachObjectCommand(Drive swerve, ObjectDetection objectDetection) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    targetPose = objectDetection.getTargetPosition();
    Logger.recordOutput("Approach Reef Command/targetPose", targetPose);

    addCommands(
        new FunctionalCommand(
            () -> {
              if (objectDetection.coralInVision()) {
                swerve.setTargetPosition(objectDetection.getTargetPosition());
              }
            },
            () -> {},
            (Boolean e) -> {
              swerve.clearTargetPositionController();
            },
            () -> {
              // Finish when we're close to the target or lost sight of coral
              if (!objectDetection.coralInVision()) {
                return true; // Lost sight, finish command
              }

              Pose2d currentPose = frc.robot.RobotState.getInstance().getEstimatedPose();
              Pose2d currentTarget = objectDetection.getTargetPosition();
              double distanceError =
                  currentPose.getTranslation().getDistance(currentTarget.getTranslation());

              // Finish when within 30cm of target (close enough for intake to grab)
              return distanceError < 0.3;
            },
            swerve,
            objectDetection));
  }
}
