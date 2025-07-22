package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.team2052.lib.helpers.MathHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.drive.alignment.AlignmentCommandFactory;
import frc.robot.subsystems.arm.ArmRollerSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.FieldElementFace;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.Logger;

public class RobotState {
    private SwerveDriveState drivetrainState = new SwerveDriveState();
    private AlignOffset selectedAlignOffset = AlignOffset.MIDDLE_REEF;

    private boolean isAlignGoal;
    private FieldElementFace desiredReefFace;
    private FieldElementFace seenReefFace;
    private static RobotState INSTANCE;

    public static RobotState getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotState();
        }

        return INSTANCE;
    }

    private RobotState() {}

    public Pose2d getFieldToRobot() {
        if (drivetrainState.Pose != null) {
            return drivetrainState.Pose;
        }

        return MathHelpers.POSE_2D_ZERO;
    }

    public boolean getHasCoral() {
        return ArmRollerSubsystem.getInstance().getHasCoral();
    }

    public void setAlignOffset(AlignOffset offset) {
        // System.out.println("NEW OFFSET " + offset.toString());
        selectedAlignOffset = offset;
    }

    public AlignOffset getAlignOffset() {
        return selectedAlignOffset;
    }

    public void seenReefFaceID(int tagID) {
        seenReefFace = AlignmentCommandFactory.idToReefFace(tagID);
    }

    public void setDesiredReefFace(FieldElementFace reefFace) {
        desiredReefFace = reefFace;
    }

    public boolean getisAlignGoal() {
        return isAlignGoal;
    }

    public void setIsAtAlignGoal(boolean atGoal) {
        isAlignGoal = atGoal;
    }

    public Pose2d getAlignPose() {
        // if (selectedAlignOffset == AlignOffset.MIDDLE_REEF) {
        //     return getFieldToRobot()
        //             .nearest(isRedAlliance() ? FieldConstants.redLeftBranchL1 : FieldConstants.blueLeftBranchL1);
        // } else
        if (selectedAlignOffset == AlignOffset.LEFT_BRANCH) {
            return getFieldToRobot()
                    .nearest(isRedAlliance() ? FieldConstants.redLeftBranches : FieldConstants.blueLeftBranches);
        } else if (selectedAlignOffset == AlignOffset.RIGHT_BRANCH) {
            return getFieldToRobot()
                    .nearest(isRedAlliance() ? FieldConstants.redRightBranches : FieldConstants.blueRightBranches);
        } else {
            return getFieldToRobot(); // this should never happen
        }
    }

    public boolean desiredReefFaceIsSeen() {
        if (seenReefFace == null || desiredReefFace == null) {
            return false;
        }

        return desiredReefFace.getTagID() == seenReefFace.getTagID();
    }

    public Command setAlignOffsetCommand(AlignOffset offset) {
        return new InstantCommand(() -> setAlignOffset(offset));
    }

    public void addDrivetrainState(SwerveDriveState drivetrainState) {
        this.drivetrainState = drivetrainState;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return drivetrainState.Speeds;
    }

    public double getRotationalSpeeds() {
        return drivetrainState.Speeds.omega; // watch out
    }

    /**
     * Returns true if the robot is on red alliance.
     *
     * @return True if the robot is on red alliance.
     */
    public boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return (alliance.get() == DriverStation.Alliance.Red) ? true : false;
        } else {
            return false;
        }
    }

    public void output() {
        Logger.recordOutput("Swerve Module States", drivetrainState.ModuleStates);
        Logger.recordOutput("Swerve Module Goals", drivetrainState.ModuleTargets);
        Logger.recordOutput("Current Pose", drivetrainState.Pose);
        Logger.recordOutput("Goal Align Pose", getAlignPose());
        Logger.recordOutput(
                "Goal Left Alignment",
                getFieldToRobot()
                        .nearest(isRedAlliance() ? FieldConstants.redLeftBranches : FieldConstants.blueLeftBranches));
        Logger.recordOutput(
                "Goal Right Alignment",
                getFieldToRobot()
                        .nearest(isRedAlliance() ? FieldConstants.redRightBranches : FieldConstants.blueRightBranches));
    }
}
