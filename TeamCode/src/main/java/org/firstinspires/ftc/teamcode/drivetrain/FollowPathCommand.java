package org.firstinspires.ftc.teamcode.drivetrain;

import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;

/**
 * Allows you to run a PathChain or a Path (which is then converted into a PathChain) by scheduling it.
 * holdEnd is set to true by default, so you only need to give it your instance of follower and the Path to follow.
 * <p>This command is based on FollowPathCommand from solverslib but adds subsystem integration</p>
 */
public class FollowPathCommand extends CommandBase {

    private final Follower follower;
    private final PathChain pathChain;
    private final boolean holdEnd;
    private final DrivetrainSubsystem drivetrainSubsystem;
    private double maxPower = 1.0;

    public FollowPathCommand(Follower follower, PathChain pathChain, DrivetrainSubsystem drivetrainSubsystem) {
        this(follower, pathChain, true, drivetrainSubsystem);
        addRequirements(drivetrainSubsystem);
    }

    public FollowPathCommand(Follower follower, PathChain pathChain, boolean holdEnd, DrivetrainSubsystem drivetrainSubsystem) {
        this(follower, pathChain, holdEnd, 1.0, drivetrainSubsystem);
    }

    public FollowPathCommand(Follower follower, PathChain pathChain, double maxPower, DrivetrainSubsystem drivetrainSubsystem) {
        this(follower, pathChain, true, maxPower, drivetrainSubsystem);
    }

    public FollowPathCommand(Follower follower, PathChain pathChain, boolean holdEnd, double maxPower, DrivetrainSubsystem drivetrainSubsystem) {
        this.follower = follower;
        this.pathChain = pathChain;
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
        this.drivetrainSubsystem = drivetrainSubsystem;
    }

    public FollowPathCommand(Follower follower, Path pathChain, DrivetrainSubsystem drivetrainSubsystem) {
        this(follower, pathChain, true, drivetrainSubsystem);
    }

    public FollowPathCommand(Follower follower, Path pathChain, boolean holdEnd, DrivetrainSubsystem drivetrainSubsystem) {
        this(follower, pathChain, holdEnd, 1.0, drivetrainSubsystem);
    }

    public FollowPathCommand(Follower follower, Path pathChain, double maxPower, DrivetrainSubsystem drivetrainSubsystem) {
        this(follower, pathChain, true, maxPower, drivetrainSubsystem);
    }

    public FollowPathCommand(Follower follower, Path pathChain, boolean holdEnd, double maxPower, DrivetrainSubsystem drivetrainSubsystem) {
        this.follower = follower;
        this.pathChain = new PathChain(pathChain);
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
        this.drivetrainSubsystem = drivetrainSubsystem;
    }

    /**
     * Sets Global Maximum Power for Follower, and overwrites maxPower in constructor
     *
     * @param globalMaxPower The new globalMaxPower
     * @return This command for compatibility in command groups
     */
    public FollowPathCommand setGlobalMaxPower(double globalMaxPower) {
        follower.setMaxPower(globalMaxPower);
        maxPower = globalMaxPower;
        return this;
    }

    @Override
    public void initialize() {
        if (maxPower != 1.0) {
            follower.followPath(pathChain, maxPower, holdEnd);
        }
        follower.followPath(pathChain, holdEnd);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}