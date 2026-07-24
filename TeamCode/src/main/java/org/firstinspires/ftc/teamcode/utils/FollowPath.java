package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.paths.PathChain;

public class FollowPath extends CommandBuilder {
    private final Follower follower;
    private final PathChain path;
    private double tConstraint = .975;

    public FollowPath(Follower f, PathChain pathChain) {
        this.follower = f;
        this.path = pathChain;
        this.initialize();
    }

    public FollowPath(Follower f, PathChain pathChain, double tConstraint) {
        this(f, pathChain);
        this.tConstraint = tConstraint;
    }

    private void initialize() {
        this.setStart(() -> this.follower.followPath(this.path));
        this.setDone(() -> this.follower.getCurrentTValue() > tConstraint);
    }
}
