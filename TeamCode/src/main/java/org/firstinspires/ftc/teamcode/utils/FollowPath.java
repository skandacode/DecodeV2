package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.paths.Path;

public class FollowPath extends CommandBuilder {
    private final Follower follower;
    private final Path path;
    private double tConstraint = .975;

    public FollowPath(Follower f, Path pathChain) {
        this.follower = f;
        this.path = pathChain;
        this.initialize();
    }

    public FollowPath(Follower f, Path pathChain, double tConstraint) {
        this(f, pathChain);
        this.tConstraint = tConstraint;
    }

    private void initialize() {
        this.setStart(() -> this.follower.follow(this.path));
        this.setDone(() -> this.follower.closestT() > tConstraint);
    }
}
