package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.RoutePolyline;

import java.util.ArrayList;

/*
 * AVOIDANDPARK: drives the Leave and Park route, and goes round an opposing robot that the Limelight sees in the way.
 *
 * ROUTE: exactly the Leave and Park route, checks and settings (LeaveAndPark.*: USE_SUPPLIED_ROUTE, ROTATE_180, MAX_POWER,
 * ENFORCE_*, TIMEOUT_SEC ...). With no obstacle it drives it unchanged.
 *
 * HOW IT AVOIDS, and what it cannot do, is described in AvoidOpModeBase (shared with AVOIDROUTE2) and in docs/LIMELIGHT_OBSTACLES.md.
 * Specific to this route: on legs 1 and 3 the robot drives sideways to its heading, so the camera cannot see where it is going there;
 * it only reacts to what it saw earlier (MEMORY_SEC) or sees ahead.
 *
 * Needs the drive motors, dead wheels and IMU plus the Limelight named "limelight".
 */
@Autonomous(name = "AVOIDANDPARK", group = "Competition")
public class AvoidAndPark extends AvoidOpModeBase {

    @Override
    String headline() {
        return "AVOIDANDPARK: the Leave and Park route, going round opposing robots the camera sees.";
    }

    @Override
    Setup buildSetup(Follower follower, boolean withChains) {
        Setup s = new Setup();
        s.plan = LeaveAndPark.buildPlan();
        s.supplied = LeaveAndPark.USE_SUPPLIED_ROUTE;
        LeaveAndPark.Paths paths = new LeaveAndPark.Paths(follower, LeaveAndPark.ROTATE_180);
        Pose[][] segments = s.supplied ? paths.segments() : LeaveAndPark.generatedSegments(s.plan);
        s.report = LeaveAndPark.evaluate(s.plan, segments, s.supplied);
        PathChain[] chains = null;
        if (withChains) {
            chains = s.supplied ? paths.chains() : new PathChain[]{follower.pathBuilder()
                    .addPath(new BezierLine(segments[0][0], segments[0][1]))
                    .setConstantHeadingInterpolation(s.plan.heading)
                    .build()};
        }
        s.legs = new ArrayList<>();
        for (int i = 0; i < segments.length; i++) {
            Leg l = new Leg();
            l.start = segments[i][0];
            l.end = segments[i][1];
            l.path = RoutePolyline.line(l.start.getX(), l.start.getY(), l.end.getX(), l.end.getY());
            l.chain = chains == null ? null : chains[i];
            s.legs.add(l);
        }
        return s;
    }

    @Override double maxPower() { return LeaveAndPark.MAX_POWER; }
    @Override double startDelaySec() { return LeaveAndPark.START_DELAY_SEC; }
    @Override boolean holdEnd() { return LeaveAndPark.HOLD_END; }
    @Override double timeoutSec() { return LeaveAndPark.TIMEOUT_SEC; }
    @Override double maxOffPathIn() { return LeaveAndPark.MAX_OFF_LINE_IN; }
    @Override boolean showParkLines() { return true; }
}
