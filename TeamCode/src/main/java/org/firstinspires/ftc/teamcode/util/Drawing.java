// @ftc-toolchain generated: hand-authored — Panels field visualization for the driver dashboard
package org.firstinspires.ftc.teamcode.util;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

/**
 * Draws the robot's live field position (and, in auto, the path it's following) onto the
 * Panels field view at http://192.168.43.1:8001 when connected to robot wifi. Adapted from
 * Pedro Pathing's official Panels Drawing class (see the "Choosing a Dashboard" doc).
 *
 * All com.bylazar.field.* and com.pedropathing.* method signatures used here were checked
 * against the actual fullpanels 1.0.12 (field 1.0.6) and pedropathing:ftc 2.1.2 artifacts
 * with javap, so they're verified even though this dev environment has no Android SDK to
 * do a full project build. A real build is still worth doing before competition.
 */
public class Drawing {

    public static final double ROBOT_RADIUS = 9; // inches

    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style("", "#3F51B5", 0.0);
    private static final Style pathLook = new Style("", "#4CAF50", 0.0);

    /** Call once at OpMode init, before the first draw call. */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /** Draws the current path (if any) and the robot's live pose. Call once per loop. */
    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), pathLook);
        }
        drawRobot(follower.getPose(), robotLook);
        update();
    }

    /** Draws just the robot at its current pose. Call once per loop, then update(). */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    public static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();
        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) points[j][i] = 0;
            }
        }
        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }

    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    /** Pushes everything drawn since the last update() to the Panels field view. */
    public static void update() {
        panelsField.update();
    }
}
