package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.List;

/**
 * Where the robot has to stand to shoot at a strip of AprilTags that hangs on the HIVE, worked out from the HIVE's size and position.
 * No hardware in here, so it is unit tested on a laptop.
 *
 * The HIVE is treated as a rectangle (its frame on the floor) with its sides along the field axes. The tag strip is on one of its four
 * faces, a given distance along that face from the face's centre. The robot stands in front of that face, looking straight at the strip,
 * the camera a chosen distance from the strip's plane.
 *
 * Everything is worked out for the RED alliance; the blue alliance's plan is the same turned 180 degrees about the field centre
 * (rotate180), the same convention as every other autonomous here.
 *
 * Field frame: inches, x from the red wall (0) to the blue wall (144), y along the walls, heading in radians counter-clockwise from +x.
 */
public final class ShootPlan {

    public static final double FIELD_IN = 144.0;

    /** Which face of the HIVE the strip is on = which way the strip looks. */
    public static final int MINUS_X = 0, PLUS_X = 1, MINUS_Y = 2, PLUS_Y = 3;

    public static String faceName(int face) {
        switch (face) {
            case MINUS_X: return "the face toward the red wall (-x)";
            case PLUS_X: return "the face toward the blue wall (+x)";
            case MINUS_Y: return "the face toward the bottom wall (-y)";
            case PLUS_Y: return "the face toward the top wall (+y)";
            default: return "an unknown face (" + face + ")";
        }
    }

    public static final class Config {
        public double hiveCx = 72, hiveCy = 72;
        /** The HIVE frame's size along x and along y, inches. */
        public double hiveSizeX = 49, hiveSizeY = 39;
        /** The face the strip is on (RED alliance's), MINUS_X, PLUS_X, MINUS_Y or PLUS_Y. */
        public int face = MINUS_Y;
        /** Where the strip's centre is along that face, inches from the face's centre (+ = toward +y on an x face, toward +x on a y face). */
        public double alongOffsetIn = 0;
        /** Distance from the CAMERA to the strip's plane, measured straight out from the face, inches. */
        public double standoffIn = 30;
        /** How far ahead of the robot's centre the camera is, inches. */
        public double cameraForwardIn = 6;
        public double robotLengthIn = 16, robotWidthIn = 16;
        /** Turn the whole plan 180 degrees about the field centre (blue alliance). */
        public boolean rotate180 = false;
        /** The robot's front must stay at least this far from the face, inches. */
        public double minFrontGapIn = 2.0;
    }

    /** The centre of the strip on the HIVE face. */
    public final double stripX, stripY;
    /** Unit vector pointing out of the HIVE through the strip's face. */
    public final double normalX, normalY;
    /** The robot's shooting pose: centre and heading (facing the strip). */
    public final double x, y, heading;
    /** The standoff (camera to strip plane) range that keeps the robot inside the field and clear of the face, inches. */
    public final double minStandoffIn, maxStandoffIn;
    /** The face the strip is on, after any 180 degree turn. */
    public final int face;
    public final List<String> problems = new ArrayList<>();

    private ShootPlan(double stripX, double stripY, double nx, double ny, double x, double y, double heading,
                      double minStandoffIn, double maxStandoffIn, int face) {
        this.stripX = stripX;
        this.stripY = stripY;
        this.normalX = nx;
        this.normalY = ny;
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.minStandoffIn = minStandoffIn;
        this.maxStandoffIn = maxStandoffIn;
        this.face = face;
    }

    private static int opposite(int face) {
        return face ^ 1;   // MINUS_X <-> PLUS_X, MINUS_Y <-> PLUS_Y
    }

    public static ShootPlan plan(Config c) {
        // ---- in the red frame
        double nx, ny, tx, ty, faceX, faceY, faceSize;
        switch (c.face) {
            case MINUS_X: nx = -1; ny = 0; tx = 0; ty = 1; faceX = c.hiveCx - c.hiveSizeX / 2; faceY = c.hiveCy; faceSize = c.hiveSizeY; break;
            case PLUS_X:  nx = 1;  ny = 0; tx = 0; ty = 1; faceX = c.hiveCx + c.hiveSizeX / 2; faceY = c.hiveCy; faceSize = c.hiveSizeY; break;
            case PLUS_Y:  nx = 0;  ny = 1; tx = 1; ty = 0; faceX = c.hiveCx; faceY = c.hiveCy + c.hiveSizeY / 2; faceSize = c.hiveSizeX; break;
            default:      nx = 0;  ny = -1; tx = 1; ty = 0; faceX = c.hiveCx; faceY = c.hiveCy - c.hiveSizeY / 2; faceSize = c.hiveSizeX; break;
        }
        double sx = faceX + tx * c.alongOffsetIn, sy = faceY + ty * c.alongOffsetIn;
        double d = c.standoffIn + c.cameraForwardIn;           // robot centre to the strip plane
        double rx = sx + nx * d, ry = sy + ny * d;
        double heading = Math.atan2(-ny, -nx);                  // looking at the strip

        // ---- limits on the standoff: the robot's length is along its heading (the normal), its width across
        double halfLen = c.robotLengthIn / 2, halfWid = c.robotWidthIn / 2;
        double minD = halfLen + c.minFrontGapIn;                // front edge clear of the face
        double wallRoom = nx < 0 ? sx : nx > 0 ? FIELD_IN - sx : ny < 0 ? sy : FIELD_IN - sy;   // strip to the wall behind the robot
        double maxD = wallRoom - halfLen;                       // back edge inside the wall
        double minStandoff = minD - c.cameraForwardIn, maxStandoff = maxD - c.cameraForwardIn;

        // ---- rotate for the other alliance
        int face = c.face;
        if (c.rotate180) {
            sx = FIELD_IN - sx; sy = FIELD_IN - sy; rx = FIELD_IN - rx; ry = FIELD_IN - ry;
            nx = -nx; ny = -ny; heading += Math.PI; face = opposite(face);
        }
        ShootPlan p = new ShootPlan(sx, sy, nx, ny, rx, ry, heading, minStandoff, maxStandoff, face);

        if (Math.abs(c.alongOffsetIn) > faceSize / 2) {
            p.problems.add(String.format("The strip is %.1f in from the centre of its face, but the face is only %.1f in wide.", Math.abs(c.alongOffsetIn), faceSize));
        }
        if (c.standoffIn < minStandoff) {
            p.problems.add(String.format("Standoff %.1f in puts the robot's front %.1f in from the HIVE face: the least is %.1f in.",
                    c.standoffIn, d - halfLen, minStandoff));
        }
        if (c.standoffIn > maxStandoff) {
            p.problems.add(String.format("Standoff %.1f in puts the robot through the field wall behind it: the most is %.1f in.", c.standoffIn, maxStandoff));
        }
        // across the standoff direction: the robot's width must fit between the walls
        double across = nx != 0 ? ry : rx;
        if (across < halfWid || across > FIELD_IN - halfWid) {
            p.problems.add(String.format("The shooting position is %.1f in along the wall, too close to a side wall for a %.0f in wide robot.", across, c.robotWidthIn));
        }
        return p;
    }

    // ---- getting there ----

    /** Is the straight line between two poses legal all the way (heading turning linearly, sampled every inch)? */
    public static boolean straightLegal(double sx, double sy, double sh, double gx, double gy, double gh, ObstacleMath.PoseLegal legal) {
        double len = Math.hypot(gx - sx, gy - sy);
        int n = Math.max(1, (int) Math.ceil(len));
        double turn = shortestTurn(sh, gh);
        for (int i = 1; i <= n; i++) {                        // the start itself need not be legal: the robot is already there
            double f = (double) i / n;
            if (!legal.ok(sx + (gx - sx) * f, sy + (gy - sy) * f, sh + turn * f)) return false;
        }
        return true;
    }

    /**
     * The waypoints between the start and the goal, not counting either: none if the straight line is legal, otherwise the way round the
     * keep-out areas that ObstacleMath's grid search finds (the robot keeps its start heading, turning only on the last leg). null if there
     * is no way, with the reason in why[0].
     */
    public static double[][] route(double sx, double sy, double sh, double gx, double gy, double gh, ObstacleMath.PoseLegal legal, String[] why) {
        if (straightLegal(sx, sy, sh, gx, gy, gh, legal)) return new double[0][];
        ObstacleMath.Plan p = ObstacleMath.planAround(sx, sy, sh, gx, gy, gh, new ArrayList<ObstacleMath.Obstacle>(), 0, 0, 0, 400, legal);
        if (p.status != ObstacleMath.Status.DETOUR) {
            if (why != null && why.length > 0) why[0] = p.note;
            return null;
        }
        return p.waypoints;
    }

    static double shortestTurn(double a, double b) {
        double d = (b - a) % (2 * Math.PI);
        if (d > Math.PI) d -= 2 * Math.PI;
        if (d < -Math.PI) d += 2 * Math.PI;
        return d;
    }
}
