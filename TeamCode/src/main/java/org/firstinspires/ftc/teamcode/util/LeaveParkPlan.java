package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.List;

/**
 * Plans the shortest legal-looking LEAVE + PARK move for BIOBUZZ and checks it against the field elements.
 * Pure geometry, no FTC or Pedro classes, so it can be unit tested on a laptop.
 *
 * Field frame (same as Pedro): 144 x 144 inches, (0,0) bottom-left, +x right, +y up, heading counter-clockwise from +x.
 *
 * The plan is built from the LOADING ZONE rectangle, which must sit against one perimeter wall:
 *   - START: robot centred just beside the zone along the wall, back edge touching the wall, facing away from it.
 *   - PARK:  robot centred on the zone, backed off the wall by leaveClearance, still facing away from the wall.
 * One straight line with constant heading joins them. Moving off the wall is LEAVE, ending partly inside the zone
 * is PARK. No turning, so the footprint stays axis-aligned and the swept area is easy to check.
 *
 * Field elements that are avoided: the perimeter walls and the HIVE structure (a keep-out rectangle in the centre of
 * the field, grown by a safety margin). The checks run before the match (plan) and during it (violation).
 */
public final class LeaveParkPlan {

    public static final double FIELD_IN = 144.0;

    /** Everything the plan depends on. Inches and degrees-free (headings come out in radians). */
    public static final class Config {
        /** LOADING ZONE rectangle in field coordinates, before ROTATE_180. */
        public double zoneXMin, zoneYMin, zoneXMax, zoneYMax;
        /** Rotate the zone 180 degrees about the field centre (red zone <-> blue zone). */
        public boolean rotate180;
        /** Which side of the zone to start on along the wall: -1 = lower along-wall side, +1 = the other side. */
        public int startSide = -1;
        /** Robot size: length along its heading (front to back) and width across it, in inches. */
        public double robotLengthIn = 18, robotWidthIn = 18;
        /** Gap between the robot's side and the zone's edge at the start, inches. */
        public double startGapIn = 2.0;
        /** How far the robot's back edge is moved off the wall at the end, so it is clearly no longer touching. */
        public double leaveClearanceIn = 5.0;
        /** Minimum depth the robot must overlap the zone at the end, inches. */
        public double minZoneOverlapIn = 3.0;
        /** HIVE structure keep-out: half sizes around the field centre, and the extra margin around it. */
        public double hiveHalfXIn = 30.0, hiveHalfYIn = 30.0, hiveMarginIn = 6.0;
        /** Slack allowed when checking the robot against the field edge (the start touches the wall). */
        public double wallToleranceIn = 0.5;
    }

    // Results (field coordinates).
    public double startX, startY, parkX, parkY, heading;
    /** Zone rectangle after rotation: {xMin, yMin, xMax, yMax}. */
    public final double[] zone = new double[4];
    /** Keep-out rectangle (hive + margin): {xMin, yMin, xMax, yMax}. */
    public final double[] keepOut = new double[4];
    /** Human readable reasons the plan must not be run. Empty = OK. */
    public final List<String> problems = new ArrayList<>();

    private final Config cfg;

    public boolean ok() {
        return problems.isEmpty();
    }

    public static LeaveParkPlan plan(Config c) {
        return new LeaveParkPlan(c);
    }

    private LeaveParkPlan(Config c) {
        this.cfg = c;

        double zx0 = Math.min(c.zoneXMin, c.zoneXMax), zx1 = Math.max(c.zoneXMin, c.zoneXMax);
        double zy0 = Math.min(c.zoneYMin, c.zoneYMax), zy1 = Math.max(c.zoneYMin, c.zoneYMax);
        if (c.rotate180) {
            double nx0 = FIELD_IN - zx1, nx1 = FIELD_IN - zx0, ny0 = FIELD_IN - zy1, ny1 = FIELD_IN - zy0;
            zx0 = nx0; zx1 = nx1; zy0 = ny0; zy1 = ny1;
        }
        zone[0] = zx0; zone[1] = zy0; zone[2] = zx1; zone[3] = zy1;

        keepOut[0] = FIELD_IN / 2 - c.hiveHalfXIn - c.hiveMarginIn;
        keepOut[1] = FIELD_IN / 2 - c.hiveHalfYIn - c.hiveMarginIn;
        keepOut[2] = FIELD_IN / 2 + c.hiveHalfXIn + c.hiveMarginIn;
        keepOut[3] = FIELD_IN / 2 + c.hiveHalfYIn + c.hiveMarginIn;

        if (zx0 < -0.01 || zy0 < -0.01 || zx1 > FIELD_IN + 0.01 || zy1 > FIELD_IN + 0.01 || zx1 - zx0 < 1 || zy1 - zy0 < 1) {
            problems.add("Loading zone rectangle is not inside the 144 x 144 field.");
            return;
        }

        // Find the wall the zone sits against. If it touches two (a corner), take the wall it runs longest along.
        double tol = 0.5;
        int wall = -1; // 0 left, 1 right, 2 bottom, 3 top
        double bestAlong = -1;
        for (int w = 0; w < 4; w++) {
            boolean touches = (w == 0 && zx0 <= tol) || (w == 1 && zx1 >= FIELD_IN - tol)
                    || (w == 2 && zy0 <= tol) || (w == 3 && zy1 >= FIELD_IN - tol);
            if (!touches) continue;
            double along = (w <= 1) ? (zy1 - zy0) : (zx1 - zx0);
            if (along > bestAlong) { bestAlong = along; wall = w; }
        }
        if (wall < 0) {
            problems.add("Loading zone does not touch a perimeter wall; this plan needs it against one.");
            return;
        }

        // Wall frame: field point = origin + d * n + s * t, d = distance from the wall, s = position along it.
        double ox, oy, nx, ny;
        switch (wall) {
            case 0:  ox = 0;         oy = 0;         nx = 1;  ny = 0;  break; // left wall, facing +x
            case 1:  ox = FIELD_IN;  oy = FIELD_IN;  nx = -1; ny = 0;  break; // right wall, facing -x
            case 2:  ox = FIELD_IN;  oy = 0;         nx = 0;  ny = 1;  break; // bottom wall, facing +y
            default: ox = 0;         oy = FIELD_IN;  nx = 0;  ny = -1; break; // top wall, facing -y
        }
        double tx = -ny, ty = nx;

        double sMin = Double.MAX_VALUE, sMax = -Double.MAX_VALUE, dMax = -Double.MAX_VALUE;
        double[][] zc = {{zx0, zy0}, {zx1, zy0}, {zx1, zy1}, {zx0, zy1}};
        for (double[] p : zc) {
            double s = (p[0] - ox) * tx + (p[1] - oy) * ty;
            double d = (p[0] - ox) * nx + (p[1] - oy) * ny;
            sMin = Math.min(sMin, s);
            sMax = Math.max(sMax, s);
            dMax = Math.max(dMax, d);
        }

        heading = Math.atan2(ny, nx);
        double halfD = c.robotLengthIn / 2, halfS = c.robotWidthIn / 2;

        double startS = (c.startSide < 0) ? sMin - c.startGapIn - halfS : sMax + c.startGapIn + halfS;
        double startD = halfD;                       // back edge touching the wall
        double parkS = (sMin + sMax) / 2;
        double parkD = halfD + c.leaveClearanceIn;   // back edge clearly off the wall

        startX = ox + startD * nx + startS * tx;
        startY = oy + startD * ny + startS * ty;
        parkX = ox + parkD * nx + parkS * tx;
        parkY = oy + parkD * ny + parkS * ty;

        // Pre-flight checks.
        double overlapDepth = dMax - c.leaveClearanceIn; // robot back edge sits at leaveClearance, zone ends at dMax
        if (overlapDepth < c.minZoneOverlapIn) {
            problems.add(String.format("Robot would overlap the zone by only %.1f in at the end (need %.1f). "
                    + "Lower leaveClearance or check the zone depth.", overlapDepth, c.minZoneOverlapIn));
        }
        if (overlapWithZone(parkX, parkY, heading) <= 0) {
            problems.add("Final pose is not inside the loading zone.");
        }
        if (overlapWithZone(startX, startY, heading) > 0) {
            problems.add("Start pose overlaps the loading zone; widen the start gap or flip the start side.");
        }
        String v = violationAlongLine();
        if (v != null) problems.add(v);
    }

    /** Checks the whole planned straight line, start to park, in 0.5 in steps. Null = clear. */
    private String violationAlongLine() {
        double len = Math.hypot(parkX - startX, parkY - startY);
        int steps = Math.max(1, (int) Math.ceil(len / 0.5));
        for (int i = 0; i <= steps; i++) {
            double f = (double) i / steps;
            String v = violation(startX + (parkX - startX) * f, startY + (parkY - startY) * f, heading);
            if (v != null) return "Planned path: " + v + String.format(" (at %.0f%% of the move)", f * 100);
        }
        return null;
    }

    /**
     * Checks one robot pose against the field. Returns null when fine, otherwise the reason.
     * Used before the match on the planned path and during the match on the live odometry pose.
     */
    public String violation(double x, double y, double h) {
        double[][] box = footprint(x, y, h);
        double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE, minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
        for (double[] p : box) {
            minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0]);
            minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1]);
        }
        double t = cfg.wallToleranceIn;
        if (minX < -t || minY < -t || maxX > FIELD_IN + t || maxY > FIELD_IN + t) {
            return "robot would be outside the field walls";
        }
        double[][] hive = {{keepOut[0], keepOut[1]}, {keepOut[2], keepOut[1]}, {keepOut[2], keepOut[3]}, {keepOut[0], keepOut[3]}};
        if (overlap(box, hive)) {
            return "robot would touch the HIVE keep-out area";
        }
        return null;
    }

    /** Smallest distance from the robot's footprint to any perimeter wall. Above 0 means "no longer touching". */
    public double wallGap(double x, double y, double h) {
        double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE, minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
        for (double[] p : footprint(x, y, h)) {
            minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0]);
            minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1]);
        }
        return Math.min(Math.min(minX, FIELD_IN - maxX), Math.min(minY, FIELD_IN - maxY));
    }

    /** Area (square inches) of the robot's bounding box inside the zone. Above 0 means "at least partially in". */
    public double overlapWithZone(double x, double y, double h) {
        double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE, minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
        for (double[] p : footprint(x, y, h)) {
            minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0]);
            minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1]);
        }
        double w = Math.min(maxX, zone[2]) - Math.max(minX, zone[0]);
        double d = Math.min(maxY, zone[3]) - Math.max(minY, zone[1]);
        return (w > 0 && d > 0) ? w * d : 0;
    }

    /** Robot corners (front-left, front-right, back-right, back-left) for a centre pose. */
    public double[][] footprint(double x, double y, double h) {
        double hl = cfg.robotLengthIn / 2, hw = cfg.robotWidthIn / 2;
        double c = Math.cos(h), s = Math.sin(h);
        double[][] local = {{hl, hw}, {hl, -hw}, {-hl, -hw}, {-hl, hw}};
        double[][] out = new double[4][2];
        for (int i = 0; i < 4; i++) {
            out[i][0] = x + local[i][0] * c - local[i][1] * s;
            out[i][1] = y + local[i][0] * s + local[i][1] * c;
        }
        return out;
    }

    /** Separating-axis test for two convex quadrilaterals. Touching counts as overlapping (the safe answer). */
    static boolean overlap(double[][] a, double[][] b) {
        for (double[][] poly : new double[][][]{a, b}) {
            for (int i = 0; i < 4; i++) {
                double ex = poly[(i + 1) % 4][0] - poly[i][0], ey = poly[(i + 1) % 4][1] - poly[i][1];
                double ax = -ey, ay = ex; // edge normal
                double aMin = Double.MAX_VALUE, aMax = -Double.MAX_VALUE, bMin = Double.MAX_VALUE, bMax = -Double.MAX_VALUE;
                for (double[] p : a) { double d = p[0] * ax + p[1] * ay; aMin = Math.min(aMin, d); aMax = Math.max(aMax, d); }
                for (double[] p : b) { double d = p[0] * ax + p[1] * ay; bMin = Math.min(bMin, d); bMax = Math.max(bMax, d); }
                if (aMax < bMin || bMax < aMin) return false;
            }
        }
        return true;
    }
}
