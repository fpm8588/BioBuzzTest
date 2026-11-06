package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.PriorityQueue;

/**
 * Geometry for seeing an obstacle (an opposing robot) with the Limelight and driving around it. Pure Java, no FTC or
 * Pedro classes, so it can be unit tested on a laptop.
 *
 * Field frame (same as Pedro): inches, +x forward along heading 0, heading counter-clockwise. Limelight tx is degrees
 * from the crosshair, positive = the target is to the RIGHT, which is a clockwise (negative) bearing.
 *
 * Distance comes from how big the coloured patch looks: area (a fraction of the image, 0 to 1) falls with the square of
 * the distance, so distance = k / sqrt(area), where k is measured once for the thing being detected (see
 * docs/LIMELIGHT_OBSTACLES.md). This is only as good as the patch is the same size on every robot, so expect about
 * +/- 30 percent and keep the clearances generous.
 */
public final class ObstacleMath {

    private ObstacleMath() {}

    /** A thing to stay away from: centre and radius in inches. */
    public static final class Obstacle {
        public final double x, y, radius;

        public Obstacle(double x, double y, double radius) {
            this.x = x;
            this.y = y;
            this.radius = radius;
        }
    }

    /** True if a pose is somewhere the robot may stand (inside the walls, out of keep-out areas). */
    public interface PoseLegal {
        boolean ok(double x, double y, double heading);
    }

    // ---- sensing ----

    /** Distance from the camera to the coloured patch, from its area. NaN if there is no area. */
    public static double distanceFromArea(double area, double k) {
        return area > 0 ? k / Math.sqrt(area) : Double.NaN;
    }

    /** The k that makes distanceFromArea() give knownDistanceIn for a patch seen with this area. */
    public static double kFromKnown(double knownDistanceIn, double area) {
        return area > 0 ? knownDistanceIn * Math.sqrt(area) : Double.NaN;
    }

    /**
     * Field position of a patch seen at txDeg (+ = right of the crosshair) and distanceIn from the camera, for a robot
     * at (rx, ry) facing rh with the camera cameraForwardIn ahead of the robot's centre.
     *
     * @return {x, y}
     */
    public static double[] fieldPosition(double rx, double ry, double rh, double txDeg, double distanceIn, double cameraForwardIn) {
        double camX = rx + cameraForwardIn * Math.cos(rh);
        double camY = ry + cameraForwardIn * Math.sin(rh);
        double bearing = rh - Math.toRadians(txDeg); // right of the crosshair = clockwise = smaller angle
        return new double[]{camX + distanceIn * Math.cos(bearing), camY + distanceIn * Math.sin(bearing)};
    }

    /**
     * The farthest floor distance at which an object whose centre is objectHeightIn above the floor is still in the picture.
     * The top edge of the image is (pitchDownDeg - verticalFovDeg / 2) below horizontal; an object lower than the lens
     * drops below that line the farther away it is, so a camera tilted down for pollen on the floor stops seeing robot signs
     * beyond this distance.
     *
     * @return inches, or infinity if the top of the picture is at or above the horizon (everything ahead is in view),
     *         or NaN if the object is not below the lens
     */
    public static double farthestVisibleDistanceIn(double cameraHeightIn, double pitchDownDeg, double verticalFovDeg, double objectHeightIn) {
        double drop = cameraHeightIn - objectHeightIn;
        if (drop <= 0) return Double.NaN;
        double topEdgeBelowHorizontal = pitchDownDeg - verticalFovDeg / 2;
        if (topEdgeBelowHorizontal <= 0.1) return Double.POSITIVE_INFINITY;
        return drop / Math.tan(Math.toRadians(topEdgeBelowHorizontal));
    }

    /**
     * Could a camera on a robot at (rx, ry) facing rh see the field point (px, py)? The camera is cameraForwardIn ahead of the
     * robot's centre, sees hfovDeg across, and sees things between nearIn and farIn (use nearestVisible / farthestVisible).
     * Used so an obstacle is only believed gone when the camera could have seen it.
     */
    public static boolean inCameraView(double rx, double ry, double rh, double px, double py, double cameraForwardIn,
                                       double hfovDeg, double nearIn, double farIn) {
        double cx = rx + cameraForwardIn * Math.cos(rh), cy = ry + cameraForwardIn * Math.sin(rh);
        double dx = px - cx, dy = py - cy;
        double fwd = dx * Math.cos(rh) + dy * Math.sin(rh), left = -dx * Math.sin(rh) + dy * Math.cos(rh);
        if (fwd <= 0) return false;
        double d = Math.hypot(dx, dy);
        return Math.toDegrees(Math.abs(Math.atan2(left, fwd))) <= hfovDeg / 2 && d >= nearIn && d <= farIn;
    }

    /**
     * Measures DISTANCE_K while driving straight at a patch: the patch grew from area a1 to a2 while the robot moved travelIn
     * toward it, and distance = k / sqrt(area), so travel = k * (1/sqrt(a1) - 1/sqrt(a2)). NaN if the patch did not grow.
     */
    public static double kFromApproach(double travelIn, double a1, double a2) {
        if (a1 <= 0 || a2 <= a1) return Double.NaN;
        return travelIn / (1 / Math.sqrt(a1) - 1 / Math.sqrt(a2));
    }

    // ---- segments ----

    /** Distance from point (px, py) to the segment a-b. */
    public static double distanceToSegment(double px, double py, double ax, double ay, double bx, double by) {
        double dx = bx - ax, dy = by - ay;
        double len2 = dx * dx + dy * dy;
        double t = len2 == 0 ? 0 : Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / len2));
        return Math.hypot(px - (ax + t * dx), py - (ay + t * dy));
    }

    /** Distance along a-b (from a, in inches) of the point of the line nearest to p. Negative = behind a. */
    public static double alongSegment(double px, double py, double ax, double ay, double bx, double by) {
        double len = Math.hypot(bx - ax, by - ay);
        return len == 0 ? 0 : ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / len;
    }

    // ---- planning ----

    public enum Status { CLEAR, DETOUR, BLOCKED }

    /** The result of planning. For DETOUR, drive through each waypoint in order, then on to the goal. */
    public static final class Plan {
        public final Status status;
        /** {x, y} of each waypoint: none for CLEAR and BLOCKED, one or two for DETOUR. */
        public final double[][] waypoints;
        /** Extra distance the detour adds compared with going straight. */
        public final double extraLengthIn;
        public final String note;

        public Plan(Status status, double[][] waypoints, double extra, String note) {
            this.status = status;
            this.waypoints = waypoints;
            this.extraLengthIn = extra;
            this.note = note;
        }
    }

    /** Safe distance between the robot's centreline and an obstacle's centre. */
    public static double clearance(Obstacle o, double robotRadiusIn, double marginIn) {
        return o.radius + robotRadiusIn + marginIn;
    }

    /**
     * Does going straight from (sx, sy) to (gx, gy) pass too close to an obstacle that is ahead and within triggerRangeIn?
     * An obstacle we are already inside the clearance of only counts as blocking if the path would bring us closer than we
     * are now, so moving sideways away from it is never blocked.
     */
    public static boolean isBlocked(double sx, double sy, double gx, double gy, List<Obstacle> obs,
                                    double robotRadiusIn, double marginIn, double triggerRangeIn, double slackIn) {
        return firstBlocker(sx, sy, gx, gy, obs, robotRadiusIn, marginIn, triggerRangeIn, slackIn) != null;
    }

    /**
     * slackIn: a path only counts as blocked if it comes more than this much INSIDE the clearance. Distances are estimates,
     * and a robot that is following a detour it planned itself sits exactly on the clearance, so without slack tiny noise
     * would flag its own plan as blocked.
     */
    private static Obstacle firstBlocker(double sx, double sy, double gx, double gy, List<Obstacle> obs,
                                         double robotRadiusIn, double marginIn, double triggerRangeIn, double slackIn) {
        Obstacle best = null;
        double bestAlong = Double.MAX_VALUE;
        for (Obstacle o : obs) {
            double c = clearance(o, robotRadiusIn, marginIn);
            double along = alongSegment(o.x, o.y, sx, sy, gx, gy);
            if (along > triggerRangeIn + c) continue;                       // too far ahead to matter yet
            if (along < -c) continue;                                       // behind us
            double allowed = Math.min(c, Math.hypot(o.x - sx, o.y - sy)) - 1e-6;
            if (distanceToSegment(o.x, o.y, sx, sy, gx, gy) < allowed - slackIn && along < bestAlong) {
                bestAlong = along;
                best = o;
            }
        }
        return best;
    }

    /**
     * Plans how to get from the start to the goal past the obstacles. If the straight line is clear the answer is CLEAR.
     * Otherwise it builds a dog-leg around the nearest blocking obstacle: move over to one side BEFORE reaching it, pass
     * alongside at a safe distance, then come back to the line. (A single waypoint level with the obstacle does not work:
     * the diagonal approach cuts inside the clearance.) It tries both sides at a few extra distances and lead-in lengths and
     * keeps the shortest dog-leg whose legs stay legal (inside the walls, out of keep-outs) and clear of EVERY obstacle.
     * If none works the answer is BLOCKED and the caller should stop and wait.
     *
     * Headings: the robot keeps startH until the last waypoint, then turns linearly to goalH on the way to the goal, and
     * legal.ok() is asked with those headings.
     */
    public static Plan planDetour(double sx, double sy, double startH, double gx, double gy, double goalH,
                                  List<Obstacle> obs, double robotRadiusIn, double marginIn, double triggerRangeIn,
                                  double slackIn, PoseLegal legal) {
        Obstacle first = firstBlocker(sx, sy, gx, gy, obs, robotRadiusIn, marginIn, triggerRangeIn, slackIn);
        if (first == null) return new Plan(Status.CLEAR, new double[0][], 0, "path is clear");

        double len = Math.hypot(gx - sx, gy - sy);
        double ux = (gx - sx) / len, uy = (gy - sy) / len;
        double nx = -uy, ny = ux;                           // left of the direction of travel
        double base = clearance(first, robotRadiusIn, marginIn);
        double firstAlong = alongSegment(first.x, first.y, sx, sy, gx, gy);
        // The legs of a detour may come up to half the slack inside the clearance, so a goal that sits a hair inside the
        // clearance of an obstacle does not make every detour impossible. The waypoints themselves are placed at full clearance.
        double legSlack = slackIn * 0.5;

        double bestLen = Double.MAX_VALUE;
        double[][] best = null;
        String bestNote = "";
        double[] extras = {0, 3, 6, 10, 15};
        double[] leads = {0, 0.4, 0.8, 1.2};                // lead-in / lead-out length, times the clearance
        for (int side = -1; side <= 1; side += 2) {
            for (double extra : extras) {
                double d = base + extra;
                double ox = first.x + side * nx * d, oy = first.y + side * ny * d;       // abeam of the obstacle
                for (double in : leads) {
                    // The lead-in cannot be longer than the distance to the obstacle: the robot must not back up.
                    double a1 = Math.min(in * base, Math.max(0, firstAlong));
                    for (double out : leads) {
                        double a2 = out * base;
                        if (firstAlong + a2 > len + 1e-6) continue;                      // would overshoot the goal
                        double w1x = ox - ux * a1, w1y = oy - uy * a1;
                        double w2x = ox + ux * a2, w2y = oy + uy * a2;
                        boolean single = a1 == 0 && a2 == 0;                              // one waypoint, abeam
                        if (!legal.ok(w1x, w1y, startH) || !legal.ok(w2x, w2y, startH)) continue;
                        // The robot is already standing at the start, so the start itself need not be legal (the route may pass
                        // close to a keep-out by design); everything after it must be.
                        if (!legSafe(sx, sy, startH, startH, w1x, w1y, obs, robotRadiusIn, marginIn, legSlack, legal, true)) continue;
                        if (!single && !legSafe(w1x, w1y, startH, startH, w2x, w2y, obs, robotRadiusIn, marginIn, legSlack, legal, false)) continue;
                        if (!legSafe(w2x, w2y, startH, goalH, gx, gy, obs, robotRadiusIn, marginIn, legSlack, legal, false)) continue;
                        double total = Math.hypot(w1x - sx, w1y - sy) + Math.hypot(w2x - w1x, w2y - w1y) + Math.hypot(gx - w2x, gy - w2y);
                        if (total < bestLen - 1e-9) {
                            bestLen = total;
                            best = single ? new double[][]{{w1x, w1y}} : new double[][]{{w1x, w1y}, {w2x, w2y}};
                            bestNote = String.format("round the %s of the obstacle at (%.0f, %.0f), %.0f in clear of it",
                                    side > 0 ? "left" : "right", first.x, first.y, d);
                        }
                    }
                }
            }
        }
        if (best == null) {
            return new Plan(Status.BLOCKED, new double[0][], 0, "no way round the obstacle at (" + Math.round(first.x) + ", " + Math.round(first.y)
                    + ") that stays inside the walls and keep-outs");
        }
        return new Plan(Status.DETOUR, best, bestLen - len, bestNote);
    }

    /**
     * The whole decision: a dog-leg round the obstacle if there is room; otherwise, if maxExtraIn allows, the LONG WAY round
     * (planAround), which can go right round the far side of the HIVE; otherwise BLOCKED.
     */
    public static Plan planRoute(double sx, double sy, double startH, double gx, double gy, double goalH,
                                 List<Obstacle> obs, double robotRadiusIn, double marginIn, double triggerRangeIn,
                                 double slackIn, double maxExtraIn, PoseLegal legal) {
        Plan p = planDetour(sx, sy, startH, gx, gy, goalH, obs, robotRadiusIn, marginIn, triggerRangeIn, slackIn, legal);
        if (p.status != Status.BLOCKED || maxExtraIn <= 0) return p;
        Plan q = planAround(sx, sy, startH, gx, gy, goalH, obs, robotRadiusIn, marginIn, slackIn, maxExtraIn, legal);
        return q.status == Status.DETOUR ? q : p;
    }

    /**
     * Finds ANY legal path from the start to the goal that keeps clear of every obstacle, on a 2 in grid (A*), then pulls it
     * straight into a few waypoints. Unlike planDetour it is not limited to going round beside the obstacle, so it can take
     * the long way round the HIVE when the lane beside the obstacle is too narrow. The robot keeps startH the whole way
     * (it strafes), turning to goalH only on the last leg.
     *
     * @param maxExtraIn longest acceptable path, as extra distance over going straight, inches
     */
    public static Plan planAround(double sx, double sy, double startH, double gx, double gy, double goalH,
                                  List<Obstacle> obs, double robotRadiusIn, double marginIn, double slackIn,
                                  double maxExtraIn, PoseLegal legal) {
        final double cell = 2.0;
        final int n = (int) Math.round(144.0 / cell) + 1;
        final double legSlack = slackIn * 0.5;
        final int startNode = nodeIndex(sx, sy, cell, n), goalNode = nodeIndex(gx, gy, cell, n);

        // Which grid points may the robot's centre stand on?
        boolean[] ok = new boolean[n * n];
        for (int j = 0; j < n; j++) {
            for (int i = 0; i < n; i++) {
                double x = i * cell, y = j * cell;
                boolean pass = legal.ok(x, y, startH);
                for (int k = 0; pass && k < obs.size(); k++) {
                    Obstacle o = obs.get(k);
                    double allowed = Math.min(clearance(o, robotRadiusIn, marginIn), Math.hypot(o.x - sx, o.y - sy)) - 1e-6 - legSlack;
                    if (Math.hypot(o.x - x, o.y - y) < allowed) pass = false;
                }
                ok[j * n + i] = pass;
            }
        }
        ok[startNode] = true;   // the robot is already here
        ok[goalNode] = true;    // its real end is checked below
        // a robot may be standing inside a keep-out margin: let it step out through nearby cells
        for (int j = 0; j < n; j++) {
            for (int i = 0; i < n; i++) {
                if (!ok[j * n + i] && Math.hypot(i * cell - sx, j * cell - sy) <= 4.5 && legal.ok(i * cell, j * cell, startH)) ok[j * n + i] = true;
            }
        }

        // A*
        double[] g = new double[n * n];
        int[] from = new int[n * n];
        java.util.Arrays.fill(g, Double.MAX_VALUE);
        java.util.Arrays.fill(from, -1);
        PriorityQueue<double[]> open = new PriorityQueue<>((a, b) -> Double.compare(a[0], b[0]));
        g[startNode] = 0;
        open.add(new double[]{heuristic(startNode, goalNode, n, cell), startNode});
        boolean found = false;
        while (!open.isEmpty()) {
            double[] top = open.poll();
            int cur = (int) top[1];
            if (cur == goalNode) {
                found = true;
                break;
            }
            if (top[0] - heuristic(cur, goalNode, n, cell) > g[cur] + 1e-9) continue; // stale entry
            int ci = cur % n, cj = cur / n;
            for (int dj = -1; dj <= 1; dj++) {
                for (int di = -1; di <= 1; di++) {
                    if (di == 0 && dj == 0) continue;
                    int ni = ci + di, nj = cj + dj;
                    if (ni < 0 || nj < 0 || ni >= n || nj >= n || !ok[nj * n + ni]) continue;
                    if (di != 0 && dj != 0 && (!ok[cj * n + ni] || !ok[nj * n + ci])) continue; // no cutting corners
                    double ng = g[cur] + cell * Math.hypot(di, dj);
                    int nb = nj * n + ni;
                    if (ng < g[nb]) {
                        g[nb] = ng;
                        from[nb] = cur;
                        open.add(new double[]{ng + heuristic(nb, goalNode, n, cell), nb});
                    }
                }
            }
        }
        if (!found) return new Plan(Status.BLOCKED, new double[0][], 0, "no legal way to the goal at all");

        List<double[]> pts = new ArrayList<>();
        for (int c = goalNode; c != -1; c = from[c]) pts.add(new double[]{(c % n) * cell, (c / n) * cell});
        Collections.reverse(pts);
        pts.set(0, new double[]{sx, sy});
        pts.set(pts.size() - 1, new double[]{gx, gy});

        // Pull the grid path straight: from each point jump to the farthest point reachable in a straight, safe line.
        List<double[]> way = new ArrayList<>();
        int i = 0;
        double total = 0;
        double[] prev = pts.get(0);
        while (i < pts.size() - 1) {
            int pick = -1;
            for (int j = pts.size() - 1; j > i; j--) {
                double[] a = pts.get(i), b = pts.get(j);
                boolean last = j == pts.size() - 1;
                if (legSafe(a[0], a[1], startH, last ? goalH : startH, b[0], b[1], obs, robotRadiusIn, marginIn, legSlack, legal, i == 0)) {
                    pick = j;
                    break;
                }
            }
            if (pick < 0) return new Plan(Status.BLOCKED, new double[0][], 0, "grid path could not be made into safe straight legs");
            double[] b = pts.get(pick);
            total += Math.hypot(b[0] - prev[0], b[1] - prev[1]);
            prev = b;
            if (pick < pts.size() - 1) way.add(b);
            i = pick;
        }
        double straight = Math.hypot(gx - sx, gy - sy);
        if (total - straight > maxExtraIn) {
            return new Plan(Status.BLOCKED, new double[0][], 0, String.format("the only way round is %.0f in longer than going straight (limit %.0f)", total - straight, maxExtraIn));
        }
        return new Plan(Status.DETOUR, way.toArray(new double[0][]), total - straight,
                String.format("the long way round, %d waypoints, %.0f in longer", way.size(), total - straight));
    }

    private static int nodeIndex(double x, double y, double cell, int n) {
        int i = Math.max(0, Math.min(n - 1, (int) Math.round(x / cell)));
        int j = Math.max(0, Math.min(n - 1, (int) Math.round(y / cell)));
        return j * n + i;
    }

    private static double heuristic(int a, int b, int n, double cell) {
        return cell * Math.hypot(a % n - b % n, a / n - b / n);
    }

    /** One leg of a detour: legal at every point (every 2 in) and not closer to any obstacle than allowed. */
    private static boolean legSafe(double ax, double ay, double h0, double h1, double bx, double by, List<Obstacle> obs,
                                   double robotRadiusIn, double marginIn, double legSlackIn, PoseLegal legal, boolean skipFirstPoint) {
        double len = Math.hypot(bx - ax, by - ay);
        int steps = Math.max(1, (int) Math.ceil(len / 2.0));
        for (int i = skipFirstPoint ? 1 : 0; i <= steps; i++) {
            double f = (double) i / steps;
            if (!legal.ok(ax + (bx - ax) * f, ay + (by - ay) * f, h0 + (h1 - h0) * f)) return false;
        }
        for (Obstacle o : obs) {
            double c = clearance(o, robotRadiusIn, marginIn);
            double allowed = Math.min(c, Math.hypot(o.x - ax, o.y - ay)) - 1e-6 - legSlackIn;
            if (distanceToSegment(o.x, o.y, ax, ay, bx, by) < allowed) return false;
        }
        return true;
    }

    /** Copy of a list, so callers cannot change a plan's inputs while it runs. */
    public static List<Obstacle> copy(List<Obstacle> in) {
        return new ArrayList<>(in);
    }
}
