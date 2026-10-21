package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.List;

/**
 * A route leg as a chain of short straight pieces, so the obstacle code (which works with straight lines) can reason about a
 * curve: how far the robot is off it, which part of it is still ahead, where it can rejoin it after a detour.
 *
 * A straight leg is a two point polyline. A Bezier curve is sampled at equal steps of its parameter t, and the
 * polyline remembers each sample's t, so a point found on the polyline can be turned back into an exact piece of the curve
 * (split()). The sampling is fine enough (a few tenths of an inch) that the polyline is the curve for every purpose here.
 *
 * Pure Java, unit tested.
 */
public final class RoutePolyline {

    /** Points {x, y}, in driving order. */
    public final double[][] pts;
    /** The Bezier parameter t of each point (0 to 1); for a straight leg just 0 and 1. */
    public final double[] ts;
    /** Distance along the polyline to each point, inches. */
    private final double[] cum;

    private RoutePolyline(double[][] pts, double[] ts) {
        this.pts = pts;
        this.ts = ts;
        this.cum = new double[pts.length];
        for (int i = 1; i < pts.length; i++) cum[i] = cum[i - 1] + Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]);
    }

    /** A straight line. */
    public static RoutePolyline line(double x0, double y0, double x1, double y1) {
        return new RoutePolyline(new double[][]{{x0, y0}, {x1, y1}}, new double[]{0, 1});
    }

    /**
     * A Bezier curve of any degree (two or more control points) sampled at samples + 1 points.
     * @param control {x, y} of each control point; the first and last are the ends of the curve
     */
    public static RoutePolyline bezier(double[][] control, int samples) {
        if (control.length == 2) return line(control[0][0], control[0][1], control[1][0], control[1][1]);
        int n = Math.max(2, samples);
        double[][] p = new double[n + 1][];
        double[] t = new double[n + 1];
        for (int i = 0; i <= n; i++) {
            t[i] = (double) i / n;
            p[i] = bezierPoint(control, t[i]);
        }
        return new RoutePolyline(p, t);
    }

    /** de Casteljau: the point of a Bezier curve at parameter t. */
    public static double[] bezierPoint(double[][] control, double t) {
        double[][] level = copy(control);
        for (int n = level.length - 1; n > 0; n--) {
            for (int i = 0; i < n; i++) {
                level[i][0] = (1 - t) * level[i][0] + t * level[i + 1][0];
                level[i][1] = (1 - t) * level[i][1] + t * level[i + 1][1];
            }
        }
        return level[0];
    }

    /**
     * The part of a Bezier curve from parameter t to its end, as the control points of an exactly equivalent curve
     * (de Casteljau subdivision): the first point is the curve's point at t, the last is still the curve's end.
     */
    public static double[][] splitFrom(double[][] control, double t) {
        int degree = control.length - 1;
        double[][] level = copy(control);
        double[][] right = new double[control.length][];
        right[degree] = level[degree].clone();                       // the last point of the first level is the last control point
        for (int n = degree; n > 0; n--) {
            for (int i = 0; i < n; i++) {
                level[i][0] = (1 - t) * level[i][0] + t * level[i + 1][0];
                level[i][1] = (1 - t) * level[i][1] + t * level[i + 1][1];
            }
            right[n - 1] = level[n - 1].clone();                     // the last point of each level, in order from the end
        }
        return right;
    }

    private static double[][] copy(double[][] a) {
        double[][] c = new double[a.length][];
        for (int i = 0; i < a.length; i++) c[i] = a[i].clone();
        return c;
    }

    // ---- measuring ----

    public double length() {
        return cum[cum.length - 1];
    }

    public double[] start() {
        return pts[0];
    }

    public double[] end() {
        return pts[pts.length - 1];
    }

    /** The closest point on the polyline to (x, y): {distance from (x, y), distance along the polyline, index of the segment}. */
    private double[] closest(double x, double y) {
        double bestD = Double.MAX_VALUE, bestS = 0;
        int bestI = 0;
        for (int i = 0; i + 1 < pts.length; i++) {
            double ax = pts[i][0], ay = pts[i][1], dx = pts[i + 1][0] - ax, dy = pts[i + 1][1] - ay;
            double len2 = dx * dx + dy * dy;
            double f = len2 == 0 ? 0 : Math.max(0, Math.min(1, ((x - ax) * dx + (y - ay) * dy) / len2));
            double d = Math.hypot(x - (ax + f * dx), y - (ay + f * dy));
            if (d < bestD) {
                bestD = d;
                bestS = cum[i] + f * Math.sqrt(len2);
                bestI = i;
            }
        }
        return new double[]{bestD, bestS, bestI};
    }

    /** How far (x, y) is from the polyline (from its nearest point, ends included), inches. */
    public double distanceTo(double x, double y) {
        return closest(x, y)[0];
    }

    /** Distance along the polyline of its point nearest to (x, y), inches. */
    public double progressAt(double x, double y) {
        return closest(x, y)[1];
    }

    /** The point a given distance along the polyline (clamped to its ends), with its Bezier parameter t: {x, y, t}. */
    public double[] pointAt(double s) {
        if (s <= 0) return new double[]{pts[0][0], pts[0][1], ts[0]};
        if (s >= length()) return new double[]{end()[0], end()[1], ts[ts.length - 1]};
        int i = 0;
        while (i + 2 < pts.length && cum[i + 1] < s) i++;
        double seg = cum[i + 1] - cum[i];
        double f = seg == 0 ? 0 : (s - cum[i]) / seg;
        return new double[]{pts[i][0] + f * (pts[i + 1][0] - pts[i][0]), pts[i][1] + f * (pts[i + 1][1] - pts[i][1]), ts[i] + f * (ts[i + 1] - ts[i])};
    }

    /** Heading fraction (0 to 1) at a distance along the polyline: how far through the leg a linear heading change has got. */
    public double fractionAt(double s) {
        return length() == 0 ? 1 : Math.max(0, Math.min(1, s / length()));
    }

    /**
     * What is still ahead of a robot at (x, y), as a list of points for straight-line tests. A straight leg gives just its end
     * (one straight piece, as the obstacle code has always treated it); a curve gives points about spacingIn apart, from the
     * robot's nearest point on the curve to the end.
     */
    public List<double[]> remaining(double x, double y, double spacingIn) {
        List<double[]> out = new ArrayList<>();
        if (pts.length == 2) {
            out.add(end());
            return out;
        }
        double s = progressAt(x, y);
        double next = s + spacingIn;
        while (next < length() - 1e-9) {
            double[] p = pointAt(next);
            out.add(new double[]{p[0], p[1]});
            next += spacingIn;
        }
        out.add(end());
        return out;
    }

    /** The point lookIn further along the polyline than the robot's nearest point (the end if there is less than that left). */
    public double[] ahead(double x, double y, double lookIn) {
        double[] p = pointAt(progressAt(x, y) + lookIn);
        return new double[]{p[0], p[1]};
    }

    /**
     * Where a robot at (x, y) can rejoin the polyline after going round obstacles: the first point, at least bufferIn past the
     * LAST stretch of the polyline that comes within clearanceIn of an obstacle centre (and no nearer than minAheadIn in front of
     * the robot). Returns {x, y, t, distanceAlong}, or null if that would be at (or past) the end, i.e. go to the end instead.
     *
     * @param obstacles {x, y, clearance} of each obstacle; clearance is the distance the robot's centre must keep from it
     */
    public double[] rejoinPoint(double x, double y, List<double[]> obstacles, double bufferIn, double minAheadIn) {
        double here = progressAt(x, y);
        double lastBlocked = Double.NEGATIVE_INFINITY;   // nothing near the path ahead: only the minimum distance applies
        for (double s = here; s <= length(); s += 0.5) {
            double[] p = pointAt(s);
            for (double[] o : obstacles) {
                if (Math.hypot(p[0] - o[0], p[1] - o[1]) < o[2]) lastBlocked = s;
            }
        }
        double target = Math.max(lastBlocked + bufferIn, here + minAheadIn);
        if (target >= length() - 2.0) return null;
        double[] p = pointAt(target);
        return new double[]{p[0], p[1], p[2], target};
    }
}
