package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.List;

/**
 * Remembers obstacles in FIELD coordinates. The camera only sees what is in front of it, and the robot turns and moves,
 * so an obstacle seen a moment ago must not be forgotten the instant it leaves the picture.
 *
 * Rules that matter:
 *  - NOT SEEING IS NOT THE SAME AS GONE. An obstacle is only forgotten quickly if the camera COULD see the place it is
 *    and sees nothing there (memorySec). An obstacle the camera cannot see (beside the robot, too close to the lens, too
 *    far, off to the side) is kept for much longer (hardMemorySec). Without this, a robot that stops right beside an
 *    obstacle (inside the camera's blind zone) decides it left and drives into it.
 *  - Distance estimates from patch size are noisy, so a new sighting is matched to a remembered obstacle by BEARING from the
 *    robot (tight) and RANGE (loose), not by distance on the field.
 *  - Obstacles seen steadily for a short time count; but one seen CLOSE counts at once, because a false stop costs
 *    a fraction of a second and a missed obstacle costs a collision.
 *  - Sightings inside "ignore" areas (the HIVE and flowers, which carry red and blue) are thrown away.
 *
 * Pure Java, unit tested.
 */
public final class ObstacleTracker {

    /** Could the camera see this field point right now? */
    public interface Visibility {
        boolean inView(double x, double y);
    }

    private static final class Track {
        double x, y, firstSeen, lastSeen;
        int hits;
        boolean immediate;
    }

    private final double matchRadiusIn, confirmSec, memorySec, hardMemorySec, obstacleRadiusIn, immediateRangeIn;
    private final List<double[]> ignoreRects;
    private final List<Track> tracks = new ArrayList<>();

    /**
     * @param matchRadiusIn    sideways tolerance when matching a sighting to a remembered obstacle, inches
     * @param confirmSec       an obstacle only counts after it has been seen for this long (and at least twice)
     * @param memorySec        an obstacle the camera could see but does not is forgotten after this long
     * @param hardMemorySec    an obstacle the camera cannot see is forgotten after this long
     * @param obstacleRadiusIn radius given to every obstacle (an opposing robot is up to about 18 in across)
     * @param immediateRangeIn a sighting this close to the robot counts at once; 0 turns this off
     * @param ignoreRects      areas {xMin, yMin, xMax, yMax} in which sightings are ignored
     */
    public ObstacleTracker(double matchRadiusIn, double confirmSec, double memorySec, double hardMemorySec,
                           double obstacleRadiusIn, double immediateRangeIn, List<double[]> ignoreRects) {
        this.matchRadiusIn = matchRadiusIn;
        this.confirmSec = confirmSec;
        this.memorySec = memorySec;
        this.hardMemorySec = Math.max(hardMemorySec, memorySec);
        this.obstacleRadiusIn = obstacleRadiusIn;
        this.immediateRangeIn = immediateRangeIn;
        this.ignoreRects = ignoreRects;
    }

    /** Older form: no separate hard memory and no immediate confirmation. */
    public ObstacleTracker(double matchRadiusIn, double confirmSec, double memorySec, double obstacleRadiusIn, List<double[]> ignoreRects) {
        this(matchRadiusIn, confirmSec, memorySec, memorySec, obstacleRadiusIn, 0, ignoreRects);
    }

    /** True if a point is inside one of the ignore areas. */
    public boolean isIgnored(double x, double y) {
        for (double[] r : ignoreRects) {
            if (x >= r[0] && x <= r[2] && y >= r[1] && y <= r[3]) return true;
        }
        return false;
    }

    /** Reports one sighting at a field position, matched by field distance only. Returns false if it was ignored. */
    public boolean observe(double now, double x, double y) {
        if (isIgnored(x, y)) return false;
        Track best = null;
        double bestD = matchRadiusIn;
        for (Track t : tracks) {
            double d = Math.hypot(t.x - x, t.y - y);
            if (d <= bestD) {
                bestD = d;
                best = t;
            }
        }
        return record(now, best, x, y, Double.NaN);
    }

    /**
     * Reports one sighting seen from a robot at (rx, ry). It is matched to a remembered obstacle by bearing (within a
     * sideways distance of matchRadiusIn at the nearer range) and by range (within half the distance, at least matchRadiusIn),
     * because the range estimate is the noisy part. Returns false if it was ignored.
     */
    public boolean observe(double now, double rx, double ry, double x, double y) {
        if (isIgnored(x, y)) return false;
        double ds = Math.hypot(x - rx, y - ry), bs = Math.atan2(y - ry, x - rx);
        Track best = null;
        double bestScore = Double.MAX_VALUE;
        for (Track t : tracks) {
            double dt = Math.hypot(t.x - rx, t.y - ry), bt = Math.atan2(t.y - ry, t.x - rx);
            double db = Math.abs(Math.atan2(Math.sin(bs - bt), Math.cos(bs - bt)));
            double lateral = Math.min(ds, dt) * Math.sin(Math.min(db, Math.PI / 2));
            double range = Math.abs(ds - dt);
            double rangeTol = Math.max(matchRadiusIn, 0.5 * Math.max(ds, dt));
            if (db < Math.PI / 2 && lateral <= matchRadiusIn && range <= rangeTol) {
                double score = lateral + 0.1 * range;
                if (score < bestScore) {
                    bestScore = score;
                    best = t;
                }
            }
        }
        return record(now, best, x, y, ds);
    }

    private boolean record(double now, Track best, double x, double y, double distFromRobot) {
        if (best == null) {
            best = new Track();
            best.x = x;
            best.y = y;
            best.firstSeen = now;
            tracks.add(best);
        } else {
            best.x = 0.5 * best.x + 0.5 * x; // smooth the noise in the estimate
            best.y = 0.5 * best.y + 0.5 * y;
        }
        best.lastSeen = now;
        best.hits++;
        if (immediateRangeIn > 0 && !Double.isNaN(distFromRobot) && distFromRobot <= immediateRangeIn) best.immediate = true;
        return true;
    }

    /** Drops obstacles not seen for memorySec, treating every obstacle as if the camera could see it. */
    public void expire(double now) {
        expire(now, null);
    }

    /**
     * Drops obstacles. One the camera could see (per the visibility test) but has not seen for memorySec is gone. One it could
     * NOT see is kept until hardMemorySec. With no visibility test every obstacle counts as visible.
     */
    public void expire(double now, Visibility visibility) {
        for (int i = tracks.size() - 1; i >= 0; i--) {
            Track t = tracks.get(i);
            double unseen = now - t.lastSeen;
            boolean couldSee = visibility == null || visibility.inView(t.x, t.y);
            if (unseen > hardMemorySec || (couldSee && unseen > memorySec)) tracks.remove(i);
        }
    }

    /** The obstacles that count: seen steadily, or seen close, and not yet forgotten. */
    public List<ObstacleMath.Obstacle> active(double now) {
        List<ObstacleMath.Obstacle> out = new ArrayList<>();
        for (Track t : tracks) {
            boolean confirmed = t.immediate || (t.hits >= 2 && t.lastSeen - t.firstSeen >= confirmSec);
            if (confirmed && now - t.lastSeen <= hardMemorySec) out.add(new ObstacleMath.Obstacle(t.x, t.y, obstacleRadiusIn));
        }
        return out;
    }

    /** Forget everything. */
    public void clear() {
        tracks.clear();
    }

    public int trackCount() {
        return tracks.size();
    }
}
