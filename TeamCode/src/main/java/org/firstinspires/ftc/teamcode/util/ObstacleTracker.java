package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.List;

/**
 * Remembers obstacles in FIELD coordinates. The camera only sees what is in front of it, and the robot turns and moves,
 * so an obstacle seen a moment ago must not be forgotten the instant it leaves the picture. Each sighting is matched to a
 * remembered obstacle (or starts a new one), only obstacles seen steadily for a short time count, and obstacles not seen
 * for a while are dropped. Sightings inside "ignore" areas (the HIVE and flowers, which have alliance colours on them and
 * are on the field anyway) are thrown away.
 *
 * Pure Java, unit tested.
 */
public final class ObstacleTracker {

    private static final class Track {
        double x, y, firstSeen, lastSeen;
        int hits;
    }

    private final double matchRadiusIn, confirmSec, memorySec, obstacleRadiusIn;
    private final List<double[]> ignoreRects;
    private final List<Track> tracks = new ArrayList<>();

    /**
     * @param matchRadiusIn    a sighting this close to a remembered obstacle is the same obstacle
     * @param confirmSec       an obstacle only counts after it has been seen for this long (and at least twice)
     * @param memorySec        an obstacle not seen for this long is forgotten
     * @param obstacleRadiusIn radius given to every obstacle (an opposing robot is up to about 18 in across)
     * @param ignoreRects      areas {xMin, yMin, xMax, yMax} in which sightings are ignored
     */
    public ObstacleTracker(double matchRadiusIn, double confirmSec, double memorySec, double obstacleRadiusIn, List<double[]> ignoreRects) {
        this.matchRadiusIn = matchRadiusIn;
        this.confirmSec = confirmSec;
        this.memorySec = memorySec;
        this.obstacleRadiusIn = obstacleRadiusIn;
        this.ignoreRects = ignoreRects;
    }

    /** True if a point is inside one of the ignore areas. */
    public boolean isIgnored(double x, double y) {
        for (double[] r : ignoreRects) {
            if (x >= r[0] && x <= r[2] && y >= r[1] && y <= r[3]) return true;
        }
        return false;
    }

    /** Reports one sighting at a field position. Returns false if it was ignored. */
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
        return true;
    }

    /** Drops obstacles not seen for memorySec. Call once per loop. */
    public void expire(double now) {
        for (int i = tracks.size() - 1; i >= 0; i--) {
            if (now - tracks.get(i).lastSeen > memorySec) tracks.remove(i);
        }
    }

    /** The obstacles that have been seen steadily and not yet forgotten. */
    public List<ObstacleMath.Obstacle> active(double now) {
        List<ObstacleMath.Obstacle> out = new ArrayList<>();
        for (Track t : tracks) {
            if (now - t.lastSeen <= memorySec && t.hits >= 2 && t.lastSeen - t.firstSeen >= confirmSec) {
                out.add(new ObstacleMath.Obstacle(t.x, t.y, obstacleRadiusIn));
            }
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
