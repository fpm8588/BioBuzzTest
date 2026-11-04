package org.firstinspires.ftc.teamcode.util;

import java.util.List;

/**
 * The arithmetic for aiming at an AprilTag and measuring the range to it. No hardware in here, so it is unit tested on a
 * laptop.
 *
 * Conventions:
 *  - tx is degrees to the RIGHT of the crosshair (positive = target is to the right), ty is degrees ABOVE it.
 *  - The camera pitch is degrees BELOW horizontal (a camera tilted down is positive), as in PollenSettings.
 *  - Camera space is the Limelight's: x to the right, y DOWN, z forward along the lens axis. All lengths here are inches.
 *  - Turn commands are Pedro's: positive = counter-clockwise (left). A target on the right needs a negative turn.
 */
public final class TagMath {

    private TagMath() {}

    /** One AprilTag the camera sees: its ID, where it is in the picture, and (if known) where it is in camera space. */
    public static final class Tag {
        public final int id;
        public final double txDeg, tyDeg, area;
        /** Position of the tag centre in camera space, inches; NaN if the Limelight did not give a pose. */
        public final double camXIn, camYIn, camZIn;

        public Tag(int id, double txDeg, double tyDeg, double area, double camXIn, double camYIn, double camZIn) {
            this.id = id;
            this.txDeg = txDeg;
            this.tyDeg = tyDeg;
            this.area = area;
            this.camXIn = camXIn;
            this.camYIn = camYIn;
            this.camZIn = camZIn;
        }

        public boolean hasPose() {
            return !Double.isNaN(camXIn) && !Double.isNaN(camYIn) && !Double.isNaN(camZIn);
        }
    }

    /**
     * The tag to aim at. wantedId below 0 means any tag (the biggest in the picture, which is the nearest); otherwise only tags with
     * that ID count, and the biggest of them wins.
     *
     * @return the chosen tag, or null if there is none
     */
    public static Tag pick(List<Tag> tags, int wantedId) {
        Tag best = null;
        for (Tag t : tags) {
            if (wantedId >= 0 && t.id != wantedId) continue;
            if (best == null || t.area > best.area) best = t;
        }
        return best;
    }

    // ---- a strip of tags ----

    /**
     * Where each tag of a scoring strip sits along it, measured from the strip's centre in tag widths (negative = to the left as you
     * face the strip), read off the printed strips: two tags, a gap, two tags. The IDs run left to right (30 31 | 32 33 for red).
     */
    public static final double[] STRIP_OFFSETS = {-2.0, -0.845, 0.845, 2.0};

    /** What the camera makes of a strip of tags: where its centre is, and how far away. */
    public static final class Strip {
        /** Degrees the strip's centre is to the right of the crosshair (average over the tags in view). */
        public final double txDeg;
        /** Degrees the strip is above the crosshair (average over the tags in view). */
        public final double tyDeg;
        /** Distance along the floor from the camera to the strip's centre, inches; NaN if no tag in view has a pose. */
        public final double poseDistanceIn;
        /** The biggest tag of the strip in view. */
        public final int bestId;
        public final double bestArea;
        public final int tagsUsed;
        /** true if every tag was corrected to the strip's centre; false if some were taken as they are (no tag size, or no pose). */
        public final boolean centred;

        Strip(double txDeg, double tyDeg, double poseDistanceIn, int bestId, double bestArea, int tagsUsed, boolean centred) {
            this.txDeg = txDeg;
            this.tyDeg = tyDeg;
            this.poseDistanceIn = poseDistanceIn;
            this.bestId = bestId;
            this.bestArea = bestArea;
            this.tagsUsed = tagsUsed;
            this.centred = centred;
        }
    }

    /**
     * Combines the tags in view that belong to a strip (IDs firstId to firstId + STRIP_OFFSETS.length - 1) into one aim point: the
     * strip's CENTRE. Each tag is a known number of tag widths from the centre, so from its camera-space position the centre is
     * at lateral (x - offset * tagSize), at the same forward distance, assuming the robot faces the strip squarely; the centre
     * directions of all tags in view are averaged. With no tag size (tagSizeIn <= 0) or no pose, a tag is taken as it is, which
     * aims at the middle of the tags in view instead of the middle of the strip.
     *
     * @return null if no tag of the strip is in view
     */
    public static Strip strip(List<Tag> tags, int firstId, double tagSizeIn, double pitchDownDeg) {
        double txSum = 0, tySum = 0, distSum = 0;
        int n = 0, withDist = 0;
        boolean centred = true;
        Tag best = null;
        for (Tag t : tags) {
            int k = t.id - firstId;
            if (k < 0 || k >= STRIP_OFFSETS.length) continue;
            double tx = t.txDeg;
            if (tagSizeIn > 0 && t.hasPose()) {
                double p = Math.toRadians(pitchDownDeg);
                double forward = t.camZIn * Math.cos(p) - t.camYIn * Math.sin(p);
                double lateral = t.camXIn - STRIP_OFFSETS[k] * tagSizeIn;
                tx = Math.toDegrees(Math.atan2(lateral, forward));
                distSum += Math.hypot(lateral, forward);
                withDist++;
            } else {
                centred = false;
                if (t.hasPose()) {
                    distSum += poseDistanceIn(t.camXIn, t.camYIn, t.camZIn, pitchDownDeg);
                    withDist++;
                }
            }
            txSum += tx;
            tySum += t.tyDeg;
            n++;
            if (best == null || t.area > best.area) best = t;
        }
        if (n == 0) return null;
        return new Strip(txSum / n, tySum / n, withDist == 0 ? Double.NaN : distSum / withDist, best.id, best.area, n, centred);
    }

    // ---- range ----

    /**
     * Distance along the floor from the camera to the tag, from the tag's position in camera space. The lens axis is pitchDownDeg
     * below horizontal, so the forward-along-the-floor part of the tag's position is z*cos(pitch) - y*sin(pitch) (y is down),
     * and the sideways part is x.
     */
    public static double poseDistanceIn(double xIn, double yIn, double zIn, double pitchDownDeg) {
        double p = Math.toRadians(pitchDownDeg);
        double forward = zIn * Math.cos(p) - yIn * Math.sin(p);
        return Math.hypot(xIn, forward);
    }

    /** How far above the lens the tag centre is, from its camera-space position (negative = below the lens). */
    public static double poseHeightAboveCameraIn(double yIn, double zIn, double pitchDownDeg) {
        double p = Math.toRadians(pitchDownDeg);
        return -yIn * Math.cos(p) - zIn * Math.sin(p);
    }

    /**
     * Distance along the floor from the camera to the tag, from the tag's vertical angle in the picture alone. The tag is
     * (ty - pitchDownDeg) degrees above horizontal as seen from the lens, and its centre is (tagCentreHeightIn - cameraHeightIn)
     * above the lens, so distance = rise / tan(angle). Needs the tag's real height above the floor.
     *
     * @return inches, or NaN if the tag is not above the lens or is within half a degree of the horizon
     */
    public static double tyDistanceIn(double tyDeg, double cameraHeightIn, double pitchDownDeg, double tagCentreHeightIn) {
        double rise = tagCentreHeightIn - cameraHeightIn;
        double above = Math.toRadians(tyDeg - pitchDownDeg);
        if (rise <= 0 || above <= Math.toRadians(0.5)) return Double.NaN;
        return rise / Math.tan(above);
    }

    /**
     * The nearest distance at which the tag centre is still inside the picture. The top edge of the picture is
     * (verticalFovDeg / 2 - pitchDownDeg) above horizontal; a tag higher than the lens leaves the top of the picture when the
     * robot gets closer than this.
     *
     * @return inches; +infinity if the top edge is at or below the horizon (a tag above the lens is never in view); NaN if the tag
     *         is not above the lens
     */
    public static double nearestVisibleDistanceIn(double cameraHeightIn, double pitchDownDeg, double verticalFovDeg, double tagCentreHeightIn) {
        double rise = tagCentreHeightIn - cameraHeightIn;
        if (rise <= 0) return Double.NaN;
        double top = Math.toRadians(verticalFovDeg / 2 - pitchDownDeg);
        if (top <= Math.toRadians(0.5)) return Double.POSITIVE_INFINITY;
        return rise / Math.tan(top);
    }

    // ---- aiming ----

    public static final class AimGains {
        /** Turn power per degree of error. */
        public double kTurn = 0.018;
        public double maxTurn = 0.6;
        /** Smallest turn power that moves the robot at all, used outside the tolerance. */
        public double minTurn = 0.08;
        /** Inside this many degrees of error the robot counts as aimed and does not turn. */
        public double toleranceDeg = 1.5;
        /** Where the crosshair should be relative to the tag, degrees to the right (the shooter is not in line with the camera). */
        public double offsetDeg = 0.0;
        /** true if turning goes the wrong way. */
        public boolean invert = false;
    }

    private static double error(double txDeg, AimGains g) {
        double e = txDeg - g.offsetDeg;
        return g.invert ? -e : e;
    }

    /** Degrees the robot still has to turn to be aimed (positive = the tag is to the right). */
    public static double aimErrorDeg(double txDeg, AimGains g) {
        return error(txDeg, g);
    }

    public static boolean aimed(double txDeg, AimGains g) {
        return Math.abs(error(txDeg, g)) <= g.toleranceDeg;
    }

    /** Turn command (Pedro: positive = counter-clockwise) that brings the tag to the crosshair; 0 once aimed. */
    public static double aimTurn(double txDeg, AimGains g) {
        double e = error(txDeg, g);
        if (Math.abs(e) <= g.toleranceDeg) return 0;
        double mag = Math.max(g.minTurn, Math.min(g.maxTurn, g.kTurn * Math.abs(e)));
        return e > 0 ? -mag : mag;
    }
}
