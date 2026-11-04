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
