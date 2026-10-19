package org.firstinspires.ftc.teamcode.util;

import java.util.List;

/**
 * Math for finding and chasing POLLEN with a Limelight colour pipeline. Pure Java, no FTC classes, so it can be unit
 * tested on a laptop.
 *
 * Limelight angles: tx is degrees from the crosshair, positive = the target is to the RIGHT. ty is degrees from the
 * crosshair, positive = the target is HIGHER in the image. Pedro: forward is +, turn is + for counter-clockwise (left).
 */
public final class PollenMath {

    /** POLLEN is a yellow ball about 2.8 in across (manual, scoring elements). */
    public static final double POLLEN_RADIUS_IN = 1.4;

    private PollenMath() {}

    /** One blob the Limelight found: angles in degrees and area as a fraction of the image (0 to 1). */
    public static final class Blob {
        public final double txDeg, tyDeg, area;

        public Blob(double txDeg, double tyDeg, double area) {
            this.txDeg = txDeg;
            this.tyDeg = tyDeg;
            this.area = area;
        }
    }

    /**
     * Distance along the floor from the camera lens to a ball, from the ball's vertical angle in the image.
     *
     * The camera looks pitchDownDeg below horizontal. A target at ty degrees above the crosshair is therefore
     * (pitchDownDeg - ty) degrees below horizontal as seen from the lens, and the ball centre sits
     * (cameraHeightIn - targetCentreHeightIn) below the lens, so distance = drop / tan(angle below horizontal).
     *
     * @return inches, or NaN when the ray does not reach the floor (the ball would be at or above the horizon)
     */
    public static double floorDistanceIn(double cameraHeightIn, double pitchDownDeg, double tyDeg, double targetCentreHeightIn) {
        double belowHorizontal = Math.toRadians(pitchDownDeg - tyDeg);
        double drop = cameraHeightIn - targetCentreHeightIn;
        if (belowHorizontal <= Math.toRadians(0.5) || drop <= 0) return Double.NaN;
        return drop / Math.tan(belowHorizontal);
    }

    /**
     * The nearest floor distance at which the camera can still see a ball: the bottom edge of the image is
     * (pitchDownDeg + half the vertical field of view) below horizontal. Closer than this the ball is out of view.
     *
     * @return inches, or NaN if the geometry does not work (camera not above the ball)
     */
    public static double nearestVisibleDistanceIn(double cameraHeightIn, double pitchDownDeg, double verticalFovDeg, double targetCentreHeightIn) {
        double drop = cameraHeightIn - targetCentreHeightIn;
        double below = Math.toRadians(pitchDownDeg + verticalFovDeg / 2);
        if (drop <= 0 || below <= 0 || below >= Math.toRadians(89.5)) return Double.NaN;
        return drop / Math.tan(below);
    }

    /**
     * Works out the camera pitch from one measurement: a ball placed knownDistanceIn in front of the camera (along the
     * floor, from the lens) reads tyDeg. Inverse of floorDistanceIn.
     *
     * @return degrees below horizontal
     */
    public static double pitchFromKnownDistance(double cameraHeightIn, double knownDistanceIn, double tyDeg, double targetCentreHeightIn) {
        double drop = cameraHeightIn - targetCentreHeightIn;
        return tyDeg + Math.toDegrees(Math.atan2(drop, knownDistanceIn));
    }

    /**
     * Picks which blob to chase. Blobs below minArea are ignored as noise. With no previous target, the nearest ball
     * wins (lowest in the image, ties broken by larger area). With a previous target, a blob within stickyDeg of its
     * last tx is kept, so the robot does not flip between two balls.
     *
     * @return the chosen blob, or null if nothing qualifies
     */
    public static Blob pick(List<Blob> blobs, double minArea, boolean haveLast, double lastTxDeg, double stickyDeg) {
        Blob best = null;
        Blob sticky = null;
        for (Blob b : blobs) {
            if (b.area < minArea) continue;
            if (best == null || b.tyDeg < best.tyDeg - 1e-9 || (Math.abs(b.tyDeg - best.tyDeg) <= 1e-9 && b.area > best.area)) {
                best = b;
            }
            if (haveLast && Math.abs(b.txDeg - lastTxDeg) <= stickyDeg
                    && (sticky == null || Math.abs(b.txDeg - lastTxDeg) < Math.abs(sticky.txDeg - lastTxDeg))) {
                sticky = b;
            }
        }
        return sticky != null ? sticky : best;
    }

    /** Controller settings. Powers are motor power fractions (0 to 1), before Pedro's own max power cap. */
    public static final class Gains {
        /** Turn power per degree of tx. */
        public double kTurn = 0.012;
        public double maxTurn = 0.35;
        /** Smallest turn power that still moves the robot, used only while the error is outside the tolerance. */
        public double minTurn = 0.08;
        /** Stop turning when the target is within this many degrees of the crosshair. */
        public double aimToleranceDeg = 1.5;
        /** Forward power per inch of distance still to cover. */
        public double kForward = 0.02;
        public double maxForward = 0.45;
        public double minForward = 0.12;
        /** Stop when the ball is this far from the camera along the floor, inches. */
        public double stopDistanceIn = 10.0;
        /** Only drive forward when the target is within this many degrees of the crosshair; fades to 0 at the edge. */
        public double alignWindowDeg = 12.0;
        /** Flip the turn direction if the camera is mounted so that tx has the opposite sign. */
        public boolean invertTx = false;
    }

    /** What the controller wants the drivetrain to do this loop. */
    public static final class Command {
        public final double forward, turn;
        public final boolean arrived;

        Command(double forward, double turn, boolean arrived) {
            this.forward = forward;
            this.turn = turn;
            this.arrived = arrived;
        }
    }

    /**
     * Visual servoing: turn to put the ball on the crosshair and drive toward it, slowing as it gets near.
     *
     * @param txDeg      ball angle from the crosshair, degrees, + = right
     * @param distanceIn floor distance from the camera to the ball, inches, or NaN if unknown (then it only turns)
     */
    public static Command command(double txDeg, double distanceIn, Gains g) {
        double tx = g.invertTx ? -txDeg : txDeg;

        // Turn: tx to the right (+) needs a clockwise turn, which is negative in Pedro.
        double turn = 0;
        if (Math.abs(tx) > g.aimToleranceDeg) {
            double mag = Math.min(g.maxTurn, Math.max(g.minTurn, g.kTurn * Math.abs(tx)));
            turn = tx > 0 ? -mag : mag;
        }

        if (Double.isNaN(distanceIn)) return new Command(0, turn, false);

        double remaining = distanceIn - g.stopDistanceIn;
        if (remaining <= 0) return new Command(0, turn, true);

        double fwd = Math.min(g.maxForward, Math.max(g.minForward, g.kForward * remaining));
        double align = Math.max(0, 1 - Math.abs(tx) / g.alignWindowDeg); // 1 when aimed, 0 at the window edge
        return new Command(fwd * align, turn, false);
    }
}
