package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

/**
 * The master switch for the auxiliary systems: the Intake, the Shooter (shoot1, shoot2) and the Adjustor.
 *
 * ENABLED = false (as shipped): none of the three looks up its hardware, so "intake", "shoot1", "shoot2" and "adjustor" do not have to
 * be in the robot configuration, and every command (spin, feed, move) does nothing. The OpModes that use them (the TeleOps, Competition
 * Auto, Shoot On Tags) still run: the drive, the dead wheels and the camera are unaffected.
 *
 * To bring them back: set ENABLED = true (here and rebuild; it is read when an OpMode creates the subsystem, so changing it in Panels
 * takes effect the next time you INIT an OpMode) and put the four devices in the configuration (docs/HARDWARE_CONFIG.md).
 */
@Configurable
public class AuxSettings {
    public static boolean ENABLED = false;
}
