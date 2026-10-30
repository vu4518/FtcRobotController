package example.team;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * Blue Auto 1 (Biobuzz) ported from the FTC LinearOpMode to the robotarena adapter.
 *
 * The adapter calls update() over and over, so the routine is a list of steps.
 * Each call runs the current step for one tick; when that step is finished
 * (distance reached, balls empty, or timeout) it moves to the next step.
 *
 * Moves are robot-relative like the original: forward/back and strafe, measured
 * by actual distance traveled from the robot's pose. Heading is held at whatever
 * direction the robot started facing.
 *
 * SAVE THIS FILE AS: src\example\team\ExampleAuto.java
 */
public final class ExampleAuto {

  // ---- Records must stay exactly like this: MyAuto.java builds and reads them. ----
  public record Sensors(double timeSeconds, double sampledAtSeconds,
      double xInches, double yInches, double clockwiseDegrees,
      boolean odometryValid, boolean cameraConnected, int ballCount,
      List<Boolean> occupiedSlots, Map<String, Object> vision) {}

  public record Motors(double frontLeft, double frontRight, double backLeft,
      double backRight, boolean intake, boolean shooter, boolean feeder,
      double pitchDegrees, double launchPower) {}

  // ---- Tuning ----
  private static final double DRIVE_SPEED     = 0.6;
  private static final double SLOW_SPEED      = 0.4;
  private static final double MIN_POWER       = 0.15; // never crawl slower than this
  private static final double SLOWDOWN_INCHES = 6.0;  // start easing off this far from target
  private static final double ARRIVE_INCHES   = 0.5;  // close enough = step done
  private static final double HEADING_KP      = 0.01; // heading hold; set to 0 to turn off,
                                                      // flip the sign if the robot spins away
  private static final double MAX_TURN        = 0.3;
  private static final double SPIN_UP_SECONDS = 0.8;  // shooter spins this long before feeding
  private static final double SECONDS_PER_BALL = 0.5;
  private static final double PITCH_DEGREES   = 66.5;
  private static final double LAUNCH_POWER    = 0.65;

  private enum Kind { MOVE, SHOOT }

  private record Step(Kind kind, String label, double forwardIn, double strafeIn,
      double power, double timeoutS, boolean intake) {}

  private final List<Step> steps = new ArrayList<>();
  private int index = 0;
  private boolean stepStarted = false;
  private double stepStartX, stepStartY, stepStartTime;
  private boolean headingSet = false;
  private double holdHeading;

  public ExampleAuto() {
    // forward + = robot forward, strafe + = robot right (same as the FTC version)

    // 1. Shoot 4 preloaded pollen
    shoot("1/8 Shoot preload", 4);

    // 2. Drive into the wall while intaking
    move("2/8 Into wall + intake",       28,   0, DRIVE_SPEED, 4, true);

    // 3. Back to shooting position, shoot
    move("3/8 Back to goal",            -20,   0, DRIVE_SPEED, 4, true);
    shoot("3/8 Shoot wall pollen", 3);

    // 4. Bottom flower, intake
    move("4/8 Strafe to bottom flower",   0,  48, DRIVE_SPEED, 5, false);
    move("4/8 Into bottom flower",       12,   0, SLOW_SPEED,  3, true);

    // 5. Back to goal, shoot
    move("5/8 Back out of flower",      -12,   0, DRIVE_SPEED, 3, true);
    move("5/8 Strafe up to goal",         0, -48, DRIVE_SPEED, 5, true);
    shoot("5/8 Shoot bottom flower pollen", 3);

    // 6. Right flower, intake
    move("6/8 Strafe toward right flower", 0, 24, DRIVE_SPEED, 4, false);
    move("6/8 Into right flower",        24,   0, DRIVE_SPEED, 4, true);

    // 7. Front of goal, shoot
    move("7/8 Back from right wall",    -24,   0, DRIVE_SPEED, 4, true);
    move("7/8 Strafe to front of goal",   0, -24, DRIVE_SPEED, 4, true);
    shoot("7/8 Shoot right flower pollen", 3);

    // 8. Park in blue box
    move("8/8 Strafe up to blue box",     0, -24, DRIVE_SPEED, 4, false);
    move("8/8 Align in tile",           -10,   0, DRIVE_SPEED, 3, false);
  }

  public Motors update(Sensors s) {
    // Bad or stale sensors: stop for this tick, pick up again when they come back.
    double age = s.timeSeconds() - s.sampledAtSeconds();
    if (!s.odometryValid() || !Double.isFinite(age) || age < 0 || age > .3
        || !Double.isFinite(s.xInches()) || !Double.isFinite(s.yInches())
        || !Double.isFinite(s.clockwiseDegrees())) {
      return stopped(false);
    }

    if (!headingSet) {
      holdHeading = s.clockwiseDegrees();
      headingSet = true;
    }

    if (index >= steps.size()) {
      return stopped(false); // routine finished: hold still
    }

    Step step = steps.get(index);
    if (!stepStarted) {
      stepStartX = s.xInches();
      stepStartY = s.yInches();
      stepStartTime = s.timeSeconds();
      stepStarted = true;
      System.err.println("[auto] " + step.label());
    }
    double elapsed = s.timeSeconds() - stepStartTime;

    // ---- SHOOT: spin up, then feed until empty or out of time ----
    if (step.kind() == Kind.SHOOT) {
      if (s.ballCount() <= 0 || elapsed >= step.timeoutS()) {
        nextStep();
        return stopped(false);
      }
      boolean feed = elapsed >= SPIN_UP_SECONDS;
      return new Motors(0, 0, 0, 0, false, true, feed, PITCH_DEGREES, LAUNCH_POWER);
    }

    // ---- MOVE: drive until we've traveled the distance or timed out ----
    double target = Math.hypot(step.forwardIn(), step.strafeIn());
    double traveled = Math.hypot(s.xInches() - stepStartX, s.yInches() - stepStartY);
    double remaining = target - traveled;

    if (remaining <= ARRIVE_INCHES || elapsed >= step.timeoutS()) {
      nextStep();
      return stopped(step.intake());
    }

    double power = Math.max(MIN_POWER,
        step.power() * Math.min(1.0, remaining / SLOWDOWN_INCHES));
    double fwd = power * step.forwardIn() / target;
    double str = power * step.strafeIn() / target;
    double turn = clamp(HEADING_KP * wrap(holdHeading - s.clockwiseDegrees()),
        -MAX_TURN, MAX_TURN);

    // Mecanum mix (turn + = clockwise)
    double fl = fwd + str + turn;
    double fr = fwd - str - turn;
    double bl = fwd - str + turn;
    double br = fwd + str - turn;

    double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
        Math.max(Math.abs(bl), Math.abs(br))));

    return new Motors(fl / max, fr / max, bl / max, br / max,
        step.intake(), false, false, PITCH_DEGREES, LAUNCH_POWER);
  }

  // ---- helpers ----
  private void move(String label, double forwardIn, double strafeIn,
      double power, double timeoutS, boolean intake) {
    steps.add(new Step(Kind.MOVE, label, forwardIn, strafeIn, power, timeoutS, intake));
  }

  private void shoot(String label, int balls) {
    steps.add(new Step(Kind.SHOOT, label, 0, 0, 0,
        SPIN_UP_SECONDS + balls * SECONDS_PER_BALL, false));
  }

  private void nextStep() {
    index++;
    stepStarted = false;
  }

  private Motors stopped(boolean intake) {
    return new Motors(0, 0, 0, 0, intake, false, false, PITCH_DEGREES, LAUNCH_POWER);
  }

  private static double wrap(double deg) {
    double d = deg % 360.0;
    if (d > 180) d -= 360;
    if (d < -180) d += 360;
    return d;
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }
}
