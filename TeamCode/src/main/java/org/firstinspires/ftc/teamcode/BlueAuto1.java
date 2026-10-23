package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Autonomous OpMode: Blue Auto 1
 * Game Field: Biobuzz Field
 * Starting Position: Top Blue Starting Tile
 * Orientation: Front of Robot faces RIGHT on screen (East).
 * Hardware Motors (5 Total):
 * 1. "left_front_drive"  - Control Hub Motor Port 0 (Encoder Port 0)
 * 2. "left_back_drive"   - Control Hub Motor Port 1 (Encoder Port 1)
 * 3. "right_front_drive" - Control Hub Motor Port 2 (Encoder Port 2)
 * 4. "right_back_drive"  - Control Hub Motor Port 3 (Encoder Port 3)
 * 5. "intake_motor"      - Expansion Hub Motor Port 0
 */
@Autonomous(name = "Blue Auto 1", group = "Autonomous")
public class BlueAuto1 extends LinearOpMode {

    // Drive Motors
    private DcMotor leftFrontDrive  = null;
    private DcMotor leftBackDrive   = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive  = null;

    // Intake Motor
    private DcMotor intakeMotor     = null;

    private final ElapsedTime runtime = new ElapsedTime();

    // Encoder Constants (goBILDA 5202 / 19.2:1 with 96mm Mecanum wheels)
    static final double COUNTS_PER_MOTOR_REV    = 537.7;
    static final double DRIVE_GEAR_REDUCTION    = 1.0;
    static final double WHEEL_DIAMETER_INCHES   = 3.77953; // 96mm
    static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                  (WHEEL_DIAMETER_INCHES * Math.PI);

    // Speed Settings
    static final double DRIVE_SPEED  = 0.6;
    static final double INTAKE_POWER = 0.8;

    @Override
    public void runOpMode() {

        // -----------------------------------------------------------------------------------
        // HARDWARE INITIALIZATION
        // -----------------------------------------------------------------------------------
        telemetry.addData("Status", "Initializing Hardware for Blue Auto 1...");
        telemetry.update();

        // Control Hub Motor Ports 0-3
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Expansion Hub Motor Port 0
        intakeMotor     = hardwareMap.get(DcMotor.class, "intake_motor");

        // Motor Directions
        // Front faces RIGHT on screen. Reversing left drive motors ensures forward moves robot Right on screen.
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Zero Power Behavior
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();

        telemetry.addData("Status", "Hardware Ready! Waiting for Start...");
        telemetry.update();

        // -----------------------------------------------------------------------------------
        // WAIT FOR START
        // -----------------------------------------------------------------------------------
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {

            // -------------------------------------------------------------------------------
            // 1. SHOOT 4 PRE-LOADED POLLEN (ESTIMATED TIME ONLY)
            // -------------------------------------------------------------------------------
            telemetry.addData("Auto Phase", "1/8: Shooting 4 Pre-Loaded Pollen");
            telemetry.update();
            estimateShootingSequence(4, "Pre-loaded Pollen"); // ~2.8s estimated time

            // -------------------------------------------------------------------------------
            // 2. MOVE RIGHT INTO THE WALL WHILE RUNNING INTAKE
            // -------------------------------------------------------------------------------
            telemetry.addData("Auto Phase", "2/8: Moving Right into Wall & Intaking");
            telemetry.update();
            setIntakePower(INTAKE_POWER);
            driveInches(DRIVE_SPEED, 28.0, 4.0); // Drive forward (right on screen into wall)

            // -------------------------------------------------------------------------------
            // 3. MOVE TO OTHER SIDE & SHOOT BALLS
            // -------------------------------------------------------------------------------
            telemetry.addData("Auto Phase", "3/8: Moving to Shooting Position & Shooting");
            telemetry.update();
            driveInches(DRIVE_SPEED, -20.0, 4.0); // Move back towards goal alignment
            setIntakePower(0.0);
            estimateShootingSequence(3, "Intaked Pollen from Wall"); // ~2.3s estimated time

            // -------------------------------------------------------------------------------
            // 4. MOVE TO FLOWER AT BOTTOM OF SCREEN & INTAKE
            // -------------------------------------------------------------------------------
            telemetry.addData("Auto Phase", "4/8: Moving to Bottom Flower & Intaking");
            telemetry.update();
            strafeInches(DRIVE_SPEED, 48.0, 5.0); // Strafe right in robot frame = Down on screen
            setIntakePower(INTAKE_POWER);
            driveInches(0.4, 12.0, 3.0); // Drive into flower elements to intake

            // -------------------------------------------------------------------------------
            // 5. MOVE TO OTHER SIDE & SHOOT
            // -------------------------------------------------------------------------------
            telemetry.addData("Auto Phase", "5/8: Returning to Shooting Position & Shooting");
            telemetry.update();
            driveInches(DRIVE_SPEED, -12.0, 3.0);
            strafeInches(DRIVE_SPEED, -48.0, 5.0); // Strafe left in robot frame = Up on screen to goal
            setIntakePower(0.0);
            estimateShootingSequence(3, "Pollen from Bottom Flower"); // ~2.3s estimated time

            // -------------------------------------------------------------------------------
            // 6. MOVE TO FLOWER ON RIGHT OF SCREEN & INTAKE
            // -------------------------------------------------------------------------------
            telemetry.addData("Auto Phase", "6/8: Moving to Right Flower & Intaking");
            telemetry.update();
            strafeInches(DRIVE_SPEED, 24.0, 4.0); // Strafe down towards right flower area
            setIntakePower(INTAKE_POWER);
            driveInches(DRIVE_SPEED, 24.0, 4.0); // Drive forward (right on screen) into flower

            // -------------------------------------------------------------------------------
            // 7. MOVE TO FRONT OF GOAL & SHOOT
            // -------------------------------------------------------------------------------
            telemetry.addData("Auto Phase", "7/8: Moving to Front of Goal & Shooting");
            telemetry.update();
            driveInches(DRIVE_SPEED, -24.0, 4.0); // Drive back away from right wall
            strafeInches(DRIVE_SPEED, -24.0, 4.0); // Move in front of goal
            setIntakePower(0.0);
            estimateShootingSequence(3, "Pollen from Right Flower"); // ~2.3s estimated time

            // -------------------------------------------------------------------------------
            // 8. PARK IN THE BLUE BOX
            // -------------------------------------------------------------------------------
            telemetry.addData("Auto Phase", "8/8: Parking in Blue Box");
            telemetry.update();
            strafeInches(DRIVE_SPEED, -24.0, 4.0); // Strafe up into top blue parking box
            driveInches(DRIVE_SPEED, -10.0, 3.0);  // Align inside parking tile

            stopRobot();
            telemetry.addData("Status", "Blue Auto 1 Routine Finished!");
            telemetry.addData("Total Execution Time", "%.2f sec", runtime.seconds());
            telemetry.update();

            sleep(1000);
        }
    }

    /**
     * Drives the robot forward (+) or backward (-) in inches.
     */
    public void driveInches(double speed, double inches, double timeoutS) {
        encoderDrive(speed, inches, inches, inches, inches, timeoutS);
    }

    /**
     * Strafes the robot right (+) or left (-) in inches.
     * (+ inches = Down on screen, - inches = Up on screen when facing East)
     */
    public void strafeInches(double speed, double inches, double timeoutS) {
        encoderDrive(speed, inches, -inches, -inches, inches, timeoutS);
    }

    /**
     * Standard 4-motor Mecanum encoder drive implementation.
     */
    public void encoderDrive(double speed,
                             double lfInches, double lbInches,
                             double rfInches, double rbInches,
                             double timeoutS) {
        if (!opModeIsActive()) return;

        int newLfTarget = leftFrontDrive.getCurrentPosition()  + (int)(lfInches * COUNTS_PER_INCH);
        int newLbTarget = leftBackDrive.getCurrentPosition()   + (int)(lbInches * COUNTS_PER_INCH);
        int newRfTarget = rightFrontDrive.getCurrentPosition() + (int)(rfInches * COUNTS_PER_INCH);
        int newRbTarget = rightBackDrive.getCurrentPosition()  + (int)(rbInches * COUNTS_PER_INCH);

        leftFrontDrive.setTargetPosition(newLfTarget);
        leftBackDrive.setTargetPosition(newLbTarget);
        rightFrontDrive.setTargetPosition(newRfTarget);
        rightBackDrive.setTargetPosition(newRbTarget);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        leftFrontDrive.setPower(Math.abs(speed));
        leftBackDrive.setPower(Math.abs(speed));
        rightFrontDrive.setPower(Math.abs(speed));
        rightBackDrive.setPower(Math.abs(speed));

        while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
               (leftFrontDrive.isBusy() || leftBackDrive.isBusy() ||
                rightFrontDrive.isBusy() || rightBackDrive.isBusy())) {
            telemetry.addData("Driving", "LF:%d LB:%d RF:%d RB:%d",
                    leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
            telemetry.update();
        }

        stopDriveMotors();

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(100);
    }

    /**
     * Controls intake motor power.
     */
    private void setIntakePower(double power) {
        if (intakeMotor != null) {
            intakeMotor.setPower(power);
        }
    }

    /**
     * Placeholder method to estimate shooter execution time without shooter hardware code.
     */
    private void estimateShootingSequence(int itemCount, String label) {
        double estimatedTime = 0.8 + (itemCount * 0.5);
        telemetry.addData("Shooter (ESTIMATED)", "Shooting %d items (%s) - Delay: %.1fs", itemCount, label, estimatedTime);
        telemetry.update();

        ElapsedTime shootTimer = new ElapsedTime();
        while (opModeIsActive() && shootTimer.seconds() < estimatedTime) {
            idle();
        }
    }

    private void resetEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stopDriveMotors() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void stopRobot() {
        stopDriveMotors();
        setIntakePower(0);
    }
}
