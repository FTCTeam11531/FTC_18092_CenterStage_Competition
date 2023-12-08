package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.system.sysIntakeArm;
import org.firstinspires.ftc.teamcode.system.sysLighting;
import org.firstinspires.ftc.teamcode.system.sysDrivetrainMecanum;
import org.firstinspires.ftc.teamcode.system.sysVision;
import org.firstinspires.ftc.teamcode.utility.enumStateDriveMotorMaxOutputPower;
import org.firstinspires.ftc.teamcode.utility.enumStateDrivetrainMode;
import org.firstinspires.ftc.teamcode.utility.utilRobotConstants;

/**
 * <h2>OpMode - Teleop - Main</h2>
 * <hr>
 * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
 * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
 * <hr>
 * <p>
 * Teleop Mode - Main
 * One OpMode to Rule them All
 * </p>
 * <br>
 */
@TeleOp(name="Teleop Main", group="_main")
//@Disabled
public class opmodeTeleopMain extends LinearOpMode {
    // ------------------------------------------------------------
    // System(s) - Define system and create instance of each system
    // ------------------------------------------------------------
    // -- Robot Initialization

    // -- Lighting System
    sysLighting sysLighting = new sysLighting(this);

    // -- Drivetrain System
    sysDrivetrainMecanum sysDrivetrain = new sysDrivetrainMecanum(this);

    // Vision System
    sysVision sysVision = new sysVision(this);

    // Intake / Arm
    sysIntakeArm sysIntakeArm = new sysIntakeArm(this);

    // ------------------------------------------------------------
    // Misc
    // ------------------------------------------------------------
    // -- Command Runtime
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // ------------------------------------------------------------
        // Configure Telemetry
        // ------------------------------------------------------------
        // Set telemetry mode to append
        telemetry.setAutoClear(false);
        telemetry.clearAll();

        // ------------------------------------------------------------
        // Initialize System(s) - set different light mode between each system init
        // ------------------------------------------------------------

        // Initialize System(s)
        sysLighting.init(utilRobotConstants.Configuration.ENABLE_LIGHTING);
        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_LIGHTING);

        sysDrivetrain.init();
        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_DRIVETRAIN);

        sysVision.init();
        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_VISION);

        sysIntakeArm.init();
        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_INTAKEARM);

        // ------------------------------------------------------------
        // Configure drivetrain for Teleop Mode
        // ------------------------------------------------------------
        sysDrivetrain.setDriveMotorRunMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // ------------------------------------------------------------
        // Configure Vision
        // ------------------------------------------------------------
        // Set AI Camera Mode
        sysVision.setAICameraMode(utilRobotConstants.Vision.AI_Camera.AI_CAMERA_MODE_APRILTAG);

        // ------------------------------------------------------------
        // Variables for OpMode
        // ------------------------------------------------------------
        double inputAxial, inputLateral, inputYaw;
        boolean isManualSlideMode = false, isManualIntakeMode = false;

        // ------------------------------------------------------------
        // Send telemetry message to signify robot completed initialization and waiting to start;
        // ------------------------------------------------------------
        telemetry.addData(">", "------------------------------------");
        telemetry.addData(">", "All Systems Ready - Waiting to Start");
        telemetry.addData(">", "------------------------------------");
        telemetry.update();

        // Reset runtime clock
        runtime.reset();

        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_COMPLETE);

        // Repeat while in initialize state (disable if using waitForStart)
//        while(opModeInInit() && !isStopRequested()) {
//
//        }

        // Wait for Start state (disable if using opModeInInit)
        waitForStart();

        // ------------------------------------------------------------
        // Configure Telemetry
        // ------------------------------------------------------------
        // Set telemetry mode to auto-clear
        telemetry.setAutoClear(true);
        telemetry.clearAll();

        // Starting LED mode
        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_DEFAULT_TELEOP);

        // Reset runtime clock
        runtime.reset();

        // Return if a Stop Action is requested
        if (isStopRequested()) {

            // Update the Transition Adjustment Value for the IMU
            utilRobotConstants.CommonSettings.setImuTransitionAdjustment(sysDrivetrain.getRobotHeadingRaw());

            return;
        }

        // ------------------------------------------------------------
        // Command Loop: run until the end of the match (driver presses STOP)
        // ------------------------------------------------------------
        while (opModeIsActive() && !isStopRequested()) {

            // ------------------------------------------------------------
            // Controls
            // ------------------------------------------------------------
            // Gamepad1 = Main Driver
            // ------------------------------------------------------------
            // -- Robot Movement
            // -- -- Axis (left_stick_x, left_stick_y): Drive
            // -- -- Axis (right_stick_x): Rotate
            // -- -- X: Set Output Speed to High
            // -- -- Y: Set Output Speed to Med
            // -- -- A: Set Output Speed to Low
            // -- -- B: Set Output Speed to Snail
            //
            // -- Drive Mode Settings
            // -- -- D-Pad Left: Field Centric Mode
            // -- -- D-Pad Left: Robot Centric Mode
            //
            // -- Intake
            // -- -- Left Bumper (hold): Intake Normal - into robot
            // -- -- Right Bumper (hold): Intake Reverse - eject from robot
            //
            // -- Override Settings
            // -- -- D-Pad Up + X: Reset Heading Override (and Raw)
            //
            // ------------------------------------------------------------
            // Gamepad2 = Co-Driver
            // ------------------------------------------------------------
            // NA

            // ------------------------------------------------------------
            // Drivetrain
            // ------------------------------------------------------------
            // Assign gamepad control to motion in relation to:
            // -- gamepad input, direction
            // -- robot orientation to field
            // -- installed direction of control hub
            // -- orientation of drivetrain/motors
            inputYaw =  (gamepad1.right_stick_x);
            inputAxial = -(gamepad1.left_stick_y);
            inputLateral = (gamepad1.left_stick_x);

            // Set Field Centric as only Drivetrain Mode
            sysDrivetrain.driveMecanumFieldCentric(inputAxial, inputLateral, inputYaw, sysDrivetrain.getValueDrivetrainOutputPower());

//            // Drivetrain Type determined by 'Drivetrain Mode' enumeration selection (Default to Field Centric)
//            if(sysDrivetrain.getLabelDrivetrainMode().equals(utilRobotConstants.Drivetrain.LIST_MODE_TYPE_DRIVETRAIN_ROBOTCENTRIC)) {
//                // Send gamepad input for drivetrain to driveMecanum method in the drivetrain system class
//                sysDrivetrain.driveMecanum(inputAxial, inputLateral, inputYaw, sysDrivetrain.getValueDrivetrainOutputPower());
//            }
//            else {
//                // Send gamepad input for drivetrain to driveMecanumFieldCentric method in the drivetrain system class
//                sysDrivetrain.driveMecanumFieldCentric(inputAxial, inputLateral, inputYaw, sysDrivetrain.getValueDrivetrainOutputPower());
//            }

//            if(gamepad1.left_bumper) {
//                sysIntakeArm.activateIntake(1);
//            }
//
//            if(!gamepad1.left_bumper) {
//                sysIntakeArm.deactivateIntake();
//            }

            // Button Action - Set Output Power Mode to Low
            if(gamepad1.right_bumper) {
                sysDrivetrain.stateDriveMotorMaxOutputPower = enumStateDriveMotorMaxOutputPower.Low;
            }

            // Button Action - Set Output Power Mode to Medium
            if(!gamepad1.right_bumper) {
                sysDrivetrain.stateDriveMotorMaxOutputPower = enumStateDriveMotorMaxOutputPower.Medium;
            }

            // Button Action - Set Output Power Mode to Low
//            if(gamepad1.a) {
//                sysDrivetrain.stateDriveMotorMaxOutputPower = enumStateDriveMotorMaxOutputPower.Low;
//            }
//
//            // Button Action - Set Output Power Mode to Snail Mode
//            if(gamepad1.b) {
//                sysDrivetrain.stateDriveMotorMaxOutputPower = enumStateDriveMotorMaxOutputPower.Snail;
//            }

            // Button Action - Set drive mode to 'Field Centric'
//            if(gamepad1.dpad_left) {
//                sysDrivetrain.stateDrivetrainMode = enumStateDrivetrainMode.Field_Centric;
//
//                // Reset the Robot Heading (normally done on init of Drivetrain system)
//                sysDrivetrain.resetZeroRobotHeading();
//            }

            // Button Action - Set drive mode to 'Robot Centric'
//            if(gamepad1.dpad_right) {
//                sysDrivetrain.stateDrivetrainMode = enumStateDrivetrainMode.Robot_Centric;
//            }

            // ------------------------------------------------------------
            // Intake / Arm
            // ------------------------------------------------------------
            // Slow arm down when below hang setpoint (not manual) to help resist overdrive
            if(!isManualSlideMode) {
                if (sysIntakeArm.getArmCurrentPosition(utilRobotConstants.Configuration.LABEL_ARM_MOTOR_LEFT_SIDE) <= utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_HANG) {
                    sysIntakeArm.setArmMotorPower(utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MIN);
                } else {
                    sysIntakeArm.setArmMotorPower(utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MAX);
                }
            }

            // Cradle - Home/Ground
            if(gamepad1.a) {
                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE, utilRobotConstants.IntakeArm.SERVO_SLOTONE_SETPOINT_OPEN);
                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO, utilRobotConstants.IntakeArm.SERVO_SLOTTWO_SETPOINT_OPEN);
                sysIntakeArm.moveIntakeToTarget(utilRobotConstants.IntakeArm.INTAKE_ENCODER_SETPOINT_HOME, utilRobotConstants.IntakeArm.INTAKE_MOTOR_OUTPUT_POWER_MIN);
            }

            // Cradle - Travel Mode
            if(gamepad1.left_bumper) {
                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE, utilRobotConstants.IntakeArm.SERVO_SLOTONE_SETPOINT_CLOSE);
                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO, utilRobotConstants.IntakeArm.SERVO_SLOTTWO_SETPOINT_CLOSE);
                sysIntakeArm.moveIntakeToTarget(utilRobotConstants.IntakeArm.INTAKE_ENCODER_SETPOINT_TRAVEL, utilRobotConstants.IntakeArm.INTAKE_MOTOR_OUTPUT_POWER_MAX);
            }

            // Cradle - Board
            if(gamepad2.left_bumper) {
                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE, utilRobotConstants.IntakeArm.SERVO_SLOTONE_SETPOINT_CLOSE);
                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO, utilRobotConstants.IntakeArm.SERVO_SLOTTWO_SETPOINT_CLOSE);
                sysIntakeArm.moveIntakeToTarget(utilRobotConstants.IntakeArm.INTAKE_ENCODER_SETPOINT_BOARD, utilRobotConstants.IntakeArm.INTAKE_MOTOR_OUTPUT_POWER_MAX);
            }

            // Cradle - Close
            if(Math.abs(gamepad1.left_trigger) >= .25) {
                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE, utilRobotConstants.IntakeArm.SERVO_SLOTONE_SETPOINT_CLOSE);
                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO, utilRobotConstants.IntakeArm.SERVO_SLOTTWO_SETPOINT_CLOSE);
            }

            // Cradle - Release
            if(gamepad2.right_bumper) {
                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE, utilRobotConstants.IntakeArm.SERVO_SLOTONE_SETPOINT_OPEN);
                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO, utilRobotConstants.IntakeArm.SERVO_SLOTTWO_SETPOINT_OPEN);
            }

//            if(Math.abs(gamepad2.right_trigger) > .10) {
//                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE, utilRobotConstants.IntakeArm.SERVO_SLOTONE_SETPOINT_CLOSE);
//                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO, utilRobotConstants.IntakeArm.SERVO_SLOTTWO_SETPOINT_CLOSE);
//            }

//            if(gamepad2.dpad_up) {
//                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE, sysIntakeArm.getIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE) - 0.05);
//            }
//
//            if(gamepad2.dpad_down) {
//                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE, sysIntakeArm.getIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE) + 0.05);
//            }

            // Arm - Manual Control
            if(Math.abs(gamepad2.right_stick_y) >= .25) {
                isManualSlideMode = true;
                if(sysIntakeArm.getArmCurrentPosition(utilRobotConstants.Configuration.LABEL_ARM_MOTOR_LEFT_SIDE) >= 0 && sysIntakeArm.getArmCurrentPosition(utilRobotConstants.Configuration.LABEL_ARM_MOTOR_LEFT_SIDE) <= utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_MAX) {
                    if(sysIntakeArm.getLimitSensorTripped(utilRobotConstants.Configuration.LABEL_ARM_SENSOR_LIMIT_LOWER)) {
                        sysIntakeArm.moveArmManually(-(Math.abs(gamepad2.right_stick_y)), utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MIN);
                    }
                    else {
                        sysIntakeArm.moveArmManually(-(gamepad2.right_stick_y), utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MIN);
                    }
                }
            }
            else {
                if(isManualSlideMode) {
                    sysIntakeArm.moveArmManually(0, utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MIN);
                }
            }

            // Arm Setpoint - High
            if(gamepad2.y) {
                isManualSlideMode = false;
                isManualIntakeMode = false;
                sysIntakeArm.moveIntakeToTarget(utilRobotConstants.IntakeArm.INTAKE_ENCODER_SETPOINT_BOARD, utilRobotConstants.IntakeArm.INTAKE_MOTOR_OUTPUT_POWER_MAX);
                sysIntakeArm.moveArmToTarget(utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_MAX, utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MAX);
            }

            // Arm Setpoint - Hang
            if(gamepad2.x) {
                isManualSlideMode = false;
                isManualIntakeMode = false;
                sysIntakeArm.moveArmToTarget(utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_HANG, utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MAX);
            }

            // Arm Setpoint - Pre-Climb
            if(gamepad2.b) {
                isManualSlideMode = false;
                isManualIntakeMode = false;
                sysIntakeArm.moveArmToTarget(utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_PRECLIMB, utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MAX);
            }

            // Arm Setpoint - Home
            if(gamepad2.a) {
                isManualSlideMode = false;
                isManualIntakeMode = false;
                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE, utilRobotConstants.IntakeArm.SERVO_SLOTONE_SETPOINT_OPEN);
                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO, utilRobotConstants.IntakeArm.SERVO_SLOTTWO_SETPOINT_OPEN);
                sysIntakeArm.moveIntakeToTarget(utilRobotConstants.IntakeArm.INTAKE_ENCODER_SETPOINT_HOME, utilRobotConstants.IntakeArm.INTAKE_MOTOR_OUTPUT_POWER_MIN);
                sysIntakeArm.moveArmToTarget(utilRobotConstants.IntakeArm.ARM_ENCODER_SETPOINT_HOME, utilRobotConstants.IntakeArm.ARM_MOTOR_OUTPUT_POWER_MIN);
            }

            // Intake Pivot - Manual Control
            if(Math.abs(gamepad2.left_stick_y) >= .25) {
                isManualIntakeMode = true;
                if(sysIntakeArm.getIntakeCurrentPosition(utilRobotConstants.Configuration.LABEL_INTAKE_MOTOR_PIVOT) >= utilRobotConstants.IntakeArm.INTAKE_ENCODER_SETPOINT_HOME &&
                        sysIntakeArm.getIntakeCurrentPosition(utilRobotConstants.Configuration.LABEL_INTAKE_MOTOR_PIVOT) <= utilRobotConstants.IntakeArm.INTAKE_ENCODER_SETPOINT_MAX){
                    sysIntakeArm.moveIntakeManually(-(gamepad2.left_stick_y), utilRobotConstants.IntakeArm.INTAKE_MOTOR_OUTPUT_POWER_MIN);
                }
            }
            else {
                if(isManualIntakeMode) {
                    sysIntakeArm.moveIntakeManually(0, utilRobotConstants.IntakeArm.INTAKE_MOTOR_OUTPUT_POWER_MIN);
                }
            }

            // Manual Control of pixel cradle - to board
//            if(gamepad2.dpad_up) {
//
//                if(sysIntakeArm.getIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT) <= utilRobotConstants.IntakeArm.SERVO_PIVOT_SETPOINT_BOARD) {
//                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT, sysIntakeArm.getIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT) + 0.05);
//                }
//
//            }
//
//            // Manual Control of pixel cradle - to home
//            if(gamepad2.dpad_down) {
//
//                if(sysIntakeArm.getIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT) >= utilRobotConstants.IntakeArm.SERVO_PIVOT_SETPOINT_HOME) {
//                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT, sysIntakeArm.getIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT) - 0.05);
//                }
//
//            }

            // ------------------------------------------------------------
            // Vision
            // ------------------------------------------------------------

            // ------------------------------------------------------------
            // Sensor
            // ------------------------------------------------------------


            // ------------------------------------------------------------
            // Override
            // ------------------------------------------------------------
            // Button Action - Reset Heading Override (and Raw)
            if(gamepad1.start && gamepad1.y) {

                // Reset the Robot Heading (normally done on init of Drivetrain system)
                sysDrivetrain.resetZeroRobotHeading();

                // Cycle Pause
//                sleep(utilRobotConstants.CommonSettings.SLEEP_TIMER_MILLISECONDS_DEFAULT);
            }

            // ------------------------------------------------------------
            // Endgame
            // ------------------------------------------------------------
            if(runtime.time() >= 90.00 && runtime.time() <= 120.00) {
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_ALERT_ENDGAME);
            }

            if (gamepad1.back && gamepad2.back) {
                sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_DRONE_PIVOT_SERVO_MAIN, utilRobotConstants.IntakeArm.SERVO_DRONE_PIVOT_SETPOINT_LAUNCH);
            }

            if (gamepad1.start && gamepad2.start) {

                // Only allow the launch if the pivot servo is open
                if(sysIntakeArm.getIntakeServoPosition(utilRobotConstants.Configuration.LABEL_DRONE_PIVOT_SERVO_MAIN) == utilRobotConstants.IntakeArm.SERVO_DRONE_PIVOT_SETPOINT_LAUNCH) {
                    sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_DRONE_LAUNCH_SERVO_MAIN, utilRobotConstants.IntakeArm.SERVO_DRONE_LAUNCH_SETPOINT_OPEN);
                }
            }

            // ------------------------------------------------------------
            // Driver Hub Feedback
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Run Time", runtime.toString());

//            // ------------------------------------------------------------
//            // - Gamepad telemetry
//            // ------------------------------------------------------------
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("-", "-- Gamepad");
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("Gamepad 1 - [Y] Axial", "%4.2f", gamepad1.left_stick_y);
//            telemetry.addData("Gamepad 1 - [X] Lateral", "%4.2f", gamepad1.left_stick_x);
//            telemetry.addData("Gamepad 1 - [R] Rotation", "%4.2f", gamepad1.right_stick_x);
//
//            // ------------------------------------------------------------
//            // - Drivetrain telemetry
//            // ------------------------------------------------------------
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("-", "-- Drivetrain");
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("Drivetrain Mode", sysDrivetrain.getLabelDrivetrainMode());
//            telemetry.addData("Drivetrain Power", sysDrivetrain.getLabelDrivetrainOutputPower());
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("Power Front left/Right", "%4.2f, %4.2f"
//                    , sysDrivetrain.getDrivetrainMotorPower(utilRobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_FRONT)
//                    , sysDrivetrain.getDrivetrainMotorPower(utilRobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_FRONT));
//            telemetry.addData("Power Back  left/Right", "%4.2f, %4.2f"
//                    , sysDrivetrain.getDrivetrainMotorPower(utilRobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_BACK)
//                    , sysDrivetrain.getDrivetrainMotorPower(utilRobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_BACK));
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("Encoder Front left/Right", "%7d, %7d"
//                    , sysDrivetrain.getDrivetrainMotorEncoderPosition(utilRobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_FRONT)
//                    , sysDrivetrain.getDrivetrainMotorEncoderPosition(utilRobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_FRONT));
//            telemetry.addData("Encoder Back  left/Right", "%7d, %7d"
//                    , sysDrivetrain.getDrivetrainMotorEncoderPosition(utilRobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_LEFT_BACK)
//                    , sysDrivetrain.getDrivetrainMotorEncoderPosition(utilRobotConstants.Configuration.LABEL_DRIVETRAIN_MOTOR_RIGHT_BACK));
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("Robot Heading Raw", sysDrivetrain.getRobotHeadingRaw());
//            telemetry.addData("Heading Adjustment", utilRobotConstants.CommonSettings.getImuTransitionAdjustment());
//            telemetry.addData("Robot Heading (Adjusted)", sysDrivetrain.getRobotHeadingAdj());
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("Robot Angle - Yaw (Z)", sysDrivetrain.getRobotAngles().getYaw(AngleUnit.DEGREES));
//            telemetry.addData("Robot Angle - Pitch (X)", sysDrivetrain.getRobotAngles().getPitch(AngleUnit.DEGREES));
//            telemetry.addData("Robot Angle - Yaw (Z)", sysDrivetrain.getRobotAngles().getRoll(AngleUnit.DEGREES));
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("Robot Angle Velocity - Yaw (Z)", sysDrivetrain.getRobotAngularVelocity().zRotationRate);
//            telemetry.addData("Robot Angle Velocity - Pitch (X)", sysDrivetrain.getRobotAngularVelocity().xRotationRate);
//            telemetry.addData("Robot Angle Velocity - Yaw (Z)", sysDrivetrain.getRobotAngularVelocity().yRotationRate);
//
//            // ------------------------------------------------------------
//            // - Vision telemetry
//            // ------------------------------------------------------------
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("-", "-- Vision");
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("Camera Block Count", sysVision.getCameraObjectList().length);
//            telemetry.addData("Alliance Color", sysVision.getDetectedAllianceTagColor());
////            telemetry.addData("R-G-B", "%4, %4, %4"
////                    , sysVision.getAllianceTagColorLevel("red")
////                    , sysVision.getAllianceTagColorLevel("green")
////                    , sysVision.getAllianceTagColorLevel("blue"));
//
//            // ------------------------------------------------------------
//            // - Intake / Arm telemetry
//            // ------------------------------------------------------------
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("-", "-- Intake / Arm");
//            telemetry.addData("-", "------------------------------");
//            telemetry.addData("Pivot Position", sysIntakeArm.getArmCurrentPosition(utilRobotConstants.Configuration.LABEL_INTAKE_MOTOR_PIVOT));
//            telemetry.addData("Slot One Position", sysIntakeArm.getIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE));
//            telemetry.addData("Slot Two Position", sysIntakeArm.getIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO));
//            telemetry.addData("Arm Position - Left", sysIntakeArm.getArmCurrentPosition(utilRobotConstants.Configuration.LABEL_ARM_MOTOR_LEFT_SIDE));
//            telemetry.addData("Arm Position - Right", sysIntakeArm.getArmCurrentPosition(utilRobotConstants.Configuration.LABEL_ARM_MOTOR_RIGHT_SIDE));
//
//            // ------------------------------------------------------------
//            // - Lighting telemetry
//            // ------------------------------------------------------------
//            if(utilRobotConstants.Configuration.ENABLE_LIGHTING) {
//                telemetry.addData("-", "------------------------------");
//                telemetry.addData("-", "-- Lighting");
//                telemetry.addData("-", "------------------------------");
//                telemetry.addData("Pattern", sysLighting.ledLightPattern.toString());
//            }

            // ------------------------------------------------------------
            // - send telemetry to driver hub
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "(reset robot)");
            telemetry.addData("-", "-- If you see an I2C error");
            telemetry.addData("-", "");
            telemetry.addData("-", "(change battery)");
            telemetry.addData("-", "-- if expansion hub loses connection");
            telemetry.addData("-", "-- if voltage below 12v");
            telemetry.addData("-", "");
            telemetry.addData("-", "Take your time!");
            telemetry.addData("-", "");
            telemetry.addData("-", "Drive with care and caution");
            telemetry.addData("-", "");
            telemetry.addData("-", "Don't drive like you stole it");
            telemetry.addData("-", "");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- Stuff you should know!");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Pivot Position", sysIntakeArm.getIntakeCurrentPosition(utilRobotConstants.Configuration.LABEL_INTAKE_MOTOR_PIVOT));
            telemetry.addData("Slot One Position", sysIntakeArm.getIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE));
            telemetry.addData("Slot Two Position", sysIntakeArm.getIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO));
            telemetry.addData("-", "------------------------------");

            telemetry.update();

            // Input assignment to 'pause' telemetry update(s)
//            if (!gamepad1.dpad_right) {
                telemetry.update();
//            }

            // Pace this loop so commands move at a reasonable speed.
//            sleep(utilRobotConstants.CommonSettings.SLEEP_TIMER_MILLISECONDS_DEFAULT);
        }

        // ------------------------------------------------------------
        // Closing Teleop
        // ------------------------------------------------------------
        // Update the Transition Adjustment Value for the IMU
        utilRobotConstants.CommonSettings.setImuTransitionAdjustment(sysDrivetrain.getRobotHeadingRaw());

    }
}
