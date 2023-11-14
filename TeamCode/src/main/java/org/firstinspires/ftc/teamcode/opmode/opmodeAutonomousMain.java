package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.system.sysDrivetrainMecanum;
import org.firstinspires.ftc.teamcode.system.sysIntakeArm;
import org.firstinspires.ftc.teamcode.system.sysLighting;
import org.firstinspires.ftc.teamcode.system.sysVision;
import org.firstinspires.ftc.teamcode.utility.utilRobotConstants;

/**
 * <h2>OpMode - Autonomous - Main</h2>
 * <hr>
 * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
 * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
 * <hr>
 * <p>
 * Autonomous Mode - Main
 * One OpMode to Rule them All
 * </p>
 * <br>
 */
@Autonomous(name = "Autonomous - Main", group = "smart", preselectTeleOp = "Teleop Main")
//@Disabled
public class opmodeAutonomousMain extends LinearOpMode {
    // ------------------------------------------------------------
    // System(s) - Define system and create instance of each system
    // ------------------------------------------------------------
    // -- Robot Initializtion

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
    public void runOpMode() {
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
        // Configure drivetrain for Autonomous Mode
        // -- Set to run without encoders for timed drive mode
        // ------------------------------------------------------------
        sysDrivetrain.setDriveMotorRunMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // ------------------------------------------------------------
        // Variables for OpMode
        // ------------------------------------------------------------
        boolean isImageFound = false;
        int targetZone = 0;

        HuskyLens.Block[] targetObjectList = null;
        HuskyLens.Block targetObject = null;

        // ------------------------------------------------------------
        // Send telemetry message to signify robot completed initialization and waiting to start;
        // ------------------------------------------------------------
        telemetry.addData(">", "------------------------------------");
        telemetry.addData(">", "All Systems Ready - Waiting to Start");
        telemetry.update();

        // Reset runtime clock
        runtime.reset();

        // System Initialization Complete
        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_COMPLETE);

        // Repeat while in initialize state (disable if using waitForStart)

        // Set AI Camera Mode
        sysVision.setAICameraMode(utilRobotConstants.Vision.AI_Camera.AI_CAMERA_MODE_OBJECT_RECOGNITION);

        // ------------------------------------------------------------
        // Configure Telemetry
        // ------------------------------------------------------------
        // Set telemetry mode to auto-clear
        telemetry.setAutoClear(true);
        telemetry.clearAll();

        while(opModeInInit() && !isStopRequested()) {

            // Look for Team Element position
            targetObjectList = sysVision.getCameraObjectList();

            if(targetObjectList.length > 0) {

                targetObject = sysVision.getCameraObject(targetObjectList, 1);

                if(targetObject != null) {

                    if (targetObject.x < utilRobotConstants.Vision.RANDOM_TARGET_ZONE_ONE_X) {
                        targetZone = 1; // left
                        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_PARK_ONE);
                    } else if (targetObject.x > utilRobotConstants.Vision.RANDOM_TARGET_ZONE_ONE_X && targetObject.x < utilRobotConstants.Vision.RANDOM_TARGET_ZONE_TWO_X) {
                        targetZone = 2; // center
                        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_PARK_TWO);
                    } else if (targetObject.x > utilRobotConstants.Vision.RANDOM_TARGET_ZONE_TWO_X) {
                        targetZone = 3; // right
                        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_PARK_THREE);
                    } else {
                        targetZone = 0; // no zone detected
                        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_PARK_INVALID);
                    }

                }
            }

            // ------------------------------------------------------------
            // Driver Hub Feedback
            // ------------------------------------------------------------
            telemetry.addData("Init Time", runtime.toString());
            // ------------------------------------------------------------
            // - Vision telemetry
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- Vision");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Alliance Color", sysVision.getDetectedAllianceTagColor());
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Camera Block Count", sysVision.getCameraObjectList().length);
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Randomized Zone", targetZone);
            if (targetObject != null) {
                telemetry.addData("-", "------------------------------");
                telemetry.addData("-", "-- Target Object");
                telemetry.addData("-", "------------------------------");
                telemetry.addData("Target x:", targetObject.x);
                telemetry.addData("Target y:", targetObject.y);
                telemetry.addData("Target width:", targetObject.width);
                telemetry.addData("Target height:", targetObject.height);
                telemetry.addData("Target top:", targetObject.top);
                telemetry.addData("Target left:", targetObject.left);
            }

            // ------------------------------------------------------------
            // - send telemetry to driver hub
            // ------------------------------------------------------------
            telemetry.update();
        }

        // Wait for Start state (disable if using opModeInInit)
        waitForStart();

        // ------------------------------------------------------------
        // Configure Telemetry
        // ------------------------------------------------------------
        // Set telemetry mode to append
        telemetry.setAutoClear(false);
        telemetry.clearAll();

        // Starting LED mode
        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_DEFAULT_AUTONOMOUS);

        // Reset runtime clock
        runtime.reset();

        // Return if a Stop Action is requested
        if (isStopRequested()) {

            // Update the Transition Adjustment Value for the IMU
            utilRobotConstants.CommonSettings.setImuTransitionAdjustment(sysDrivetrain.getRobotHeadingRaw());
            return;
        }

        // Autonomous Routine
        if (opModeIsActive()) {

            // If the targetObject was not detected attempt to detect before movement(s)
            // -- for 5 seconds
            if(targetZone == 0) {

                while (opModeIsActive() && !isStopRequested() && runtime.time() < 5 && targetZone == 0) {

                    // Look for Team Element position
                    targetObjectList = sysVision.getCameraObjectList();

                    if (targetObjectList.length > 0) {

                        targetObject = sysVision.getCameraObject(targetObjectList, 1);

                        if (targetObject != null) {

                            if (targetObject.x < utilRobotConstants.Vision.RANDOM_TARGET_ZONE_ONE_X) {
                                targetZone = 1; // left
                                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_PARK_ONE);
                            } else if (targetObject.x > utilRobotConstants.Vision.RANDOM_TARGET_ZONE_ONE_X && targetObject.x < utilRobotConstants.Vision.RANDOM_TARGET_ZONE_TWO_X) {
                                targetZone = 2; // center
                                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_PARK_TWO);
                            } else if (targetObject.x > utilRobotConstants.Vision.RANDOM_TARGET_ZONE_TWO_X) {
                                targetZone = 3; // right
                                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_PARK_THREE);
                            } else {
                                targetZone = 0; // no zone detected
                                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_PARK_INVALID);
                            }

                        }
                    }
                }
            }


            // ------------------------------------------------------------
            // Driver Hub Feedback
            // ------------------------------------------------------------
            telemetry.addData("Init Time", runtime.toString());
            // ------------------------------------------------------------
            // - Vision telemetry
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- Vision");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Alliance Color", sysVision.getDetectedAllianceTagColor());
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Camera Block Count", sysVision.getCameraObjectList().length);
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Randomized Zone", targetZone);
            if (targetObject != null) {
                telemetry.addData("-", "------------------------------");
                telemetry.addData("-", "-- Target Object");
                telemetry.addData("-", "------------------------------");
                telemetry.addData("Target x:", targetObject.x);
                telemetry.addData("Target y:", targetObject.y);
                telemetry.addData("Target width:", targetObject.width);
                telemetry.addData("Target height:", targetObject.height);
                telemetry.addData("Target top:", targetObject.top);
                telemetry.addData("Target left:", targetObject.left);
            }

            // Set Camera Mode
            sysVision.setAICameraMode(utilRobotConstants.Vision.AI_Camera.AI_CAMERA_MODE_OBJECT_RECOGNITION);



            // Set Camera Mode
            sysVision.setAICameraMode(utilRobotConstants.Vision.AI_Camera.AI_CAMERA_MODE_APRILTAG);



//            sysDrivetrain.driveEncoderOdometryAxial(-10, 1);


        }


    }


}
