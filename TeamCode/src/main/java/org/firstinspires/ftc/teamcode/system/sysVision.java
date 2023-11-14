package org.firstinspires.ftc.teamcode.system;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.utility.utilRobotConstants;

/**
 * <h2>System - Vision - Vision and Sensors</h2>
 * <hr>
 * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
 * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
 * <hr>
 * <p>
 * System that contains attributes and methods for:
 * vision control systems including cameras and other sensors.
 * </p>
 * <br>
 */
public class sysVision {

    private LinearOpMode sysOpMode = null;

    private HuskyLens frontAICamera;

    private ColorSensor allianceTagSensor;

    public sysVision(LinearOpMode inOpMode) { sysOpMode = inOpMode; }

    public void init() {

        // Camera(s)
        frontAICamera = sysOpMode.hardwareMap.get(HuskyLens.class, utilRobotConstants.Configuration.LABEL_FRONT_AI_CAMERA);

        // Sensor(s)
        allianceTagSensor = sysOpMode.hardwareMap.get(ColorSensor.class, utilRobotConstants.Configuration.LABEL_ALLIANCE_COLOR_SENSOR);

        // Configure Sensor
        allianceTagSensor.enableLed(utilRobotConstants.Vision.Sensor.ALLIANCE_SENSOR_LED_ACTIVE);

        // Display telemetry
        sysOpMode.telemetry.addData(">", "------------------------------------");

        // Check for camera communication
        if (!frontAICamera.knock()) {
            sysOpMode.telemetry.addData(">", " ERROR: Cannot communicate with " + frontAICamera.getDeviceName());
        }
        else {
            sysOpMode.telemetry.addData(">", " AI Camera Initialized");
        }

        // Initialize in April Tag Mode
        frontAICamera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        sysOpMode.telemetry.addData("Alliance Color", getDetectedAllianceTagColor());

        sysOpMode.telemetry.addData(">", " System: Vision Initialized");
        sysOpMode.telemetry.update();

    }

    public HuskyLens.Block[] getCameraObjectList() {

        HuskyLens.Block[] arrayBlockList = frontAICamera.blocks();

        return arrayBlockList;
    }

    public HuskyLens.Block getCameraObject(HuskyLens.Block[] inBlockList, int inBlockID) {
        HuskyLens.Block outBlock = null;

        for (HuskyLens.Block itemBlock: inBlockList) {
            if(itemBlock.id == inBlockID) {
              outBlock = itemBlock;
            }
        }

        return outBlock;
    }

    public String getDetectedAllianceTagColor() {
        String outDetectedColor;

        if(allianceTagSensor.blue() > allianceTagSensor.red()) {
            outDetectedColor = "blue";
        }
        else {
            outDetectedColor = "red";
        }

        return outDetectedColor;
    }

    public int getAllianceTagColorLevel(String inCheckColor) {
        int outColorLevel;

        switch(inCheckColor) {
            case("blue"):
                outColorLevel = allianceTagSensor.blue();
                break;
            case("red"):
                outColorLevel = allianceTagSensor.red();
                break;
            case("green"):
                outColorLevel = allianceTagSensor.green();
                break;
            default:
                outColorLevel = 0;
        }

        return outColorLevel;
    }

    public void setAICameraMode(String inCameraMode) {

        switch(inCameraMode) {
            case(utilRobotConstants.Vision.AI_Camera.AI_CAMERA_MODE_OBJECT_RECOGNITION):
                frontAICamera.selectAlgorithm(HuskyLens.Algorithm.OBJECT_RECOGNITION);
                break;
            case(utilRobotConstants.Vision.AI_Camera.AI_CAMERA_MODE_OBJECT_TRACKING):
                frontAICamera.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
                break;
            default:
                frontAICamera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
                break;
        }

    }


}
