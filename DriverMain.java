package org.firstinspires.ftc.teamcode;

// OpMode imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

// Java tool imports
import java.lang.reflect.Array;
import java.util.concurrent.Delayed;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

// Java time tool imports
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

// Sensor and motor imports
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

// Image recognition imports
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

// Unit conversion imports
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.concurrent.TimeUnit;
// }

@TeleOp(name="DriverMain", group="Custom Opmodes")

public class DriverMain extends LinearOpMode {

    // Declare OpMode members.
    private int reenactFrame = 0;
    private ElapsedTime runtime = new ElapsedTime();
    private double intake = 0, hookOffset = 0;
    private DcMotor frMotor, flMotor, brMotor, blMotor, ilMotor, irMotor;
    private Servo udServo, lrServo;
    private DistanceSensor proximFront, proximFront2;
    private boolean padOneYToggle, padOneXToggle, padOneAToggle, padOneBToggle, hookCurrentPos, positionMaintain, recordingToggle, reenactToggle;
    private Servo lHook, rHook;
    private double lx, ly, rx, ry, theta, d, offset, lBumper, modTrigLeft, m;
    private List<double[]> recordingData = new ArrayList<double[]>();
    
    private boolean test;
    
    // Declare Image Detection members
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    /*
     * NOTE: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "Ab0nmoX/////AAAAGbeJ1P4G1E64jrrtI450zd1U87lxHOj9akkdSsxM2egUUijNJboj+0CWTpHtFQkn8cpXWxw3sIdEZYdKvqNSjRi1BeqSZNdhK7oDiGEMoeyNReLItGa0qLSoRwbfhUzWXpp2k/naIkca8YGkv3AMy7i2znvv71hv6qB0qOAn1Kq/axatBXvTozw4pVPvM7cZQ4iH3kjH8Rt/3jnnWJhqYA24w0iE3976fA1mNQIuL0J76rlZXiTNayZwFTu0/jLUfoLcxSKNXeGcnSJHkrxb0b/NH+cB3elIC+cIagGEL/xvDtLpgxOgDLH+TFhnvSuCkoQSKKlsd7r1+MVqIugJJbLfuyQtx+a8GcEmHzD+iK0m";
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;
    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Motor setup and declaration
        brMotor  = hardwareMap.get(DcMotor.class, "brMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        frMotor  = hardwareMap.get(DcMotor.class, "frMotor");
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        proximFront = hardwareMap.get(DistanceSensor.class, "pfSensor");
        //proximFront2 = hardwareMap.get(DistanceSensor.class, "pf2Sensor");
        ilMotor  = hardwareMap.get(DcMotor.class, "ilMotor");
        irMotor = hardwareMap.get(DcMotor.class, "irMotor");
        
        lHook  = hardwareMap.get(Servo.class, "lHook");
        rHook = hardwareMap.get(Servo.class, "rHook");
        
        // Just a test value, could be used for anything
        test = true;
        
        // Toggle values
        positionMaintain = false;
        lBumper = 0;

        // Recording and reenact
        recordingToggle = false;
        reenactToggle = false;

        // Button setup and declaration
        hookCurrentPos = false;

        waitForStart();
        // Main runtime loop
        while (opModeIsActive()) {
            // Test Progress (Tested: Will need to be more precise, or have longer process time)
            //vuforiaObjectDetection();
            
            // Basic Auton 
            if(test) {  
                //moveForward(6, 0.15);
                //moveSideways(6, 0.15);
                //moveForward(-6, 0.15);
                //moveSideways(-6, 0.15);
                test = false;
            }
            
            // Test Progress (Tested: Successful)
            toggleRecording();
            
            // Test Progress (Tested: Successful)
            toggleReenacting();

            // Test Progress (Tested: Successful: Will add more features)
            if(recordingToggle == true && reenactToggle == false) {
                recordRobotData();
            }

            // Test Progress (Tested: Successful: Will add more features)
            if(reenactToggle == true && recordingToggle == false) {
                reenactFrameUpdate();
                reenactValueUpdate(reenactFrame);
            } else {
                runtimeValueUpdate();
            }

            // Test Progress (Untested)
            intakeMotors(0.6);
            // Test Progress (Tested: Successful)
            hookServos(180);
            // Test Progress (Tested: Currently Revising)
            mecanumDrive(rx, ry, lx, m, offset);
            // Telemetry values
            logTelemetry();
            telemetry.update();
        }
    }

    /*
        NOTE TO THOSE WHO WOULD LIKE TO DEVELOP OP-MODE FOR THE ROBOT:
        - The following methods below are used to program and manage aspects of
          of the robot. Each method has a description of what it does, along with
          a note of the input-output structure, ie: What values go in and what values
          come out.
        - You may add onto the code so long as what you add on is in functional format.
          This is so future programmers and those who want to add onto the code aren't hindered
          by your conventions. By using functional format, ie: you create a method to run your
          code along with a description of what it does and what the inputs are, programmers will be
          able to further add to the code and also understand what you wrote.

        THE FOLLOWING CODE BELOW ARE FUNCTIONS USED WITHIN THE OPMODE, ALONG WITH DEVELOPER FUNCTIONS
    */

    // Function that manages the key for toggling reenaction
    // Input-Output Structure: (none) => (void)
    public void toggleReenacting() {
        // Currently mapped to gamepad 1 b-key for toggling
        if(!gamepad1.b) {
            padOneBToggle = false;
        } else if (gamepad1.b && padOneBToggle == false){
            padOneBToggle = true;
            if(reenactToggle == false) {
                reenactFrame = 0;
                reenactToggle = true;
            } else {
                reenactToggle = false;
            }
        }
    }

    // Function that manages the key for toggling recording
    // Input-Output Structure: (none) => (void)
    public void toggleRecording() {
        // Currently mapped to gamepad 1 a-key for toggling
        if(!gamepad1.a) {
            padOneAToggle = false;
        } else if (gamepad1.a && padOneAToggle == false){
            padOneAToggle = true;
            if(recordingToggle == false && reenactToggle == false) {
            recordingData.clear();
                recordingToggle = true;
            } else {
                recordingToggle = false;
            }
        }
    }

    // Function that when active, will start recording the robot's current state
    // Input-Output Structure: (none) => (void)
    public void recordRobotData() {
        // Places all values into a dataset for management
        double[] dataSet = {rx, ry, lx, lx, offset, lBumper, gamepad1.left_trigger, intake, hookOffset};

        // Adds data to the dataset
        recordingData.add(dataSet);

        // Adds 100ms delay to prevent data overwrite
        // Might wanna reduce delay in future
        try {
            Thread.sleep(100);
        } catch(InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }

    // Function that updates/manages gamepad values during runtime
    // Input-Output Structure: (none) => (void)
    public void runtimeValueUpdate() {
        // This chunk manages the maintain orientation function of the code where the left bumper toggles whether the robot will maintain initial orientation
        // This is WIP and will most likely be discarded in the future
        if(gamepad1.left_bumper == true && lBumper == 0) {
            lBumper = 1;
            if(positionMaintain == true) {
                positionMaintain = false;
            } else {
                positionMaintain = true;
            }
        }
        if(gamepad1.left_bumper == false) {
            lBumper = 0;
        }
        if(positionMaintain == true) {
            offset += 0.09 * lx / (2 * Math.PI);
        }

        // This updates all gamepad values
        rx = gamepad1.left_stick_x;
        ry = -gamepad1.left_stick_y;
        lx = gamepad1.right_stick_x;
        ly = -gamepad1.right_stick_y;
        m = 0.45 + 1 / (1 + Math.exp(-10 * (gamepad1.left_trigger - 0.5)));
        hookOffset = hookOffset;
    }

    // Function that updates the gamepad values with respect to reenaction frame
    // Input-Output Structure: (frame) => (void)
    public void reenactValueUpdate(int frame) {
        // Set all gamepad values to that of a certain frame in time
        rx = recordingData.get(frame)[0];
        ry = recordingData.get(frame)[1];
        lx = recordingData.get(frame)[2];
        ly = recordingData.get(frame)[3];
        offset = recordingData.get(frame)[4];
        lBumper = recordingData.get(frame)[5];
        m = 0.45 + 1 / (1 + Math.exp(-10 * (recordingData.get(frame)[6] - 0.5)));
        hookOffset = recordingData.get(frame)[8];

        // Following below will be added later:

        //ilMotor.setPower(-recordingData.get(reenactFrame)[7]);
        //irMotor.setPower(recordingData.get(reenactFrame)[7]);
        //rHook.setPosition(180 -recordingData.get(reenactFrame)[8]);
        //lHook.setPosition(recordingData.get(reenactFrame)[8]);
    }

    // Function that updates reenaction frames for the robot
    // Input-Output Structure: (none) => (void)
    public void reenactFrameUpdate() {
        // Checks if recording even exists accessing it for reenaction
        if(recordingData != null) {
            // Makes sure that non-existant indices are not accessed during runtime
            if(reenactFrame + 1 != recordingData.size()) {
                // Increases reenacting frame
                reenactFrame += 1;
            } else {
                reenactToggle = false;
            }

            // Adds 100ms delay to prevent data overwrite
            // Might wanna reduce delay in future
            try {
                Thread.sleep(100);
            } catch(InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
        } else {
            // Error handeling
            System.out.println("Error: Data for reenactment does not exist!");
        }
    }

    // Function that takes a point and rotates it an angle theta in the 2d plane, returning the newly rotated points
    // Input-Output Structure: (x1, y1, angle-offset) => ({new x-value, new y-value})
    public double[] planarRotation(double x, double y, double theta) {
        // x and y are the values of the coordinate you want to rotate
        // theta is the degree you would like the points to be rotated in their plane

        // Applies rotation matrix to x and y points and calculates their new rotated forms
        double nx = x*Math.cos(theta) - y*Math.sin(theta);
        double ny = y*Math.cos(theta) + x*Math.sin(theta);

        // Stores points in an array and prepares them for export
        double[] pointArray = {nx, ny};
        return pointArray;
    }

    // Function that controls intake motors for the robot based off of value
    // Input-Output Structure: (rotation) => (void)
    public void intakeMotors(double rotation) {
        // Setting intake left and right motors both during intake and reenactment
        if(!gamepad1.y) {
            padOneYToggle = false;
        } else if (gamepad1.y && padOneYToggle == false){
            padOneYToggle = true;
            // Only change intake if you are in op-mode and not reenacting
            if(reenactToggle == false) {
                intake = Math.abs(intake - rotation);
            }
        }
        ilMotor.setPower(intake);
        irMotor.setPower(-intake);
    }

    // Function that controls the hook servos for the robot
    // Input-Output Structure: (offset in degrees) => (void)
    public void hookServos(float position) {
        // Setting hook right and left both during opMode and reenactment
        if(!gamepad1.x) {
            padOneXToggle = false;
        } else if (gamepad1.x && padOneXToggle == false){
            padOneXToggle = true;
            // Only change offset if you are in op-mode and not reenacting
            if(reenactToggle == false) {
                hookOffset = Math.abs(hookOffset - position);
            }
        }
        // Set servo positions based upon offset position
        rHook.setPosition(hookOffset/180);
        lHook.setPosition(1 - hookOffset/180);
    }

    // Function that controls drive base motors for a mecanum-drive
    // Input-Output Structure: (x1, y1, x2, m, angle-offset) => (void)
    public void mecanumDrive(double x1, double y1, double x2, double m, double theta) {
        // x1 and y1 are the x and y values from the joystick which controls the robot's drive
        // x2 is the x value from the joystick that will control the robot's left-right rotation
        // m is the speed modifier for the motors, making spin slower or faster
        // theta is the orientation you would like the robot to be in

        // Finds new orientation coordinates
        double nx = planarRotation(x1, y1, theta)[0];
        double ny = planarRotation(x1, y1, theta)[1];

        // Set power to motors accordingly
        flMotor.setPower((-(nx + ny) + 0.75 * -x2) * m);
        brMotor.setPower((-(nx + ny) + 0.75 * x2) * m);
        blMotor.setPower((-(nx - ny) + 0.75 * x2) * m);
        frMotor.setPower((-(nx - ny) + 0.75 * -x2) * m);
    }

    // Function that controls drive base motors for a omni-drive
    // Input-Output Structure: (x1, y1, x2, m, angle-offset) => (void)
    public void omniDrive(double x1, double y1, double x2, double m, double theta) {
        // x1 and y1 are the x and y values from the joystick which controls the robot's drive
        // x2 is the x value from the joystick that will control the robot's left-right rotation
        // m is the speed modifier for the motors, making spin slower or faster
        // theta is the orientation you would like the robot to be in

        // Finds new orientation coordinates
        double nx = planarRotation(x1, y1, -Math.PI / 4 + theta)[0];
        double ny = planarRotation(x1, y1, -Math.PI / 4 + theta)[1];

        // Set power to motors accordingly
        flMotor.setPower((-nx + 0.45 * -x2) * m);
        brMotor.setPower((-nx + 0.45 * -x2) * m);
        blMotor.setPower((ny + 0.45 * -x2) * m);
        frMotor.setPower((ny + 0.45 * -x2) * m);
    }

    // Function that converts arraylist into string format
    // Input-Output Structure: (arraylist<double[]>) => (string)
    public String convertListToString(List<double[]> inputList) {
      String output = "[";
      // Ensure input is verified and functional
      if(inputList != null) {
        for(int i = 0; i < inputList.size(); i++) {
          // Represents converting first dimension to string
          output += Arrays.toString(inputList.get(i));
        }
         // Closes final string and returns the conversion
         output += "]";
         return output;
      } else {
        // Error handeling
        System.out.println("Error: Following list is badly formatted or corrupted!");
        return "";
      }
    }
    
    // Function that acts as alternate method to auton coding through encoder-distance ratios and returns target deltaEncoder
    // Input-Output Structure: (distance, power applied) => (void) 
    public void moveSideways(double distance, double power) {
      // Empirically tested deltaEncoder per deltaDistance
      double deltaEncoder = 1000;
      // Empirically tested deltaDistance per deltaEncoder
      double deltaDistance = 17;
      // Uses ratios to determine the target deltaEncoderValue
      double targetDeltaEncoder = deltaEncoder / deltaDistance * distance;
     
      // Reset motors and encoders
      brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          
      // Set power to motors for achieved speed
      brMotor.setPower(-power);
      blMotor.setPower(power);
      frMotor.setPower(power);
      flMotor.setPower(-power);
     
      // Rotate Motors to target position
      brMotor.setTargetPosition(-1 * (int)Math.round(targetDeltaEncoder));
      blMotor.setTargetPosition((int)Math.round(targetDeltaEncoder));
      frMotor.setTargetPosition((int)Math.round(targetDeltaEncoder));
      flMotor.setTargetPosition(-1 * (int)Math.round(targetDeltaEncoder));
      // Prepare motors for encoder positioning
      brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     
      // Wait until robot has reached the target position
      while (opModeIsActive() && blMotor.isBusy()) {
        telemetry.addData("encoder-fwd", brMotor.getCurrentPosition() + "  busy=" + brMotor.isBusy());
        telemetry.addData("encoder-fwd", blMotor.getCurrentPosition() + "  busy=" + blMotor.isBusy());
        telemetry.addData("encoder-fwd", frMotor.getCurrentPosition() + "  busy=" + frMotor.isBusy());
        telemetry.addData("encoder-fwd", flMotor.getCurrentPosition() + "  busy=" + flMotor.isBusy());
        telemetry.update();
        idle();
      }
      
      // Reset motors for regular motion
      brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    // Function that acts as alternate method to auton coding through encoder-distance ratios and returns target deltaEncoder
    // Input-Output Structure: (distance, power applied) => (void)
    public void moveForward(double distance, double power) {
      // Empirically tested
      double deltaEncoder = 1000;
      // Empirically tested
      double deltaDistance = 19;
      // Uses ratios to determine the target deltaEncoderValue
      double targetDeltaEncoder = deltaEncoder / deltaDistance * distance;
     
      // Reset motors and encoders
      brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          
      // Set power to motors for achieved speed
      brMotor.setPower(power);
      blMotor.setPower(power);
      frMotor.setPower(power);
      flMotor.setPower(power);
     
      // Rotate Motors to target position
      brMotor.setTargetPosition(-(int)Math.round(targetDeltaEncoder));
      blMotor.setTargetPosition((int)Math.round(targetDeltaEncoder));
      frMotor.setTargetPosition((int)Math.round(targetDeltaEncoder));
      flMotor.setTargetPosition(-(int)Math.round(targetDeltaEncoder));
      
      // Prepare motors for encoder positioning
      brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     
      // Wait until robot has reached the target position
      while (opModeIsActive() && brMotor.isBusy()) {
        telemetry.addData("encoder-fwd", brMotor.getCurrentPosition() + "  busy=" + brMotor.isBusy());
        telemetry.addData("encoder-fwd", blMotor.getCurrentPosition() + "  busy=" + blMotor.isBusy());
        telemetry.addData("encoder-fwd", frMotor.getCurrentPosition() + "  busy=" + frMotor.isBusy());
        telemetry.addData("encoder-fwd", flMotor.getCurrentPosition() + "  busy=" + flMotor.isBusy());
        telemetry.update();
        idle();
      }
      
      // Reset motors for regular motion
      brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
   }

    // Function that uses Vuforia + Tensorflow to detect the existance of stones, storing the data in an array and returning it for analysis
    // Input-Output Structure: (void) => (stringData[])
    public String[] imageRecogFunction() {
        // Still in dev stage
        String[] returnValue = {""};
        return returnValue;
    }

    // Initialize the Vuforia localization engine
    // Input-Output Structure: (void) => (void)
    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    // Updates and posts current telemetry values
    // Input-Ouput Structure: (void) => (void)
    public void logTelemetry() {
        telemetry.addData("-------- Sensor Data --------", "");
        telemetry.addData("Distance", String.format("%.01f in", proximFront.getDistance(DistanceUnit.INCH)));
        telemetry.addData("-------- Dev Tools --------", "");
        telemetry.addData("Currently Recording", recordingToggle);
        telemetry.addData("Currently Reenacting", reenactToggle);
        telemetry.addData("Current ReenactFrame", reenactFrame);
        telemetry.addData("CurrentTrackedValues", convertListToString(recordingData));
        telemetry.addData("-------- Internal Data --------", "");
        telemetry.addData("Servo Offset", hookOffset);
    }

    // Initialize the TensorFlow Object Detection engine.
    // Input-Output Structure: (void) => (void)
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.7;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    // Actual vuforia image detection method, will be used during Autonomous mode and dev test to see if image recognition is functional
    // Input-Output Structure: (void) => (void)
    public void vuforiaObjectDetection() {
      // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
      // first.
      initVuforia();

      if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
        initTfod();
      } else {
        telemetry.addData("Sorry!", "This device is not compatible with TFOD");
      }

      /**
        * Activate TensorFlow Object Detection before we wait for the start command.
        * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
      **/
      if (tfod != null) {
        tfod.activate();
      }

      /** Wait for the game to begin */
      telemetry.addData(">", "Press Play to start op mode");
      telemetry.update();
      waitForStart();

      if (opModeIsActive()) {
        while (opModeIsActive()) {
          if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
              telemetry.addData("# Object Detected", updatedRecognitions.size());

              // step through the list of recognitions and display boundary info.
              int i = 0;
              for (Recognition recognition : updatedRecognitions) {
                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f", recognition.getLeft(), recognition.getTop());
                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f", recognition.getRight(), recognition.getBottom());
              }
                logTelemetry();
                telemetry.update();
              }
            }
         }
      }

      if (tfod != null) {
        tfod.shutdown();
      }
    }
 }
