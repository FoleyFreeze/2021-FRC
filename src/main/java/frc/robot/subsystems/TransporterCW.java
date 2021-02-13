package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.cals.CWheelCals;
import frc.robot.cals.TransporterCals;
import frc.robot.motors.Motor;
import frc.robot.util.LimitedList;

public class TransporterCW extends SubsystemBase{

    public Motor rotateMotor;
    public Motor loadMotor;
    public TransporterCals tCals;
    public CWheelCals cCals;
    public AnalogInput ballSensor;
    public ColorSensorV3 colorSensor;
    public Solenoid launcher;
    public Solenoid CWNotTransport;
    private double targetpos = 0;
    private boolean[] ballpositions = {false, false, false, false, false};
    public boolean isIndexing;
    public int ballnumber;
    private RobotContainer mSubsystem;
    public ColorMatch colorMatch;
    public String colorString;
    public Color detectedColor;
    public Color lastColor;
    public String gameData;
    public LimitedList<String> gameDataList;

    public TransporterCW(TransporterCals tCals, CWheelCals cCals, RobotContainer subsystem){
        this.tCals = tCals;
        this.cCals = cCals;
        mSubsystem = subsystem;
        if(tCals.disabled && cCals.disabled) return;

        rotateMotor = Motor.initMotor(tCals.rotateMotor);
        loadMotor = Motor.initMotor(tCals.loadMotor);
        ballSensor = new AnalogInput(tCals.sensorValue);
        ballSensor.setAverageBits(4);
        colorSensor = new ColorSensorV3(Port.kOnboard);
        launcher = new Solenoid(tCals.launcherValue);
        CWNotTransport = new Solenoid(tCals.CWNotTransport);
        colorMatch = new ColorMatch();

        colorMatch.addColorMatch(cCals.Blue);
        colorMatch.addColorMatch(cCals.Green);
        colorMatch.addColorMatch(cCals.Red);
        colorMatch.addColorMatch(cCals.Yellow);

        //this is a hack
        gameDataList = new LimitedList<>(2);
        gameDataList.addFirst(DriverStation.getInstance().getGameSpecificMessage());
        Thread t = new Thread(new Runnable(){
            public void run(){
                while(true){
                    try{
                        Thread.sleep(1000);
                        gameDataList.addFirst(DriverStation.getInstance().getGameSpecificMessage());
                    } catch (Exception e){

                    }
                }
            }
        });
        t.start();
    }

    public void autonInit(){
        first = true;
    }

    boolean first = true;
    double jamTime;
    boolean prevJammed;
    public void periodic(){
        if(first && DriverStation.getInstance().isAutonomous()){
            first = false;

            //TODO: This should only be 3 for the skills challenge
            //but probably doesn't matter that much overall
            ballnumber = 5;
            ballpositions[0] = true;
            ballpositions[1] = true;
            ballpositions[2] = true;
            ballpositions[3] = true;
            ballpositions[4] = true;
        }

        //gameData = gameDataList.getFirst();

        if(tCals.disabled && cCals.disabled) return;

        if(!mSubsystem.m_input.cwActivate()){
            CWNotTransport.set(false);
        }

        /*if(Timer.getFPGATimestamp() > jamTime){
            if(prevJammed) {
                index(1);
                prevJammed = false;
            }
        } 
        if(rotateMotor.isJammed() && !prevJammed){
            jamTime = Timer.getFPGATimestamp() + tCals.jamRestTime;
            index(-2);
            prevJammed = true;
        }*/

        int x = (int) Math.round(rotateMotor.getPosition() / tCals.countsPerIndex);
        if(x < 0) x = 5 - Math.abs(x%5);

        if(rotateMotor.isJammed() && !prevJammed){
            prevJammed = true;
            if(targetpos - rotateMotor.getPosition() > tCals.countsPerIndex/2){
                index(-1);
            }
        }

        if(prevJammed){
            rotateMotor.setPower(0);
        } else {
            rotateMotor.setPosition(targetpos);
        }
        

        if(ballpositions[(x + 1) % 5] && launcher.get()){
            ballnumber--;
            ballpositions[(x + 1) % 5] = false;
        }

        if(mSubsystem.m_intake.isOut() && !ballpositions[x%5]){
            gatePower(tCals.TN_LOADSPEED);
        }else if(mSubsystem.m_intake.isOut()) {
            gatePower(tCals.TN_STOPSPEED);
        }else if(CWNotTransport.get()){
            if(mSubsystem.m_input.cwRotNotPos()){
                gatePower(cCals.rotSpeed);
            } else{
                gatePower(cCals.colSpeed);
            }
        } else gatePower(0.0);

        /*
        detectedColor = colorSensor.getColor();
        ColorMatchResult match = colorMatch.matchClosestColor(detectedColor);
        if(match.color == cCals.Blue){
            colorString = "Blue";
        } else if(match.color == cCals.Green){
            colorString = "Green";
        } else if(match.color == cCals.Red){
            colorString = "Red";
        } else if(match.color == cCals.Yellow){
            colorString = "Yellow";
        } else colorString = "N/A";
        */

        Display.put("Current Pos", rotateMotor.getPosition() / tCals.countsPerIndex);
        Display.put("Ball Number", ballnumber);
        Display.put("Ball Position 0", ballpositions[0]);
        Display.put("Ball Position 1", ballpositions[1]);
        Display.put("Ball Position 2", ballpositions[2]);
        Display.put("Ball Position 3", ballpositions[3]);
        Display.put("Ball Position 4", ballpositions[4]);
        Display.put("TCMotorCurrent0", rotateMotor.getCurrent());
        Display.put("TCMotorCurrent1", loadMotor.getCurrent());
        Display.put("TC Motor Temp 0", rotateMotor.getTemp());
        Display.put("TC Motor Temp 1", loadMotor.getTemp());
        Display.put("BallSenseV", ballSensor.getAverageVoltage());
        Display.put("Color Info", String.format("R: %d G: %d B: %d IR: %d Prox: %d", 
            colorSensor.getRed(), colorSensor.getGreen(), colorSensor.getBlue(), 
            colorSensor.getIR(), colorSensor.getProximity()));
        /* Display.put("Detected Color", String.format("Color Guess: " + colorString + 
            " Confidence: %f", match.confidence)); */
        lastColor = detectedColor;
    }

    //increment the ball storage
    public void index(double positions){
        targetpos += positions * tCals.countsPerIndex;
    }

    public boolean isIndexing(){
        if(tCals.disabled) return false;
        double error = targetpos - rotateMotor.getPosition();
        return Math.abs(error) > tCals.allowedIndexError;
    }

    double waitTime = 0;
    double prevTime = 0;
    //used for gathering
    public void gatherIndex(){
        if(tCals.disabled) return;
        double error = targetpos - rotateMotor.getPosition();
        double time = Timer.getFPGATimestamp();
        if(hasBall() && Math.abs(error) < tCals.countsPerIndex / 2 && ballnumber < tCals.maxBallCt){ //only spin if not moving & we have an open spot
            waitTime += time - prevTime;
            if(waitTime > tCals.ballSenseDelay){
                waitTime = 0;
                ballnumber++;
                int x = (int) Math.round(rotateMotor.getPosition() / tCals.countsPerIndex);
                if(x < 0) x = 5 - Math.abs(x%5);
                ballpositions[x % 5] = true;
                if(ballnumber < tCals.maxBallCt) {
                    index(1);
                }
            }
        }
        SmartDashboard.putNumber("waittime",waitTime);
        prevTime = time;
    }

    double restTime = 0;
    public void gatePower(double power){
        if(restTime > Timer.getFPGATimestamp()){
            loadMotor.setPower(tCals.TN_STOPSPEED);
        } else {
            loadMotor.setPower(power);
        }

        if(loadMotor.getCurrent() > tCals.maxGateCurr){
            restTime = Timer.getFPGATimestamp() + tCals.gateRestTime;
        }
    }

    public void shootAll(){
        if(tCals.disabled) return;
        //enablefire(ballnumber > 0);
        double error = targetpos - rotateMotor.getPosition();
        if(Math.abs(error) < tCals.countsPerIndex && ballnumber > 0){
            index(1);
        }
    }

    //stop shooting
    public void stoprot(){
        if(tCals.disabled) return;
        enablefire(false);
        double x = rotateMotor.getPosition() / tCals.countsPerIndex;
        x = Math.round(x);
        targetpos = x * tCals.countsPerIndex;
    }

    public void enablefire(boolean on){
        if(tCals.disabled) return;
        launcher.set(on);
    }

    public void deployCW(boolean activated){
        if(tCals.disabled) return;
        CWNotTransport.set(activated);
    }

    public boolean hasBall(){
        double volts = ballSensor.getAverageVoltage();
        return volts > tCals.hasBallMinV && volts < tCals.hasBallMaxV;
    }

    public void resetJammed(){
        prevJammed = false;
    }
}