package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.cals.PneumaticsCals;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pneumatics extends SubsystemBase{
    
    private Compressor mCompressor;
    private PneumaticsCals mCals;
    private double pauseTime;
    private boolean paused;
    public AnalogInput pressureSensor;
    public double pressure;

    public Pneumatics(PneumaticsCals cals){
        mCals = cals;
        if(mCals.disabled) return;
        mCompressor = new Compressor();
        pressureSensor = new AnalogInput(mCals.PNE_PSENSORID);
        pressureSensor.setAverageBits(8);
    }

    public void pauseReq(boolean breakThresh){
        if(breakThresh){
            paused = true;
            pauseTime = Timer.getFPGATimestamp() + mCals.pauseTime;
        }
    }

    @Override
    public void periodic(){
        if(mCals.disabled) return;
        if(Timer.getFPGATimestamp() > pauseTime){
            paused = false;
        }

        pressure = .5 * (pressureSensor.getAverageVoltage() - 0.35) * 115 / (1.82-0.35);

        if(paused && pressure > mCals.minPressure){
            mCompressor.stop();
        } else if(pressure > mCals.maxPressure){
            mCompressor.stop();
        } else if(pressure < mCals.maxPressure - mCals.hystPressure){
            mCompressor.start();
        }

        Display.put("Pressure", pressure);
        Display.put("CompressorRun", mCompressor.getCompressorCurrent() > 4);
        //SmartDashboard.putNumber("CompCurr",mCompressor.getCompressorCurrent());
    }
}