package frc.robot.modules;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LEDs {
    private Spark ledController;

    public LEDs(int id) {
        ledController = new Spark(id);
    }

    public void setRainbow() {
        ledController.set(-0.99);
    }

    public void setPartyRainbow() {
        ledController.set(-0.97);
    }

    public void setOceanRainbow() {
        ledController.set(-0.95);
    }

    public void setLavaRainbow() {
        ledController.set(-0.93);
    }

    public void setForestRainbow() {
        ledController.set(-0.91);
    }

    public void setGlitterRainbow() {
        ledController.set(-0.89);
    }

    public void setConfetti() {
        ledController.set(-0.87);
    }

    public void setShotRed() {
        ledController.set(-0.85);
    }

    public void setShotBlue() {
        ledController.set(-0.83);
    }

    public void setShotWhite() {
        ledController.set(-0.81);
    }

    public void setSinelonRainbow() {
        ledController.set(-0.79);
    }

    public void setSinelonParty() {
        ledController.set(-0.77);
    }

    public void setSinelonOcean() {
        ledController.set(-0.75);
    }

    public void setSinelonLava() {
        ledController.set(-0.73);
    }

    public void setSinelonForest() {
        ledController.set(-0.71);
    }

    public void setBPMRainbow() {
        ledController.set(-0.69);
    }

    public void setBPMParty() {
        ledController.set(-0.67);
    }

    public void setBPMOcean() {
        ledController.set(-0.65);
    }

    public void setBPMLava() {
        ledController.set(-0.63);
    }

    public void setBPMForest() {
        ledController.set(-0.61);
    }

    public void setFireMedium() {
        ledController.set(-0.59);
    }

    public void setFireLarge() {
        ledController.set(-0.57);
    }

    public void setTwinkleRainbow() {
        ledController.set(-0.55);
    }

    public void setTwinkleParty() {
        ledController.set(-0.53);
    }

    public void setTwinkleOcean() {
        ledController.set(-0.51);
    }

    public void setTwinkleLava() {
        ledController.set(-0.49);
    }

    public void setTwinkleForest() {
        ledController.set(-0.47);
    }

    public void setColorWaveRainbow() {
        ledController.set(-0.45);
    }

    public void setColorWaveParty() {
        ledController.set(-0.43);
    }

    public void setColorWaveOcean() {
        ledController.set(-0.41);
    }

    public void setColorWaveLava() {
        ledController.set(-0.39);
    }

    public void setColorWaveForest() {
        ledController.set(-0.37);
    }

    public void setLarsonScanRed() {
        ledController.set(-0.35);
    }

    public void setLarsonScanGray() {
        ledController.set(-0.33);
    }

    public void setLightChaseRed() {
        ledController.set(-0.31);
    }

    public void setLightChaseBlue() {
        ledController.set(-0.29);
    }

    public void setLightChaseGray() {
        ledController.set(-0.27);
    }

    public void setHeartbeatRed() {
        ledController.set(-0.25);
    }

    public void setHeartbeatBlue() {
        ledController.set(-0.23);
    }

    public void setHeartbeatWhite() {
        ledController.set(-0.21);
    }

    public void setHeartbeatGray() {
        ledController.set(-0.19);
    }

    public void setBreathRed() {
        ledController.set(-0.17);
    }

    public void setBreathBlue() {
        ledController.set(-0.15);
    }

    public void setBreathGray() {
        ledController.set(-0.13);
    }

    public void setStrobeRed() {
        ledController.set(-0.11);
    }

    public void setStrobeBlue() {
        ledController.set(-0.09);
    }

    public void setStrobeGold() {
        ledController.set(-0.07);
    }

    public void setStrobeWhite() {
        ledController.set(-0.05);
    }

    public void setE2Eb2bC1() {
        ledController.set(-0.03);
    }

    public void setLarsonScannerC1() {
        ledController.set(-0.01);
    }

    public void setLightChaseC1() {
        ledController.set(0.01);
    }

    public void setHeartbeatSlowC1() {
        ledController.set(0.03);
    }

    public void setHeartbeatMediumC1() {
        ledController.set(0.05);
    }

    public void setHeartbeatFastC1() {
        ledController.set(0.07);
    }

    public void setBreathSlowC1() {
        ledController.set(0.09);
    }

    public void setBreathFastC1() {
        ledController.set(0.11);
    }

    public void setShotC1() {
        ledController.set(0.13);
    }

    public void setStrobeC1() {
        ledController.set(0.15);
    }

    public void setE2Eb2bC2() {
        ledController.set(0.17);
    }

    public void setLarsonScannerC2() {
        ledController.set(0.19);
    }

    public void setLightChaseC2() {
        ledController.set(0.21);
    }

    public void setHeartbeatSlowC2() {
        ledController.set(0.23);
    }

    public void setHeartbeatMediumC2() {
        ledController.set(0.25);
    }

    public void setHeartbeatFastC2() {
        ledController.set(0.27);
    }

    public void setBreathSlowC2() {
        ledController.set(0.29);
    }

    public void setBreathFastC2() {
        ledController.set(0.31);
    }

    public void setShotC2() {
        ledController.set(0.33);
    }

    public void setStrobeC2() {
        ledController.set(0.35);
    }

    public void setSparkleC1_2() {
        ledController.set(0.37);
    }

    public void setSparkle2_1() {
        ledController.set(0.39);
    }

    public void setGradient1_2() {
        ledController.set(0.41);
    }

    public void setBPM1_2() {
        ledController.set(0.43);
    }

    public void setE2E1_2() {
        ledController.set(0.45);
    }

    public void setE2E() {
        ledController.set(0.47);
    }

    public void setNoblend1_2() {
        ledController.set(0.49);
    }

    public void setTwinkles1_2() {
        ledController.set(0.51);
    }

    public void setColorwaves1_2() {
        ledController.set(0.53);
    }

    public void setSinelon1_2() {
        ledController.set(0.55);
    }

    public void setHotPink() {
        ledController.set(0.57);
    }

    public void setDarkRed() {
        ledController.set(0.59);
    }

    public void setRed() {
        ledController.set(0.61);
    }

    public void setRedOrange() {
        ledController.set(0.63);
    }

    public void setOrange() {
        ledController.set(0.65);
    }

    public void setGold() {
        ledController.set(0.67);
    }

    public void setYellow() {
        ledController.set(0.69);
    }

    public void setLawnGreen() {
        ledController.set(0.71);
    }

    public void setLime() {
        ledController.set(0.73);
    }

    public void setDarkGreen() {
        ledController.set(0.75);
    }

    public void setGreen() {
        ledController.set(0.77);
    }

    public void setBlueGreen() {
        ledController.set(0.79);
    }

    public void setAqua() {
        ledController.set(0.81);
    }

    public void setSkyBlue() {
        ledController.set(0.83);
    }

    public void setDarkBlue() {
        ledController.set(0.85);
    }

    public void setBlue() {
        ledController.set(0.87);
    }

    public void setBlueViolet() {
        ledController.set(0.89);
    }

    public void setViolet() {
        ledController.set(0.91);
    }

    public void setWhite() {
        ledController.set(0.93);
    }

    public void setGray() {
        ledController.set(0.95);
    }

    public void setDarkGray() {
        ledController.set(0.97);
    }

    public void setBlack() {
        ledController.set(0.99);
    }

}
