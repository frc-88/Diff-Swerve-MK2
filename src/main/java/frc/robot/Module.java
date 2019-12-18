package frc.robot;

public class Module {
    
    private String name;
    private Motor posMotor;
    private Motor negMotor;

    public Module(String name, Motor posMotor, Motor negMotor) {
        this.name = name;
        this.posMotor = posMotor;
        this.negMotor = negMotor;
    }

    public void update() {
        this.posMotor.update();
        this.negMotor.update();
    }
}