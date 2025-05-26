package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.Collection;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import static frc.robot.subsystems.drive.DriveConstants.kOdometryFrequencyHz;

/**An odometry thread designed to work with SparkMax motor controllers. This is a singleton class, only one instance exists at any given time*/
public class OdometryThread {

    //SHARED RESOURCES
    private final ArrayList<DoubleSupplier> signals = new ArrayList<>();
    private final ArrayList<BooleanSupplier> errorSignals = new ArrayList<>();

    private final ArrayList<ConcurrentLinkedQueue<Double>> signalQueues = new ArrayList<>();
    private final ArrayList<ConcurrentLinkedQueue<Long>> timestampQueues = new ArrayList<>();

    private final Lock signalLock = new ReentrantLock();

    private static OdometryThread instance = null;

    private final Notifier notifier = new Notifier(this::run);

    //ODOMETRY THREAD ONLY
    private final AtomicInteger samplesSinceLastPoll = new AtomicInteger(0);

    //MAIN THREAD ONLY
    public int sampleCount = 0;

    private OdometryThread(){
        notifier.setName("OdometryThread");
    }

    public static OdometryThread getInstance() {
        if (instance == null){
            instance = new OdometryThread();
        }
        return instance;
    }

    public void start(){
        notifier.startPeriodic(1.0/kOdometryFrequencyHz);
    }

    /**Registers a signal from the main thread*/
    public ConcurrentLinkedQueue<Double> registerSignal(DoubleSupplier signal){
        ConcurrentLinkedQueue<Double> queue = new ConcurrentLinkedQueue<>();
        signalLock.lock();
        try {
            signals.add(signal);
            signalQueues.add(queue);
        } catch (Exception e) {
            throw e;
        } finally {
            signalLock.unlock();
        }
        return queue;
    }

    /**Registers an error signal from the main thread. If any error signals are detected, then the thread does not register odometry for any devices during the given odometry cycle */
    public void registerErrorSignal(BooleanSupplier errorSignal){
        signalLock.lock();
        try {
            errorSignals.add(errorSignal);
        } catch (Exception e) {
            throw e;
        } finally {
            signalLock.unlock();
        }
    }

    /**Makes a timestamp queue. Timestamps are recorded in microseconds as a Long */
    public ConcurrentLinkedQueue<Long> makeTimestampQueue(){
        ConcurrentLinkedQueue<Long> queue = new ConcurrentLinkedQueue<>();
        signalLock.lock();
        try {
            timestampQueues.add(queue);
        } catch (Exception e) {
            throw e;
        }
        return queue;
    }

    /**Periodic function to run in the odometry thread, updates queues with latest odometry values*/
    private void run(){

        long timestamp = RobotController.getFPGATime();
        signalLock.lock();
        try {
            for (int i = 0; i < errorSignals.size(); i++){
                if (errorSignals.get(i).getAsBoolean()){
                    return;
                }
            }
            for (int i = 0; i < signals.size(); i++){
                signalQueues.get(i).offer(signals.get(i).getAsDouble());
            }
            for (int i = 0; i < timestampQueues.size(); i++){
                timestampQueues.get(i).offer(timestamp);
            }
        } catch (Exception e) {
            throw e;
        } finally {
            signalLock.unlock();
        }
        samplesSinceLastPoll.incrementAndGet(); //All queues are GUARANTEED to have at least samplesSinceLastPoll elements in them, all correctly ordered

    }

    /** This method should be called ONCE per main-cycle thread */
    public void poll() {
        sampleCount = samplesSinceLastPoll.getAndSet(0);
    }

    /** Safely reads N elements of type T from a queue into a collection. Will not block if queue is empty*/
    public static <T> void safeDrain(ConcurrentLinkedQueue<T> source, Collection<T> dest, int n) {
        for (int i = 0; i < n; i++){
            dest.add(source.poll());
        }
    }
    
}
