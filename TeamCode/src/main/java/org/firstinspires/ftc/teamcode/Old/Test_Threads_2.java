package org.firstinspires.ftc.teamcode.Old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

@Disabled
@Autonomous
public class Test_Threads_2 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private ExecutorService executor;
    private AtomicInteger counter;

    @Override
    public void runOpMode() throws InterruptedException {
        // Create a thread pool with two threads
        executor = Executors.newFixedThreadPool(2);

        // Create a shared counter that is incremented by the threads
        counter = new AtomicInteger();

        // Wait for the start button to be pressed
        waitForStart();
        runtime.reset();

        // Create and submit the first task to the thread pool
        executor.submit(() -> {
            // Count from 0 to 10

        });

        // Create and submit the second task to the thread pool
        executor.submit(() -> {
            // Count from 0 to 10

        });

        // Wait for the stop button to be pressed
        try {
            Thread.sleep(2000);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }
        // Shut down the thread pool
        executor.shutdown();
        executor.awaitTermination(1, TimeUnit.MINUTES);
    }
}
