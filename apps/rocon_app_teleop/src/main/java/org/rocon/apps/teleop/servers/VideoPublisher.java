package org.rocon.apps.teleop.servers;

import java.awt.event.*;
import java.awt.image.BufferedImage;

import javax.swing.*;

import org.apache.commons.logging.Log;
import org.ros.message.Time;
import org.ros.message.sensor_msgs.CompressedImage;
import org.ros.node.*;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;

import au.edu.jcu.v4l4j.*;
import au.edu.jcu.v4l4j.exceptions.*;

import com.google.common.base.Preconditions;

/**
 * This class demonstrates how to perform a simple push-mode capture.
 * It starts the capture and display the video stream in a JLabel
 * @author jorge
 *
 */
public class VideoPublisher extends WindowAdapter implements CaptureCallback, NodeMain {

    private static int    WIDTH    = 320;
    private static int    HEIGHT   = 240;
    private static int    STANDARD = V4L4JConstants.STANDARD_WEBCAM;
    private static int    CHANNEL  = 0;
    private static int    QUALITY  = 40;
    private static int    FR_RATE  = 5;
    private static String DEVICE   = "/dev/video0";

    private VideoDevice   videoDevice;
    private FrameGrabber  frameGrabber;

    private JLabel        label;
    private JFrame        frame;

    private Log           log;
    private Node          node;
    private Publisher<CompressedImage> publisher;


    @Override
    public void main(Node node) {
        Preconditions.checkState(this.node == null);
        this.node = node;

        try {
            log = node.getLog();

            // Create a publisher to transmit incoming video frames
            publisher = node.newPublisher("/server/video", "sensor_msgs/CompressedImage");

            // Create and initialize UI
            initGUI();
        } catch (Exception e) {
            log.fatal("Video publisher initialization failed", e);
            System.exit(-1);
        }

        // Initialize video device and frame grabber
        try {
            initFrameGrabber();
        } catch (V4L4JException e) {
            log.error("Setting up video capture failed", e);

            // cleanup and exit
            cleanupCapture();
            System.exit(-1);
        }

        // Start video capture
        try {
            frameGrabber.startCapture();
        } catch (V4L4JException e){
            log.error("Starting video capture failed", e);

            // cleanup and exit
            cleanupCapture();
            System.exit(-1);
        }
    }

    /**
     * Initializes the FrameGrabber object
     * @throws V4L4JException if any parameter if invalid
     */
    private void initFrameGrabber() throws V4L4JException {
        ParameterTree params = node.newParameterTree();

        DEVICE  = params.getString ("/video/device",  DEVICE);
        WIDTH   = params.getInteger("/video/width",   WIDTH);
        HEIGHT  = params.getInteger("/video/height",  HEIGHT);
        QUALITY = params.getInteger("/video/quality", QUALITY);
        FR_RATE = params.getInteger("/video/fr_rate", FR_RATE);

        videoDevice = new VideoDevice(DEVICE);
        frameGrabber = videoDevice.getJPEGFrameGrabber(WIDTH, HEIGHT, CHANNEL,
                                                       STANDARD, QUALITY);
        frameGrabber.setCaptureCallback(this);
        frameGrabber.setFrameInterval(1, FR_RATE);
        WIDTH  = frameGrabber.getWidth();  // Sometimes it corrects our
        HEIGHT = frameGrabber.getHeight(); // misguided configurations...
        log.info("Starting video capture ["+ WIDTH + "x" + HEIGHT
               + "  " + FR_RATE + " fps  jpeg format]");
    }

    /**
     * Creates the UI components and initializes them
     */
    private void initGUI(){
        frame = new JFrame();
        label = new JLabel();
        frame.getContentPane().add(label);
        frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
        frame.addWindowListener(this);
        frame.setVisible(true);
        frame.setSize(WIDTH, HEIGHT);
        frame.setTitle("Video Publisher");
    }

    /**
     * this method stops the capture and releases the frame grabber and video device
     */
    private void cleanupCapture() {
        try {
            frameGrabber.stopCapture();
        } catch (StateException ex) {
            // the frame grabber may be already stopped, so we just ignore
            // any exception and simply continue.
        }

        // release the frame grabber and video device
        videoDevice.releaseFrameGrabber();
        videoDevice.release();
    }

    /**
     * Catch window closing event so we can free up resources before exiting
     * @param e
     */
    public void windowClosing(WindowEvent e) {
        shutdown();
    }

    @Override
    public void shutdown() {
        cleanupCapture();

        node.shutdown();
        node = null;

        // close window
        frame.dispose();
        System.exit(0);
    }

    @Override
    public void exceptionReceived(V4L4JException e) {
        // This method is called by v4l4j if an exception
        // occurs while waiting for a new frame to be ready.
        // The exception is available through e.getCause()
        e.printStackTrace();
    }

    @Override
    public void nextFrame(VideoFrame frame) {
        // This method is called when a new frame is ready.
        // Don't forget to recycle it when done dealing with the frame.

        try {
            // Draw the new frame onto the JLabel
            BufferedImage image = frame.getBufferedImage();
            label.getGraphics().drawImage(image, 0, 0, WIDTH, HEIGHT, null);

            // Get the position of the jpeg end marker (0xFF 0xD9)
            // Ignore the value given by getFrameLength, as it's always wrong!
            byte [] buffer = frame.getBytes();
            int pos = 0;
            for (pos = 10; pos < buffer.length - 10; pos++) {
                if ((buffer[pos] == -1) && (buffer[pos + 1] == -39))
                    break;
            }

            if (pos >= buffer.length - 10)
                log.warn("JPEG EOI marker not found on frame "
                         + frame.getSequenceNumber() + "; skipped");
            else {
                // Fulfill and publish a compressed image message
                CompressedImage message = new CompressedImage();
                message.header.seq = frame.getSequenceNumber();
                message.header.stamp = new Time(frame.getCaptureTime()/1000000.0);
                message.format = "jpeg";
                message.data = new byte[pos + 2];
                System.arraycopy(buffer, 0, message.data, 0, message.data.length);

                publisher.publish(message);
            }

            // recycle the frame
            frame.recycle();
        }
        catch (ArrayIndexOutOfBoundsException e) {
            log.error("Array index out of bounds", e);
        }
        catch (Exception e) {
            log.error("Process video frame failed", e);
        }
    }
}