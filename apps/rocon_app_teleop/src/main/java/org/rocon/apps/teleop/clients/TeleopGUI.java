package org.rocon.apps.teleop.clients;

import java.awt.*;
import java.awt.event.*;
import java.awt.image.BufferedImage;
import java.io.*;
import java.util.zip.*;

import javax.imageio.ImageIO;
import javax.sound.sampled.*;
import javax.swing.*;

import org.apache.commons.logging.Log;
import org.ros.exception.ParameterNotFoundException;
import org.ros.message.MessageListener;
import org.ros.message.device_comms.KeyboardInput;
import org.ros.message.sensor_msgs.CompressedImage;
import org.ros.message.std_msgs.ByteMultiArray;
import org.ros.node.*;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;

import com.google.common.base.Preconditions;


/**
 * This rosjava {@link Node} provides a GUI for teleoperating a remote robot.
 * It receives and reproduces audio and video, and sends control commands to
 * the robot. It assumes an external roscore is already running.
 *
 * @author jorge
 *
 */
public class TeleopGUI extends WindowAdapter implements NodeMain, KeyListener {

    private static final int WIDTH = 640, HEIGHT = 480;

    private Log  log;
    private Node node;
    private ParameterTree params;
    private Publisher<org.ros.message.std_msgs.String>            cmdPublisher;
    private Publisher<org.ros.message.device_comms.KeyboardInput> keyPublisher;

    private JLabel video;
    private JLabel seqNb;  // video frame's sequence number
    private JFrame frame;

    private VideoWatchdog vwd = new VideoWatchdog();

    private SourceDataLine line;

    private boolean running = true;


    @Override
    public void main(Node node) {

        Preconditions.checkState(this.node == null);
        this.node = node;

        log = node.getLog();
        params = node.newParameterTree();

        try {
            initGUI();
            initTeleOp();
            initKeyOp();
            playAudio();
            playVideo();
        } catch (Exception e) {
            log.error("Unexpected error", e);
            System.exit(-1);
        }
    }

    /**
     * Get the format of the audio to be played, provided as ROS parameters.
     * @return Audio format object.
     * @throws ParameterNotFoundException if any of the audio parameters is not
     * found.
     */
    private AudioFormat getFormat() throws ParameterNotFoundException {

        float sampleRate  = params.getInteger("/audio/sample_rate");
        int   sampleSize  = params.getInteger("/audio/sample_size"); // in bits
        int   channels    = params.getInteger("/audio/channels");
        boolean signed    = params.getBoolean("/audio/signed");
        boolean bigEndian = params.getBoolean("/audio/big_endian");

        return new AudioFormat(sampleRate, sampleSize, channels, signed, bigEndian);
    }

    /**
     * Subscribes to an audio provider node and plays messages as they come.
     */
    private void playAudio() {

        try {
            final AudioFormat format = getFormat();

            DataLine.Info info = new DataLine.Info(SourceDataLine.class, format);
            if (!AudioSystem.isLineSupported(info)) {
                log.error("Line [" + info.toString() + "] not supported");
                System.exit(-1);
            }

            // Select the mixer that will provide our line
            Mixer.Info mixers[] = AudioSystem.getMixerInfo();
            log.debug("Selecting audio mixer (" + mixers.length + " available)...");
            for (Mixer.Info mi : mixers) {
                String desc = mi.getDescription();
                if ((desc.toLowerCase().indexOf("default, ") > 0) ||
                    (desc.toLowerCase().indexOf("ear-candy") > 0)) {
                    // Avoid using; these mixers are known to capture very poorly
                    log.debug("[SKIPPED]      " + desc);
                    continue;
                }

                try {
                    line = AudioSystem.getSourceDataLine(format, mi);
                } catch(LineUnavailableException e){
                    log.debug("[UNAVAILABLE]  " + desc);
                    continue;
                } catch(IllegalArgumentException e){
                    log.debug("[UNSUPPORTED]  " + desc);
                    continue;
                }

                log.debug("[SELECTED]     " + desc);
                break;
            }

            if (line == null) {
                // This case is almost impossible (if you have an audio card!)
                log.error("No appropriate mixer has been found");
                System.exit(-1);
            }

            line.open(format);
            line.start();

            node.newSubscriber("/client/audio", "std_msgs/ByteMultiArray",
                    new MessageListener<ByteMultiArray>() {

                int    seq = -1;         // Sequence number
                int    skip = 0;         // Messages to skip
                long   lastMsgTime = 0;  // Reception time of last incoming message
                double avgTimeBMsg = 0;  // Average time between incoming messages

                // Decompress the data
                byte[] buf = new byte[line.getBufferSize()/5];

                @Override
                public void onNewMessage(ByteMultiArray message) {
                    seq++;

                    long thisMsgTime = new java.util.Date().getTime();

                    if (seq >= 100) {
                        // Control the time between incoming messages; if it's to long
                        // (compared with the historical average) discard some messages
                        // to keep pace with the input audio stream (if not, the audio
                        // gets out of sync)
                        long timeBMsg = new java.util.Date().getTime() - lastMsgTime;
                        if ((skip > 0) || (timeBMsg > Math.round(avgTimeBMsg*2))) {
                            if (skip > 0) {
                                // We are already out of sync; continue skipping messages
                                skip--;
                            }
                            else {
                                // Out of sync! estimate how much audio time we must discard
                                 skip = (int)Math.ceil((timeBMsg - avgTimeBMsg)/avgTimeBMsg);
                                 skip++;
                                log.warn("Audio out of sync; skip " + skip + " messages");
                            }

                            lastMsgTime = thisMsgTime;
                            return;
                        }
                    }

                    // Coarse method to estimate the average time between incoming messages
                    if (seq > 100)
                        avgTimeBMsg = avgTimeBMsg*0.999 + (thisMsgTime - lastMsgTime)*0.001;
                    else if (seq > 50)
                        avgTimeBMsg = avgTimeBMsg*0.99  + (thisMsgTime - lastMsgTime)*0.01;
                    else if (seq > 1)
                        avgTimeBMsg = avgTimeBMsg*0.9   + (thisMsgTime - lastMsgTime)*0.1;
                    else if (seq == 1)
                        avgTimeBMsg = thisMsgTime - lastMsgTime;

                    lastMsgTime = thisMsgTime;

                    // Create the decompressor and give it the data to compress
                    Inflater decompressor = new Inflater();
                    decompressor.setInput(message.data);

                    // Create an expandable byte array to hold the decompressed data
                    ByteArrayOutputStream bos = new ByteArrayOutputStream(buf.length);

                    int count = 0;
                    while (!decompressor.finished()) {
                        try {
                            count = decompressor.inflate(buf);
                            bos.write(buf, 0, count);
                        } catch (DataFormatException e) {
                            log.warn("Wrong compressed data format", e);
                        }
                    }

                    decompressor.reset();
                    try { bos.close(); } catch (IOException e) { }

                    // Get the decompressed data
                    byte[] decompressedData = bos.toByteArray();

                    // Write message content on audio output line
                    line.write(decompressedData, 0, decompressedData.length);

                    if (seq == 0) {
                        // Start a separated thread (it's time consuming) for continuously
                        // drain audio output line as soon as we receive the first message
                        new Thread() {
                            public void run() { while (running == true) line.drain(); }
                        }.start();
                    }
                }
            });
        } catch (ParameterNotFoundException e) {
            log.error("Audio parameters not set", e);
            System.exit(-1);
        } catch (LineUnavailableException e) {
            log.error("Unavailable line", e);
            System.exit(-1);
        } catch (IllegalArgumentException e) {
            log.error("Unsuported line", e);
            System.exit(-1);
        }
    }

    /**
     * Connects to a video stream and shows frames as they come.
     */
    private void playVideo() {

        try {
            node.newSubscriber("/client/video", "sensor_msgs/CompressedImage",
                    new MessageListener<CompressedImage>() {
                @Override
                public void onNewMessage(CompressedImage message) {
                    try {
                        // Read message and draw the new frame onto the JLabel
                        BufferedImage image =
                                ImageIO.read(new ByteArrayInputStream(message.data));
                        video.getGraphics().drawImage(image, 0, 0, WIDTH, HEIGHT, null);

                        seqNb.setText("frame: " + message.header.seq);

                        vwd.reset();
                    } catch (IOException e) {
                        log.error("Read image message failed", e);
                    }
                }
            });
        } catch (Exception e) {
            log.error("Create video subscriber failed", e);
            System.exit(-1);
        }
    }

    /**
     * Creates the UI components and initializes them.
     */
    private void initGUI() {
        JPanel panel = new JPanel(new GridBagLayout());
        JButton backgrButton  = new JButton("Change Background");
        JButton callRobButton = new JButton("Call Robot");
        JButton dimissButton  = new JButton("Dismiss Robot");
        JButton takePhoButton = new JButton("Take Photo");
        JButton takePanButton = new JButton("Take Panorama");

        backgrButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                frame.requestFocus();

                org.ros.message.std_msgs.String message =
                	new org.ros.message.std_msgs.String();
                message.data = "change_background";
                log.info("Sending teleop. cmd. message: " + message.data);
                cmdPublisher.publish(message);
            }
        });
        callRobButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                frame.requestFocus();

                org.ros.message.std_msgs.String message =
                	new org.ros.message.std_msgs.String();
                message.data = "call";
                log.info("Sending teleop. cmd. message: " + message.data);
                cmdPublisher.publish(message);
            }
        });
        dimissButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                frame.requestFocus();

                org.ros.message.std_msgs.String message =
                	new org.ros.message.std_msgs.String();
                message.data = "dismiss";
                log.info("Sending teleop. cmd. message: " + message.data);
                cmdPublisher.publish(message);
            }
        });
        takePhoButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                frame.requestFocus();

                org.ros.message.std_msgs.String message =
                	new org.ros.message.std_msgs.String();
                message.data = "take photo";
                log.info("Sending teleop. cmd. message: " + message.data);
                cmdPublisher.publish(message);
            }
        });
        takePanButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                frame.requestFocus();

                org.ros.message.std_msgs.String message =
                	new org.ros.message.std_msgs.String();
                message.data = "take panorama";
                log.info("Sending teleop. cmd. message: " + message.data);
                cmdPublisher.publish(message);
            }
        });

        JTextArea help =
        		new JTextArea("Keyboard control:\n"
        				    + " Arrows incr/decr. velocity\n"
    			            + " Spacebar stops the robot\n"
                            + " e/d enable/disable motors");
        help.setEditable(false);
        seqNb = new JLabel("frame: 0");
        seqNb.setMinimumSize(new Dimension(100, 20));
        video = new JLabel("Waiting for video signal...", JLabel.CENTER);
        video.setPreferredSize(new Dimension(WIDTH, HEIGHT));
        video.setMinimumSize(new Dimension(WIDTH, HEIGHT));
        panel.add(video, new GridBagConstraints(0, 0, 1, 7, 0, 0,
                GridBagConstraints.WEST, GridBagConstraints.NONE,
                new Insets(20, 20, 20, 20), 0, 0));
        panel.add(help,  new GridBagConstraints(1, 5, 1, 1, 0, 0,
                GridBagConstraints.CENTER, GridBagConstraints.NONE,
                new Insets(20, 20, 20, 20), 0, 0));
        panel.add(seqNb, new GridBagConstraints(1, 6, 1, 1, 0, 0,
                GridBagConstraints.SOUTHWEST, GridBagConstraints.NONE,
                new Insets(20, -9, 20, 20), 0, 0));
        panel.add(backgrButton, new GridBagConstraints(1, 0, 1, 1, 0, 0,
                GridBagConstraints.CENTER, GridBagConstraints.HORIZONTAL,
                new Insets(20, 20, 20, 20), 0, 0));
        panel.add(callRobButton, new GridBagConstraints(1, 1, 1, 1, 0, 0,
                GridBagConstraints.CENTER, GridBagConstraints.HORIZONTAL,
                new Insets(20, 20, 20, 20), 0, 0));
        panel.add(dimissButton, new GridBagConstraints(1, 2, 1, 1, 0, 0,
                GridBagConstraints.CENTER, GridBagConstraints.HORIZONTAL,
                new Insets(20, 20, 20, 20), 0, 0));
        panel.add(takePhoButton, new GridBagConstraints(1, 3, 1, 1, 0, 0,
                GridBagConstraints.CENTER, GridBagConstraints.HORIZONTAL,
                new Insets(20, 20, 20, 20), 0, 0));
        panel.add(takePanButton, new GridBagConstraints(1, 4, 1, 1, 0, 0,
                GridBagConstraints.CENTER, GridBagConstraints.HORIZONTAL,
                new Insets(20, 20, 20, 20), 0, 0));

        frame = new JFrame();
        frame.getContentPane().add(panel);
        frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
        frame.addWindowListener(this);
        frame.setVisible(true);
        frame.setSize(WIDTH + 220, HEIGHT + 80);
        frame.setTitle("Teleoperation module");
        frame.pack();

        // Ensure that the main frame always has the focus so it listen for key events
        KeyboardFocusManager.getCurrentKeyboardFocusManager().clearGlobalFocusOwner();
        frame.addKeyListener(this);
        frame.requestFocus();
    }

    /**
     * Create a relay that re-publish incoming teleop. command messages. It is intended
     * to re-direct commands generated outside the robot concert (probably from an android
     * device) to the concert master.
     */
    private void initTeleOp() {
        try {
        	// Publisher for outgoing messages; the topic name must be the one specified
        	// in the interface file, as it's part of the application interface with the
        	// concert master
        	cmdPublisher = node.newPublisher("teleop_cmd", "std_msgs/String");

        	// Subscriber for incoming messages
            node.newSubscriber("/client/cmdop", "std_msgs/String",
                    new MessageListener<org.ros.message.std_msgs.String>() {
                @Override
                public void onNewMessage(org.ros.message.std_msgs.String message) {
                    log.info("Relaying incoming message: " + message.data);
                    cmdPublisher.publish(message);
                }
            });

            log.info("Teleoperation commands relay ready");
        } catch (Exception e) {
            log.error("Create teleoperation commands relay failed", e);
            System.exit(-1);
        }
    }

    /**
     * Create a publisher to transmit incoming key typed events to the robot.
     */
    private void initKeyOp() {
        try {
        	keyPublisher = node.newPublisher("/server/keyop", "device_comms/KeyboardInput");

            log.info("Reading from keyboard:");
            log.info("---------------------------");
            log.info("Forward/back arrows : linear velocity incr/decr.");
            log.info("Right/left arrows : angular velocity incr/decr.");
            log.info("Spacebar : reset linear/angular velocities.");
            log.info("e : enable / d : disable motors.");
        } catch (Exception e) {
            log.error("Create keyboard inputs publisher failed", e);
            System.exit(-1);
        }
    }

    @Override
    public void keyPressed(KeyEvent ke) {
        // Generate and publish a keyboard input message
        KeyboardInput kbi = new KeyboardInput();

        switch (ke.getKeyCode()) {
        case KeyEvent.VK_UP:
            kbi.pressedKey = KeyboardInput.KeyCode_Up;
            keyPublisher.publish(kbi);
            break;
        case KeyEvent.VK_DOWN:
            kbi.pressedKey = KeyboardInput.KeyCode_Down;
            keyPublisher.publish(kbi);
            break;
        case KeyEvent.VK_LEFT:
            kbi.pressedKey = KeyboardInput.KeyCode_Left;
            keyPublisher.publish(kbi);
            break;
        case KeyEvent.VK_RIGHT:
            kbi.pressedKey = KeyboardInput.KeyCode_Right;
            keyPublisher.publish(kbi);
            break;
        case KeyEvent.VK_SPACE:
            kbi.pressedKey = KeyboardInput.KeyCode_Space;
            keyPublisher.publish(kbi);
            break;
        case KeyEvent.VK_E:
            kbi.pressedKey = KeyboardInput.KeyCode_Enable;
            keyPublisher.publish(kbi);
            break;
        case KeyEvent.VK_D:
            kbi.pressedKey = KeyboardInput.KeyCode_Disable;
            keyPublisher.publish(kbi);
            break;
        }
    }

    @Override
    public void keyReleased(KeyEvent ke) {
        // TODO Auto-generated method stub
    }

    @Override
    public void keyTyped(KeyEvent ke) {
        // TODO Auto-generated method stub
    }

    /**
     * Catch window closing event so we can free up resources before exiting.
     *
     * @param e
     */
    public void windowClosing(WindowEvent e) {
        shutdown();
    }

    @Override
    public void shutdown() {
        try {
            running = false;

            // Close audio output line
            if (line != null) {
                line.stop();
                line.close();
            }

            // Dispose window and shutdown ROS node
            node.shutdown();
            node = null;

            frame.dispose();
            System.exit(0);
        } catch (Exception e) {
            log.error("Error on node shutdown", e);
        }
    }


    /**
     * Thread to ensure that video label doesn't freeze.
     * @author jorge
     */
    private class VideoWatchdog extends Thread {
        public void reset() {
            if (super.isAlive() == true)
                super.interrupt();
            else
                super.start();
        }

        public void run() {
            while (true)
                try { sleep(500); video.repaint(); } catch (InterruptedException e) { }
        }
    }
}

