package org.rocon.apps.teleop.servers;

import java.io.*;
import java.util.zip.Deflater;

import javax.sound.sampled.*;

import org.apache.commons.logging.Log;
import org.ros.exception.ParameterNotFoundException;
import org.ros.message.std_msgs.ByteMultiArray;
import org.ros.node.*;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;

import com.google.common.base.Preconditions;


/**
 * This is a simple rosjava {@link Publisher} {@link Node} that captures audio
 * from the system's default microphone and sends it packaged as ros messages.
 * It assumes an external roscore is already running.
 *
 * @author jorge
 */
public class AudioPublisher implements NodeMain {

    public static final int     SAMPLE_RATE = 16000;
    public static final int     SAMPLE_SIZE =    16; // in bits
    public static final int     CHANNELS    =     1;
    public static final boolean SIGNED      =  true;
    public static final boolean BIG_ENDIAN  =  true;

    private Log  log;
    private Node node;
    private TargetDataLine line;

    @Override
    public void main(Node node) {
        Preconditions.checkState(this.node == null);
        this.node = node;

        try {
            log = node.getLog();

            // Obtain a target (input) audio line with the parameterized format
            AudioFormat format = getFormat();

            DataLine.Info info = new DataLine.Info(TargetDataLine.class, format);
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
                    line = AudioSystem.getTargetDataLine(format, mi);
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

            byte buffer[] = new byte[line.getBufferSize()/5];
            byte compBuffer[] = new byte[buffer.length];

            // Create the compressor with highest level of compression
            Deflater compressor = new Deflater();
            compressor.setLevel(Deflater.BEST_COMPRESSION);

            // Publish audio messages as we record frames
            Publisher<ByteMultiArray> publisher =
                    node.newPublisher("/server/audio", "std_msgs/ByteMultiArray");

            log.info("Starting audio capture [" + format.toString() + "]");

            int seq = 0;
            while (true) {
                int count = line.read(buffer, 0, buffer.length);
                if (count > 0) {
                    // Give the compressor the data to compress
                    compressor.setInput(buffer);
                    compressor.finish();

                    ByteArrayOutputStream bos = new ByteArrayOutputStream(buffer.length);

                    // Compress the data
                    int compCount = 0;
                    while (!compressor.finished()) {
                        compCount = compressor.deflate(compBuffer);
                        bos.write(compBuffer, 0, compCount);
                    }

                    compressor.reset();
                    try {  bos.close();  } catch (IOException e) { }

                    // Get the compressed data
                    byte[] compData = bos.toByteArray();

                    ByteMultiArray message = new ByteMultiArray();
                    message.data = new byte[compData.length];
                    System.arraycopy(compData, 0, message.data, 0, message.data.length);

                    publisher.publish(message);
                }
                seq++;
            }
        } catch (ParameterNotFoundException e) {
            log.error("Audio parameters not set", e);
            System.exit(-1);
        } catch (Exception e) {
            log.fatal("Audio publisher failed", e);
            System.exit(-1);
        }
    }

    /**
     * Get the format of the audio to be played, provided as ROS parameters.
     * @return Audio format object.
     */
    private AudioFormat getFormat() {

        float   sampleRate = SAMPLE_RATE;
        int     sampleSize = SAMPLE_SIZE;
        int     channels   = CHANNELS;
        boolean signed     = SIGNED;
        boolean bigEndian  = BIG_ENDIAN;

        ParameterTree params = node.newParameterTree();

        // If a parameter is missing, use the default value and save it to the
        // parameters server to allow subscribers to properly play the received audio
        if (params.has("/audio/sample_rate") == false) {
            log.warn("Audio parameter missing (sample_rate); using default value: "
                    + SAMPLE_RATE);
            params.set("/audio/sample_rate", SAMPLE_RATE);
        }
        else
            sampleRate = params.getInteger("/audio/sample_rate");

        if (params.has("/audio/sample_size") == false) {
            log.warn("Audio parameter missing (sample_size); using default value: "
                    + SAMPLE_SIZE);
            params.set("/audio/sample_size", SAMPLE_SIZE);
        }
        else
            sampleSize  = params.getInteger("/audio/sample_size");

        if (params.has("/audio/channels")      == false) {
            log.warn("Audio parameter missing (channels); using default value: "
                    + CHANNELS);
            params.set("/audio/channels",    CHANNELS);
        }
        else
            channels    = params.getInteger("/audio/channels");

        if (params.has("/audio/signed")      == false) {
            log.warn("Audio parameter missing (signed); using default value: "
                    + SIGNED);
            params.set("/audio/signed",      SIGNED);
        }
        else
            signed    = params.getBoolean("/audio/signed");

        if (params.has("/audio/big_endian")  == false) {
            log.warn("Audio parameter missing (big_endian); using default value: "
                    + BIG_ENDIAN);
            params.set("/audio/big_endian",  BIG_ENDIAN);
        }
        else
            bigEndian = params.getBoolean("/audio/big_endian");

        return new AudioFormat(sampleRate, sampleSize, channels, signed, bigEndian);
    }

    @Override
    public void shutdown() {
        line.stop();
        line.close();

        node.shutdown();
        node = null;
    }
}
