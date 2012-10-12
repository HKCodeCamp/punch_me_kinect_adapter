// TrackerPanel.java
// Andrew Davison, November 2011, ad@fivedots.psu.ac.th

/* Panel that shows the Kinect camera image, and displays multiple
 hand trails (which are drawn by HandTrail objects).

 Based on OpenNI's HandTracker and NITE's PointViewer examples.
 */

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.PrintWriter;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.text.DecimalFormat;
import java.util.HashMap;

import javax.swing.JFrame;
import javax.swing.JPanel;

import org.OpenNI.Context;
import org.OpenNI.DepthGenerator;
import org.OpenNI.GeneralException;
import org.OpenNI.GestureGenerator;
import org.OpenNI.HandsGenerator;
import org.OpenNI.IObservable;
import org.OpenNI.IObserver;
import org.OpenNI.ImageGenerator;
import org.OpenNI.ImageMetaData;
import org.OpenNI.License;
import org.OpenNI.MapOutputMode;
import org.OpenNI.PixelFormat;
import org.OpenNI.StatusException;

import com.primesense.NITE.DirectionVelocityAngleEventArgs;
import com.primesense.NITE.HandEventArgs;
import com.primesense.NITE.HandPointContext;
import com.primesense.NITE.IdEventArgs;
import com.primesense.NITE.NullEventArgs;
import com.primesense.NITE.PointControl;
import com.primesense.NITE.PointEventArgs;
import com.primesense.NITE.PushDetector;
import com.primesense.NITE.SessionManager;
import com.primesense.NITE.SwipeDetector;
import com.primesense.NITE.VelocityAngleEventArgs;
import com.primesense.NITE.WaveDetector;

enum SessionState {
    IN_SESSION, NOT_IN_SESSION, QUICK_REFOCUS
}

public class TrackerPanel extends JPanel implements Runnable {
    private static final int MAX_DEPTH_SIZE = 10000;

    // image vars
    private BufferedImage image = null;
    private int imWidth, imHeight;

    private volatile boolean isRunning;

    // used for the average ms processing information
    private int imageCount = 0;
    private long totalTime = 0;
    private DecimalFormat df;
    private Font msgFont;

    // OpenNI and NITE vars
    private Context context;
    private ImageGenerator imageGen;
    private DepthGenerator depthGen;

    private SessionManager sessionMan;
    private SessionState sessionState;

    private HashMap<Integer, HandTrail> handTrails;

    // for storing multiple hand trails

    public TrackerPanel(JFrame top) {
	setBackground(Color.DARK_GRAY);

	df = new DecimalFormat("0.#"); // 1 dp
	msgFont = new Font("SansSerif", Font.BOLD, 18);

	configKinect();
	handTrails = new HashMap<Integer, HandTrail>();

	new Thread(this).start(); // start updating the panel's image
    } // end of TrackerPanel()

    private void configKinect()
    // set up OpenNI and NITE generators and listerners
    {
	try {
	    context = new Context();

	    // add the NITE Licence
	    License licence = new License("PrimeSense", "0KOIk2JeIBYClPWVnMoRKn5cdY4=");
	    context.addLicense(licence);

	    // set up image and depth generators
	    imageGen = ImageGenerator.create(context);
	    // for displaying the scene
	    depthGen = DepthGenerator.create(context);
	    // for converting real-world coords to screen coords in HandTrail
	    // class

	    MapOutputMode mapMode = new MapOutputMode(640, 480, 30); // xRes,
								     // yRes,
								     // FPS
	    imageGen.setMapOutputMode(mapMode);
	    depthGen.setMapOutputMode(mapMode);

	    imageGen.setPixelFormat(PixelFormat.RGB24);

	    ImageMetaData imageMD = imageGen.getMetaData();
	    imWidth = imageMD.getFullXRes();
	    imHeight = imageMD.getFullYRes();
	    System.out.println("Image dimensions (" + imWidth + ", " + imHeight + ")");

	    // set Mirror mode for all
	    context.setGlobalMirror(true);

	    // set up hands and gesture generators
	    HandsGenerator hands = HandsGenerator.create(context);
	    hands.SetSmoothing(0.1f);

	    GestureGenerator gesture = GestureGenerator.create(context);

	    context.startGeneratingAll();
	    System.out.println("Started context generating...");

	    // set up session manager and points listener
	    sessionMan = new SessionManager(context, "Click,Wave", "RaiseHand");
	    // raise isn't recognized when click is included in focus ??
	    setSessionEvents(sessionMan);
	    sessionState = SessionState.NOT_IN_SESSION;

	    WaveDetector wd = initWaveDetector();
	    sessionMan.addListener(wd);

	    PointControl pointCtrl = initPointControl();
	    sessionMan.addListener(pointCtrl);

	    PushDetector pd = initPushDetector();
	    sessionMan.addListener(pd);

	    SwipeDetector sd = initSwipeDetector();
	    sessionMan.addListener(sd);
	} catch (GeneralException e) {
	    e.printStackTrace();
	    System.exit(1);
	}
    } // end of configKinect()

    private void setSessionEvents(SessionManager sessionMan)
    // create session callbacks
    {
	try {
	    // session start (S1)
	    sessionMan.getSessionStartEvent().addObserver(new IObserver<PointEventArgs>() {
		public void update(IObservable<PointEventArgs> observable, PointEventArgs args) {
		    System.out.println("Session started...");
		    sessionState = SessionState.IN_SESSION;
		}
	    });

	    // session end (S2)
	    sessionMan.getSessionEndEvent().addObserver(new IObserver<NullEventArgs>() {
		public void update(IObservable<NullEventArgs> observable, NullEventArgs args) {
		    System.out.println("Session ended");
		    isRunning = false;
		    sessionState = SessionState.NOT_IN_SESSION;
		}
	    });
	} catch (StatusException e) {
	    e.printStackTrace();
	}
    } // end of setSessionEvents()

    private PushDetector initPushDetector() {
	PushDetector pushDetector = null;
	try {
	    pushDetector = new PushDetector();

	    // some push settings; change with set
	    float minVel = pushDetector.getPushImmediateMinimumVelocity();
	    // minimum velocity in the time span to define as push, in m/s

	    float duration = pushDetector.getPushImmediateDuration();
	    // time used to detect push, in ms

	    float angleZ = pushDetector.getPushMaximumAngleBetweenImmediateAndZ();
	    // max angle between immediate direction and Z-axis, in degrees

	    System.out.printf("Push settings -- min velocity: %.1f m/s; min duration: %.1f ms; max angle to z-axis: %.1f degs \n", minVel, duration,
		    angleZ);

	    // callback
	    pushDetector.getPushEvent().addObserver(new IObserver<VelocityAngleEventArgs>() {
		public void update(IObservable<VelocityAngleEventArgs> observable, VelocityAngleEventArgs args) {
		    System.out.printf("Push: velocity %.1f m/s, angle %.1f degs \n", args.getVelocity(), args.getAngle());
		    // System.out.println("  " + pi); // show current hand point

		    sendPunchToServer();
		}
	    });
	} catch (GeneralException e) {
	    e.printStackTrace();
	}
	return pushDetector;
    } // end of initPushDetector()

    private SwipeDetector initSwipeDetector() {
	SwipeDetector swipeDetector = null;
	try {
	    swipeDetector = new SwipeDetector();

	    // some swipe settings; change with set
	    System.out.println("Swipe setting -- min motion time: " + swipeDetector.getMotionTime() + " ms");

	    // general swipe callback
	    swipeDetector.getGeneralSwipeEvent().addObserver(new IObserver<DirectionVelocityAngleEventArgs>() {
		public void update(IObservable<DirectionVelocityAngleEventArgs> observable, DirectionVelocityAngleEventArgs args) {
		    System.out.printf("Swipe %s: velocity %.1f m/s, angle %.1f degs \n", args.getDirection(), args.getVelocity(), args.getAngle());
		    // System.out.println("  " + pi); // show current hand point
		}
	    });

	    // callback for left swipes only;
	    swipeDetector.getSwipeLeftEvent().addObserver(new IObserver<VelocityAngleEventArgs>() {
		public void update(IObservable<VelocityAngleEventArgs> observable, VelocityAngleEventArgs args) {
		    System.out.printf("*Left* Swipe: velocity %.1f m/s, angle %.1f degs \n", args.getVelocity(), args.getAngle());
		}
	    });
	} catch (GeneralException e) {
	    e.printStackTrace();
	}
	return swipeDetector;
    } // end of initSwipeDetector()

    private WaveDetector initWaveDetector() {
	WaveDetector waveDetector = null;
	try {
	    waveDetector = new WaveDetector();

	    // some wave settings; change with set
	    int flipCount = waveDetector.getFlipCount();
	    int flipLen = waveDetector.getMinLength();
	    System.out.println("Wave settings -- no. of flips: " + flipCount + "; min length: " + flipLen + "mm");

	    // callback
	    waveDetector.getWaveEvent().addObserver(new IObserver<NullEventArgs>() {
		public void update(IObservable<NullEventArgs> observable, NullEventArgs args) {
		    System.out.println("Wave detected");
		    // System.out.println("  " + pi); // show current hand point
		}
	    });
	} catch (GeneralException e) {
	    e.printStackTrace();
	}
	return waveDetector;
    } // end of initWaveDetector()

    private PointControl initPointControl()
    // create 4 hand point listeners
    {
	PointControl pointCtrl = null;
	try {
	    pointCtrl = new PointControl();

	    // create new hand point, and hand trail (P1)
	    pointCtrl.getPointCreateEvent().addObserver(new IObserver<HandEventArgs>() {
		public void update(IObservable<HandEventArgs> observable, HandEventArgs args) {
		    sessionState = SessionState.IN_SESSION;
		    HandPointContext handContext = args.getHand();
		    int id = handContext.getID();
		    System.out.println("  Creating hand trail " + id);
		    HandTrail handTrail = new HandTrail(id, depthGen);
		    handTrail.addPoint(handContext.getPosition());
		    handTrails.put(id, handTrail);
		}
	    });

	    // hand point has moved; add to its trail (P2)
	    pointCtrl.getPointUpdateEvent().addObserver(new IObserver<HandEventArgs>() {
		public void update(IObservable<HandEventArgs> observable, HandEventArgs args) {
		    sessionState = SessionState.IN_SESSION;
		    HandPointContext handContext = args.getHand();
		    int id = handContext.getID();
		    // System.out.println("  Extending hand trail " + id);
		    HandTrail handTrail = handTrails.get(id);
		    handTrail.addPoint(handContext.getPosition());
		}
	    });

	    // destroy hand point and its trail (P3)
	    pointCtrl.getPointDestroyEvent().addObserver(new IObserver<IdEventArgs>() {
		public void update(IObservable<IdEventArgs> observable, IdEventArgs args) {
		    int id = args.getId();
		    System.out.println("  Deleting hand trail " + id);
		    handTrails.remove(id);
		    if (handTrails.isEmpty())
			System.out.println("  No hand trails left...");
		}
	    });

	    // no active hand point, which triggers refocusing (P4)
	    pointCtrl.getNoPointsEvent().addObserver(new IObserver<NullEventArgs>() {
		public void update(IObservable<NullEventArgs> observable, NullEventArgs args) {
		    if (sessionState != SessionState.NOT_IN_SESSION) {
			System.out.println("  Lost hand point, so refocusing");
			sessionState = SessionState.QUICK_REFOCUS;
		    }
		}
	    });

	} catch (GeneralException e) {
	    e.printStackTrace();
	}
	return pointCtrl;
    } // end of initPointControl()

    public Dimension getPreferredSize() {
	return new Dimension(imWidth, imHeight);
    }

    public void closeDown() {
	isRunning = false;
    }

    public void run()
    /*
     * update the Kinect info and redraw
     */
    {
	isRunning = true;
	while (isRunning) {
	    try {
		context.waitAnyUpdateAll();
		sessionMan.update(context);
	    } catch (StatusException e) {
		System.out.println(e);
		System.exit(1);
	    }
	    long startTime = System.currentTimeMillis();
	    updateCameraImage();
	    totalTime += (System.currentTimeMillis() - startTime);
	    repaint();
	}

	// close down
	try {
	    context.stopGeneratingAll();
	} catch (StatusException e) {
	}
	context.release();
	System.exit(1);
    } // end of run()

    private void updateCameraImage()
    // update Kinect camera's image
    {
	try {
	    ByteBuffer imageBB = imageGen.getImageMap().createByteBuffer();
	    image = bufToImage(imageBB);
	    imageCount++;
	} catch (GeneralException e) {
	    System.out.println(e);
	}
    } // end of updateCameraImage()

    private BufferedImage bufToImage(ByteBuffer pixelsRGB)
    /*
     * Transform the ByteBuffer of pixel data into a BufferedImage Converts RGB
     * bytes to ARGB ints with no transparency.
     */
    {
	int[] pixelInts = new int[imWidth * imHeight];

	int rowStart = 0;
	// rowStart will index the first byte (red) in each row;
	// starts with first row, and moves down

	int bbIdx; // index into ByteBuffer
	int i = 0; // index into pixels int[]
	int rowLen = imWidth * 3; // number of bytes in each row
	for (int row = 0; row < imHeight; row++) {
	    bbIdx = rowStart;
	    // System.out.println("bbIdx: " + bbIdx);
	    for (int col = 0; col < imWidth; col++) {
		int pixR = pixelsRGB.get(bbIdx++);
		int pixG = pixelsRGB.get(bbIdx++);
		int pixB = pixelsRGB.get(bbIdx++);
		pixelInts[i++] = 0xFF000000 | ((pixR & 0xFF) << 16) | ((pixG & 0xFF) << 8) | (pixB & 0xFF);
	    }
	    rowStart += rowLen; // move to next row
	}

	// create a BufferedImage from the pixel data
	BufferedImage im = new BufferedImage(imWidth, imHeight, BufferedImage.TYPE_INT_ARGB);
	im.setRGB(0, 0, imWidth, imHeight, pixelInts, 0, imWidth);
	return im;
    } // end of bufToImage()

    // -------------------- drawing ---------------------------------

    public void paintComponent(Graphics g)
    /*
     * Draw the camera image, hand trails, user instructions and statistics.
     */
    {
	super.paintComponent(g);
	Graphics2D g2 = (Graphics2D) g;

	if (image != null)
	    g2.drawImage(image, 0, 0, this); // draw camera's image

	drawTrails(g2);
	writeMessage(g2);

	writeStats(g2);
    } // end of paintComponent()

    private void drawTrails(Graphics2D g2) {
	HandTrail handTrail;
	for (Integer id : handTrails.keySet()) {
	    handTrail = handTrails.get(id);
	    handTrail.draw(g2);
	}
    } // end of drawTrails()

    private void writeMessage(Graphics2D g2)
    // draw user information based on the session state
    {
	g2.setColor(Color.YELLOW);
	g2.setFont(msgFont);

	String msg = null;
	switch (sessionState) {
	case IN_SESSION:
	    if (handTrails.size() == 1)
		msg = "Bring your second hand close to your first to track it";
	    else
		msg = "Tracking " + handTrails.size() + " hands...";
	    break;
	case NOT_IN_SESSION:
	    msg = "Click/Wave to start tracking";
	    break;
	case QUICK_REFOCUS:
	    msg = "Click/Wave/Raise your hand to resume tracking";
	    break;
	}
	if (msg != null)
	    g2.drawString(msg, 5, 20); // top left
    } // end of writeMessage()

    private void writeStats(Graphics2D g2)
    /*
     * write statistics in bottom-left corner, or "Loading" at start time
     */
    {
	g2.setColor(Color.YELLOW);
	g2.setFont(msgFont);
	int panelHeight = getHeight();
	if (imageCount > 0) {
	    double avgGrabTime = (double) totalTime / imageCount;
	    g2.drawString("Pic " + imageCount + "  " + df.format(avgGrabTime) + " ms", 5, panelHeight - 10); // bottom
													     // left
	} else
	    // no image yet
	    g2.drawString("Loading...", 5, panelHeight - 10);
    } // end of writeStats()

    private void sendPunchToServer() {
	try {
	    Socket skt = new Socket("192.168.100.48", 9999);
	    PrintWriter out = new PrintWriter(skt.getOutputStream(), true);
	    out.print("PUNCH");
	    out.close();
	    skt.close();
	} catch (Exception e) {
	    System.out.print("Whoops! It didn't work!\n");
	}
    }
} // end of TrackerPanel class

