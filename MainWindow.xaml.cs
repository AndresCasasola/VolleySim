//------------------------------------------------------------------------------
// Project: VolleySim (Volleyball simulator)
// Authors: Andrés Casasola Domínguez
//          Pedro Rico Pinazo
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    using System;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using System.Collections.Generic;
    using Microsoft.Speech.AudioFormat;
    using Microsoft.Speech.Recognition;
    using System.Windows.Documents;
    using System.ComponentModel;
    using System.Globalization;

    public struct Ball
    {
        public Point pos;         // Ball position
        public Vector vel;        // Ball velocity
        public bool onScreen;     // If it is or not onScreen
        public bool onNet;        // If ball is touching the net
    }

    public enum FieldSide {None, Right, Left};

    public struct Racket
    {
        public Point pos;              // Racket position
        public List<Point> posList;    // Racket old tracked positions
        public float theta;            // Racket theta angle
        public List<float> thetaList;  // Racket old tracked angles
        public bool onScreen;          // If it is or not on screen
        public FieldSide side;         // Field side where the racket should be
        public bool onItsSide;         // If the racket is currently in its side
    }

    /// Interaction logic for MainWindow.xaml
    public partial class MainWindow : Window
    {
        private KinectSensor sensor; // Active Kinect sensor
        private const float RenderWidth = 640.0f; // Width of output drawing
        private const float RenderHeight = 480.0f; // Height of our output drawing

        // Skeleton
        private const double JointThickness = 3; // Thickness of drawn joint lines
        private const double BodyCenterThickness = 10; // Thickness of body center ellipse
        private const double ClipBoundsThickness = 10; // Thickness of clip edge rectangles
        private readonly Brush centerPointBrush = Brushes.Blue; // Brush used to draw skeleton center point
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68)); // Brush used for drawing joints that are currently tracked    
        private readonly Brush inferredJointBrush = Brushes.Yellow; // Brush used for drawing joints that are currently inferred
        private readonly Pen trackedBonePen = new Pen(Brushes.Green, 6); // Pen used for drawing bones that are currently tracked
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1); // Pen used for drawing bones that are currently inferred
        private DrawingGroup drawingGroupSkeleton; // Drawing group for skeleton rendering output
        private DrawingImage imageSourceSkeleton; // Drawing image that we will display

        // FIR
        private readonly int FIR_SIZE = 3;
        private readonly double[] fir = { 0.5, 0.3, 0.2 };

        // Racket
        private Racket racket1;
        private Racket racket2;
        private readonly float racketSize = 140; // Racket size from on end to the other
        private double racketWidth = 6.0;

        // Ball
        private Ball ball;
        private readonly double gravity = 0.4; // Gravity of the ball
        readonly double initialVel = 5; // Initial velocity of the ball
        readonly Brush ballBrush = Brushes.White; // Brush used to draw ball
        float ballSize = 20;
        private DrawingGroup drawingGroupBall; // Drawing group for skeleton rendering output
        private DrawingImage imageSourceBall; // Drawing image that we will display

        // Net
        private readonly float netHeight = RenderHeight * 0.5f;
        private readonly float netX = RenderWidth * 0.5f;
        private double netWidth = 6.0;
        private Brush netBrush = Brushes.White;

        // Game
        private int scoreR, scoreL;
        private bool gameStarted;
        private bool twoPlayersOnScreen;

        private double penThickness = 2.0;
        private Brush penBrush = Brushes.Black;

        // RGB
        private WriteableBitmap colorBitmap; // Bitmap that will hold color information
        private byte[] colorPixels; // Intermediate storage for the color data received from the camera

        /// Speech recognition engine using audio data from Kinect.
        private SpeechRecognitionEngine speechEngine;

        /// Initializes a new instance of the MainWindow class.
        public MainWindow()
        {
            InitializeComponent();
        }

        // Execute startup tasks
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            // Look through all sensors and start the first connected one.
            // This requires that a Kinect is connected at the time of app startup.
            // To make your app robust against plug/unplug, 
            // it is recommended to use KinectSensorChooser provided in Microsoft.Kinect.Toolkit (See components in Toolkit Browser).
            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.sensor = potentialSensor;
                    break;
                }
            }

            if (null != this.sensor)
            {
                // Skeleton
                // Create the drawing group we'll use for drawing
                this.drawingGroupSkeleton = new DrawingGroup();
                // Create an image source that we can use in our image control
                this.imageSourceSkeleton = new DrawingImage(this.drawingGroupSkeleton);
                // Display the drawing using our image control
                ImageSkeleton.Source = this.imageSourceSkeleton;
                // Turn on the skeleton stream to receive skeleton frames
                this.sensor.SkeletonStream.Enable();
                // Add an event handler to be called whenever there is new color frame data
                this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;

                // Ball
                // Create the drawing group we'll use for drawing
                this.drawingGroupBall = new DrawingGroup();
                // Create an image source that we can use in our image control
                this.imageSourceBall = new DrawingImage(this.drawingGroupBall);
                // Display the drawing using our image control
                ImageBall.Source = this.imageSourceBall;
                // Turn on the skeleton stream to receive skeleton frames
                ball.vel = new Vector(0, initialVel);
                ball.pos = new Point();

                // Racket
                racket1.posList = new List<Point>();
                racket1.thetaList = new List<float>();
                racket2.posList = new List<Point>();
                racket2.thetaList = new List<float>();

                // RGB
                // Turn on the color stream to receive color frames
                this.sensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
                // Allocate space to put the pixels we'll receive
                this.colorPixels = new byte[this.sensor.ColorStream.FramePixelDataLength];
                // This is the bitmap we'll display on-screen
                this.colorBitmap = new WriteableBitmap(this.sensor.ColorStream.FrameWidth, this.sensor.ColorStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);
                // Set the image we display to point to the bitmap where we'll put the image data
                this.ImageRGB.Source = this.colorBitmap;
                // Add an event handler to be called whenever there is new color frame data
                this.sensor.ColorFrameReady += this.SensorColorFrameReady;

                // Start the sensor!
                try
                {
                    this.sensor.Start();
                }
                catch (IOException)
                {
                    this.sensor = null;
                }
            }

            if (null == this.sensor)
            {
                this.statusBarText.Text = Properties.Resources.NoKinectReady;
            }

            // v----- Speech recognizer -----v //
            RecognizerInfo ri = GetKinectRecognizer();

            if (null != ri)
            {

                this.speechEngine = new SpeechRecognitionEngine(ri.Id);
 
                var directions = new Choices();
                directions.Add(new SemanticResultValue("pelota", "PELOTA"));   // Grammar, Command
                directions.Add(new SemanticResultValue("reset", "RESET"));

                var gb = new GrammarBuilder { Culture = ri.Culture };
                gb.Append(directions);
                
                var g = new Grammar(gb);
                 
                // Create a grammar not from grammar definition XML file.
                speechEngine.LoadGrammar(g);

                speechEngine.SpeechRecognized += SpeechRecognized;
                //speechEngine.SpeechRecognitionRejected += SpeechRejected;

                // For long recognition sessions (a few hours or more), it may be beneficial to turn off adaptation of the acoustic model. 
                // This will prevent recognition accuracy from degrading over time.
                ////speechEngine.UpdateRecognizerSetting("AdaptationOn", 0);

                speechEngine.SetInputToAudioStream(
                sensor.AudioSource.Start(), new SpeechAudioFormatInfo(EncodingFormat.Pcm, 16000, 16, 1, 32000, 2, null));
                speechEngine.RecognizeAsync(RecognizeMode.Multiple);
            }
            // ^----- Speech recognizer -----^ //
        }

        // Execute shutdown tasks
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor)
            {
                this.sensor.Stop();
            }

            // v----- Speech recognizer -----v //
            if (null != this.sensor)
            {
                this.sensor.AudioSource.Stop();

                this.sensor.Stop();
                this.sensor = null;
            }

            if (null != this.speechEngine)
            {
                this.speechEngine.SpeechRecognized -= SpeechRecognized;
                //this.speechEngine.SpeechRecognitionRejected -= SpeechRejected;
                this.speechEngine.RecognizeAsyncStop();
            }
            // ^----- Speech recognizer -----^ //
        }

        // Event handler for Kinect sensor's ColorFrameReady event
        private void SensorColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
            {
                if (colorFrame != null)
                {
                    // Copy the pixel data from the image to a temporary array
                    colorFrame.CopyPixelDataTo(this.colorPixels);

                    // Write the pixel data into our bitmap
                    this.colorBitmap.WritePixels(
                        new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight),
                        this.colorPixels,
                        this.colorBitmap.PixelWidth * sizeof(int),
                        0);
                }
            }

            DrawBall();
        }

        // Event handler for Kinect sensor's SkeletonFrameReady event
        private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons = new Skeleton[0];

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);
                    Console.WriteLine(skeletonFrame.SkeletonArrayLength);
                }
            }

            using (DrawingContext dc = this.drawingGroupSkeleton.Open())
            {
                bool onePlayerOnScreen = false;
                twoPlayersOnScreen = false;

                // Draw a transparent background to set the render size
                dc.DrawRectangle(Brushes.Transparent, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));

                foreach (Skeleton skel in skeletons)
                {
                    //RenderClippedEdges(skel, dc);

                    if (skel.TrackingState == SkeletonTrackingState.Tracked)
                    {
                        if (!onePlayerOnScreen) // Fisrt skeleton found
                        {
                            DrawRacket(skel, dc, ref racket1);
                            racket1.onScreen = true;
                            onePlayerOnScreen = true;
                        }
                        else // Second skeleton found
                        {
                            DrawRacket(skel, dc, ref racket2);
                            racket2.onScreen = true;
                            twoPlayersOnScreen = true;
                        }
                    }
                }

                // prevent drawing outside of our render area
                this.drawingGroupSkeleton.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }
        }

        // Handler for recognized speech events.
        private void SpeechRecognized(object sender, SpeechRecognizedEventArgs e)
        {
            // Speech utterance confidence below which we treat speech as if it hadn't been heard
            const double ConfidenceThreshold = 0.3;

            if (e.Result.Confidence >= ConfidenceThreshold)
            {
                switch (e.Result.Semantics.Value.ToString())
                {
                    case "PELOTA":
                        if(!gameStarted  &&  twoPlayersOnScreen)
                        {
                            if (racket1.side != FieldSide.None && racket2.side != FieldSide.None && racket1.side != racket2.side)
                                gameStarted = true;
                        }

                        if (gameStarted  &&  !ball.onScreen  &&  racket1.onItsSide  &&  racket2.onItsSide)
                        {
                            Random random = new Random();
                            int rand = random.Next(0, 2);
                            ball.pos.X = RenderWidth * 0.25 + (RenderWidth * 0.5 * rand);
                            ball.pos.Y = 0;
                            ball.vel.X = 0;
                            ball.vel.Y = initialVel;
                            ball.onScreen = true;
                            ball.onNet = false;
                        }
                        break;

                    case "RESET":
                        ball.onScreen = false;
                        scoreR = 0;
                        scoreL = 0;
                        textScoreL.Text = scoreL.ToString();
                        textScoreR.Text = scoreR.ToString();
                        gameStarted = false;
                    break;

                }
            }
        }

        // Handles the checking or unchecking of the seated mode combo box
        private void CheckBoxSeatedModeChanged(object sender, RoutedEventArgs e)
        {
            if (null != this.sensor)
            {
                if (this.checkBoxSeatedMode.IsChecked.GetValueOrDefault())
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
                else
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Default;
            }
        }

        // Update ball position, check collisions
        private void DrawBall()
        {
            using (DrawingContext dc = this.drawingGroupBall.Open())
            {
                // Draw a transparent background to set the render size
                dc.DrawRectangle(Brushes.Transparent, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));

                if (ball.onScreen)
                {
                    // Update position
                    ball.pos = ball.pos + ball.vel;
                    if (ball.pos.Y > RenderHeight + ballSize / 2.0) // If ball is out of screen
                    {
                        ball.onScreen = false;
                        if (ball.pos.X > netX)  // Right field side
                        {
                            scoreL++;  // Point for left player
                            textScoreL.Text = scoreL.ToString();
                        }
                        else  // Left field side
                        {
                            scoreR++;  // Point for right player
                            textScoreR.Text = scoreR.ToString();
                        }
                    }
                    else  // If ball is not out of screen
                        ball.vel.Y += gravity;
                }

                // Screen edges collision
                if (ball.pos.Y < -ballSize / 2)  // Out of top side
                {
                    ball.vel.Y = -ball.vel.Y;
                    ball.pos.Y = -ballSize - ball.pos.Y;
                }
                if (ball.pos.X < -ballSize / 2)  // Out of left side
                {
                    ball.vel.X = -ball.vel.X;
                    ball.pos.X = -ballSize - ball.pos.X;
                }
                if (ball.pos.X > RenderWidth + ballSize / 2)  // Out of right side
                {
                    ball.vel.X = -ball.vel.X;
                    ball.pos.X = 2 * RenderWidth + ballSize - ball.pos.X;
                }

                if (!ball.onNet) // If ball is on net, rackets cannot touch it
                {
                    // Racket 1 collision
                    if (racket1.onScreen)
                    {
                        ball.vel = ToRacketReferenceFrame(ball.vel, racket1.pos, racket1.theta);
                        ball.pos = ToRacketReferenceFrame(ball.pos, racket1.pos, racket1.theta);
                        if (ball.pos.X < ballSize / 2 && ball.pos.X > -ballSize / 2 && ball.pos.Y < racketSize / 2 && ball.pos.Y > -racketSize / 2)
                        {
                            // Reflects Position
                            ball.pos.X = -ball.pos.X - ballSize * Math.Sign(ball.vel.X);
                            // Reflects velocity
                            ball.vel.X = -ball.vel.X;
                        }
                        ball.vel = ToScreenReferenceFrame(ball.vel, racket1.pos, racket1.theta);
                        ball.pos = ToScreenReferenceFrame(ball.pos, racket1.pos, racket1.theta);
                    }

                    // Racket 2 collision
                    if (racket2.onScreen)
                    {
                        ball.vel = ToRacketReferenceFrame(ball.vel, racket2.pos, racket2.theta);
                        ball.pos = ToRacketReferenceFrame(ball.pos, racket2.pos, racket2.theta);
                        if (ball.pos.X < ballSize / 2 && ball.pos.X > -ballSize / 2 && ball.pos.Y < racketSize / 2 && ball.pos.Y > -racketSize / 2)
                        {
                            // Reflects Position
                            ball.pos.X = -ball.pos.X - ballSize * Math.Sign(ball.vel.X);
                            // Reflects velocity
                            ball.vel.X = -ball.vel.X;
                        }
                        ball.vel = ToScreenReferenceFrame(ball.vel, racket2.pos, racket2.theta);
                        ball.pos = ToScreenReferenceFrame(ball.pos, racket2.pos, racket2.theta);
                    }

                    // Net collision
                    if (ball.pos.Y > netHeight && ball.pos.X > netX - ballSize / 2 && ball.pos.X < netX + ballSize / 2)
                    {
                        if (ball.vel.X > 0) // It comes from the left side
                            ball.pos.X = netX - ballSize / 2;
                        else  // It comes from the right side
                            ball.pos.X = netX + ballSize / 2;

                        ball.vel.X = 0;
                        ball.vel.Y = 0;
                        ball.onNet = true;
                    }
                }

                // Draw Ball
                if (ball.onScreen)
                    dc.DrawEllipse(ballBrush, new Pen(Brushes.Black, penThickness), ball.pos, ballSize, ballSize);

                // Draw Net
                dc.DrawRectangle(netBrush, new Pen(penBrush, penThickness), new Rect(netX - netWidth / 2, netHeight, netWidth, RenderHeight));
            }
            this.drawingGroupBall.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
        }

        private void DrawRacket(Skeleton skeleton, DrawingContext drawingContext, ref Racket racket)
        {
            FieldSide side;
            Brush racketBrush;
            Point rightEnd = new Point(0, racketSize / 2);
            Point leftEnd = new Point(0, -racketSize / 2);
            Point newPoint;
            float newTheta;

            Point handR;
            Point handL;
            Joint handJointR;
            Joint handJointL;

            handJointR = skeleton.Joints[JointType.HandRight];
            handR = SkeletonPointToScreen(handJointR.Position);
            handJointL = skeleton.Joints[JointType.HandLeft];
            handL = SkeletonPointToScreen(handJointL.Position);

            newPoint = new Point((handR.X + handL.X) / 2, (handR.Y + handL.Y) / 2);
            newTheta = (float)Math.Acos((handR.Y - handL.Y) / Point.Subtract(handR, handL).Length);
            if (handR.X > handL.X)
                newTheta = -newTheta;


            // Insert new theta and filter it
            racket.thetaList.Insert(0, newTheta);
            if (racket.thetaList.Count > FIR_SIZE) // If there is enough elements for the filter
            {
                racket.thetaList.RemoveAt(FIR_SIZE);
                racket.theta = 0;
                for (int k = 0; k < FIR_SIZE; ++k)
                    racket.theta += racket.thetaList[k] * (float)fir[k];
            }
            else
            {
                racket.theta = racket.thetaList[0];
            }


            // Insert new position and filter it
            racket.posList.Insert(0, newPoint);
            if (racket.posList.Count > FIR_SIZE) // If there is enough elements for the filter
            {
                racket.posList.RemoveAt(FIR_SIZE);
                racket.pos.X = 0;
                racket.pos.Y = 0;
                for (int k = 0; k < FIR_SIZE; ++k)
                {
                    racket.pos.X += racket.posList[k].X * fir[k];
                    racket.pos.Y += racket.posList[k].Y * fir[k];
                }
            }
            else
            {
                racket.pos = racket.posList[0];
            }

            // Obtain the ends of the racket
            rightEnd = ToScreenReferenceFrame(rightEnd, racket.pos, racket.theta);
            leftEnd = ToScreenReferenceFrame(leftEnd, racket.pos, racket.theta);

            // Check which field side is the racket in
            if (rightEnd.X > RenderWidth / 2 && leftEnd.X > RenderWidth / 2)
                side = FieldSide.Right;
            else if (rightEnd.X < RenderWidth / 2 && leftEnd.X < RenderWidth / 2)
                side = FieldSide.Left;
            else
                side = FieldSide.None;

            if (!gameStarted) // When game is not started, players can change the field side
            {
                racket.side = side;
            }
            else if (racket.side != side) // When game is started, if they don't remain on the same side
            {
                racket.onItsSide = false;
                if (ball.onScreen) // And the ball is on the screen
                {
                    // The other player gets one point
                    if (racket.side == FieldSide.Right)
                    {
                        ++scoreL;
                        textScoreL.Text = scoreL.ToString();
                    }
                    else
                    {
                        ++scoreR;
                        textScoreR.Text = scoreR.ToString();
                    }
                }
                ball.onScreen = false;
            }
            else
            {
                racket.onItsSide = true;
            }

            // Select the racket color depending on the field side
            switch (racket.side)
            {
                case FieldSide.Right:
                    racketBrush = Brushes.Blue;
                    break;
                case FieldSide.Left:
                    racketBrush = Brushes.Red;
                    break;
                default:
                    racketBrush = Brushes.Black;
                    break;
            }

            // Draw racket
            drawingContext.DrawLine(new Pen(racketBrush, 5), rightEnd, leftEnd);
        }

        // Translate point from racket reference frame to screen reference frame 
        private Point ToScreenReferenceFrame(Point point, Point racketPos, float theta)
        {
            Matrix mat = new Matrix(Math.Cos(theta), Math.Sin(theta),
                                                      -Math.Sin(theta), Math.Cos(theta),
                                                      racketPos.X, racketPos.Y);
            return mat.Transform(point);
        }

        // Translate vector from racket reference frame to screen reference frame
        private Vector ToScreenReferenceFrame(Vector vector, Point racketPos, float theta)
        {
            Point point;

            Matrix mat = new Matrix(Math.Cos(theta), Math.Sin(theta),
                                                      -Math.Sin(theta), Math.Cos(theta),
                                                      0, 0);
            point = new Point(vector.X, vector.Y);
            point = mat.Transform(point);
            vector.X = point.X;
            vector.Y = point.Y;
            return vector;
        }

        // Translate point from screen reference frame to racket reference frame
        private Point ToRacketReferenceFrame(Point point, Point racketPos, float theta)
        {
            Matrix mat = new Matrix(Math.Cos(theta), Math.Sin(theta),
                                                      -Math.Sin(theta), Math.Cos(theta),
                                                      racketPos.X, racketPos.Y);
            mat.Invert();
            return mat.Transform(point);
        }
        
        // Translate vector from screen reference frame to racket reference frame
        private Vector ToRacketReferenceFrame(Vector vector, Point racketPos, float theta)
        {
            Point point;

            Matrix mat = new Matrix(Math.Cos(theta), Math.Sin(theta),
                                                      -Math.Sin(theta), Math.Cos(theta),
                                                      0, 0);
            mat.Invert();
            point = new Point(vector.X, vector.Y);
            point = mat.Transform(point);
            vector.X = point.X;
            vector.Y = point.Y;
            return vector;
        }

        // Maps a SkeletonPoint to lie within our render space and converts to Point
        private Point SkeletonPointToScreen(SkeletonPoint skelpoint)
        {
            // Convert point to depth space.  
            // We are not using depth directly, but we do want the points in our 640x480 output resolution.
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            return new Point(depthPoint.X, depthPoint.Y);
        }

        /// Gets the metadata for the speech recognizer (acoustic model) most suitable to
        /// process audio from Kinect device.
        private static RecognizerInfo GetKinectRecognizer()
        {
            foreach (RecognizerInfo recognizer in SpeechRecognitionEngine.InstalledRecognizers())
            {
                string value;
                recognizer.AdditionalInfo.TryGetValue("Kinect", out value);
                if ("True".Equals(value, StringComparison.OrdinalIgnoreCase) && "es-ES".Equals(recognizer.Culture.Name, StringComparison.OrdinalIgnoreCase))
                {
                    return recognizer;
                }
            }

            return null;
        }
    }
}


