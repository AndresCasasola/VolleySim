//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
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
    // Libraries for speech recognition:
    using Microsoft.Speech.AudioFormat;
    using Microsoft.Speech.Recognition;
    using System.Windows.Documents;

    using System.ComponentModel;
    using System.Globalization;

    public struct Ball
    {
        public Point pos; // Ball position
        public Vector vel; // Ball velocity
        public bool onScreen; // If it is or not onScreen
        public bool onNet; // If ball is touching the net
    }

    public struct Racket
    {
        public Point pos; //
        public List<Point> posList; // Racket old tracked positions
        public float theta; // Angle from x axis following the right hand rule
        public List<float> thetaList; 
        public bool onScreen; // If it is or not on screen
    }

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
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
        private readonly Brush racketBrush = Brushes.Blue; // Brush used to draw ball

        // Ball
        private Ball ball;
        private readonly double gravity = 0.4; // Gravity of the ball
        readonly double initialVel = 5; // Initial velocity of the ball
        readonly Brush ballBrush = Brushes.Blue; // Brush used to draw ball
        float ballSize = 20;
        private DrawingGroup drawingGroupBall; // Drawing group for skeleton rendering output
        private DrawingImage imageSourceBall; // Drawing image that we will display

        // Net
        private readonly float netHeight = RenderHeight * 0.5f;
        private readonly float netX = RenderWidth * 0.5f;

        // Scores
        private int scoreR, scoreL;

        // RGB
        private WriteableBitmap colorBitmap; // Bitmap that will hold color information
        private byte[] colorPixels; // Intermediate storage for the color data received from the camera

        /// Speech recognition engine using audio data from Kinect.
        private SpeechRecognitionEngine speechEngine;

        /// List of all UI span elements used to select recognized text. Delete if works commented
        //private List<Span> recognitionSpans;


        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();
        }

        /// Gets the metadata for the speech recognizer (acoustic model) most suitable to
        /// process audio from Kinect device.
        /// RecognizerInfo if found, <code>null</code> otherwise.
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

        /// <summary>
        /// Draws indicators to show which edges are clipping skeleton data
        /// </summary>
        /// <param name="skeleton">skeleton to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private static void RenderClippedEdges(Skeleton skeleton, DrawingContext drawingContext)
        {
            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, RenderHeight - ClipBoundsThickness, RenderWidth, ClipBoundsThickness));
            }
            
            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, RenderHeight));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(RenderWidth - ClipBoundsThickness, 0, ClipBoundsThickness, RenderHeight));
            }
        }

        /// <summary>
        /// Execute startup tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
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
                speechEngine.SpeechRecognitionRejected += SpeechRejected;

                // For long recognition sessions (a few hours or more), it may be beneficial to turn off adaptation of the acoustic model. 
                // This will prevent recognition accuracy from degrading over time.
                ////speechEngine.UpdateRecognizerSetting("AdaptationOn", 0);

                speechEngine.SetInputToAudioStream(
                    sensor.AudioSource.Start(), new SpeechAudioFormatInfo(EncodingFormat.Pcm, 16000, 16, 1, 32000, 2, null));
                speechEngine.RecognizeAsync(RecognizeMode.Multiple);
            }
            // ^----- Speech recognizer -----^ //
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
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
                this.speechEngine.SpeechRecognitionRejected -= SpeechRejected;
                this.speechEngine.RecognizeAsyncStop();
            }
            // ^----- Speech recognizer -----^ //
        }

        /// <summary>
        /// Event handler for Kinect sensor's ColorFrameReady event
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
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

        private void DrawBall()
        {
            using (DrawingContext dc = this.drawingGroupBall.Open())
            {
                // Draw a transparent background to set the render size
                dc.DrawRectangle(Brushes.Transparent, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));

                if(ball.onScreen)
                {
                    ball.pos = ball.pos + ball.vel;
                    if (ball.pos.Y > RenderHeight + ballSize / 2.0) // If ball is out of screen
                    {
                        ball.onScreen = false;
                        if (ball.pos.X > netX)
                        {
                            scoreL++;
                            textScoreL.Text = scoreL.ToString();
                            //Format("{0}", scoreL);
                        }
                        else
                        {
                            scoreR++;
                            textScoreR.Text = scoreR.ToString();
                        }
                    }
                    else                                           // If not
                        ball.vel.Y += gravity;
                }

                // Screen edges collision
                if (ball.pos.Y < -ballSize / 2)  // Out of top side
                {
                    ball.vel.Y = -ball.vel.Y;
                    ball.pos.Y = - ballSize - ball.pos.Y;
                }
                if (ball.pos.X < -ballSize / 2 )  // Out of left side
                {
                    ball.vel.X = -ball.vel.X;
                    ball.pos.X = -ballSize - ball.pos.X;
                }
                if (ball.pos.X > RenderWidth + ballSize / 2)  // Out of right side
                {
                    ball.vel.X = -ball.vel.X;
                    ball.pos.X = 2* RenderWidth + ballSize - ball.pos.X;
                }

                if (!ball.onNet)
                {
                    // Racket 1 collision
                    if (racket1.onScreen)
                    {
                        ball.vel = ToRacketSystem(ball.vel, racket1.pos, racket1.theta); //racket1.pos[0]
                        ball.pos = ToRacketSystem(ball.pos, racket1.pos, racket1.theta);
                        if (ball.pos.X < ballSize / 2 && ball.pos.X > -ballSize / 2 && ball.pos.Y < racketSize / 2 && ball.pos.Y > -racketSize / 2)
                        {
                            ball.pos.X = -ball.pos.X - ballSize * Math.Sign(ball.vel.X); // Reflects Position
                            ball.vel.X = -ball.vel.X; // Reflects velocity
                        }
                        ball.vel = ToScreenSystem(ball.vel, racket1.pos, racket1.theta);
                        ball.pos = ToScreenSystem(ball.pos, racket1.pos, racket1.theta);
                    }

                    // Racket 2 collision
                    if (racket2.onScreen)
                    {
                        ball.vel = ToRacketSystem(ball.vel, racket2.pos, racket2.theta);
                        ball.pos = ToRacketSystem(ball.pos, racket2.pos, racket2.theta);
                        if (ball.pos.X < ballSize / 2 && ball.pos.X > -ballSize / 2 && ball.pos.Y < racketSize / 2 && ball.pos.Y > -racketSize / 2)
                        {
                            ball.pos.X = -ball.pos.X - ballSize * Math.Sign(ball.vel.X); // Reflects Position
                            ball.vel.X = -ball.vel.X; // Reflects velocity
                        }
                        ball.vel = ToScreenSystem(ball.vel, racket2.pos, racket2.theta);
                        ball.pos = ToScreenSystem(ball.pos, racket2.pos, racket2.theta);
                    }

                    // Net collision
                    if (ball.pos.Y > netHeight && ball.pos.X > netX - ballSize / 2 && ball.pos.X < netX + ballSize / 2)
                    {
                        if (ball.vel.X > 0) // It comes from the left side
                            ball.pos.X = netX - ballSize / 2;
                        else
                            ball.pos.X = netX + ballSize / 2;

                        ball.vel.X = 0;
                        ball.vel.Y = 0;
                        ball.onNet = true;
                    }
                }

                // Draw Ball
                if(ball.onScreen)
                    dc.DrawEllipse(ballBrush, null, ball.pos, ballSize, ballSize);

                // Draw Net
                dc.DrawLine(new Pen(ballBrush, 5), new Point(netX, RenderHeight), new Point(netX, netHeight));
            }
            this.drawingGroupBall.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
        }

        /// <summary>
        /// Event handler for Kinect sensor's SkeletonFrameReady event
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
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
                bool oneplayer = false;

                // Draw a transparent background to set the render size
                dc.DrawRectangle(Brushes.Transparent, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));



                foreach (Skeleton skel in skeletons)
                {
                    RenderClippedEdges(skel, dc);

                    if (skel.TrackingState == SkeletonTrackingState.Tracked)
                    {
                        //this.DrawBonesAndJoints(skel, dc);
                        if (!oneplayer)
                        {
                            DrawRacket(skel, dc, ref racket1);
                            racket1.onScreen = true;
                            oneplayer = true;
                        }
                        else
                        {
                            DrawRacket(skel, dc, ref racket2);
                            racket2.onScreen = true;
                        }
                    }
                    /*
                    else if (skel.TrackingState == SkeletonTrackingState.PositionOnly)
                    {
                        dc.DrawEllipse(
                        this.centerPointBrush,
                        null,
                        this.SkeletonPointToScreen(skel.Position),
                        BodyCenterThickness,
                        BodyCenterThickness);
                    }
                    */
                }

                // prevent drawing outside of our render area
                this.drawingGroupSkeleton.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }
        }

        /// Handler for recognized speech events.
        private void SpeechRecognized(object sender, SpeechRecognizedEventArgs e)
        {
            // Speech utterance confidence below which we treat speech as if it hadn't been heard
            const double ConfidenceThreshold = 0.3;

            // Number of degrees in a right angle.
            const int DegreesInRightAngle = 90;

            // Number of pixels turtle should move forwards or backwards each time.
            const int DisplacementAmount = 60;

            //ClearRecognitionHighlights();

            if (e.Result.Confidence >= ConfidenceThreshold)
            {
                switch (e.Result.Semantics.Value.ToString())
                {
                    case "PELOTA":
                        if (!ball.onScreen)
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
                    break;

                }
            }
        }

        private void DrawRacket(Skeleton skeleton, DrawingContext drawingContext, ref Racket racket)
        {
            Point rightEnd = new Point(0, racketSize / 2);
            Point leftEnd = new Point(0, -racketSize / 2);
            Point newPoint;
            float newTheta;


            /*
            Point wrist;
            Point elbow;
            Joint elbowJoint;
            Joint wristJoint;
            
            wristJoint = skeleton.Joints[JointType.WristRight];
            wrist = SkeletonPointToScreen(wristJoint.Position);

            if(wrist.X > RenderWidth / 2) // If right hand is on the right use it
            {
                elbowJoint = skeleton.Joints[JointType.ElbowRight];
            }
            else                         // If not, use the left hand
            {
                wristJoint = skeleton.Joints[JointType.WristLeft];
                wrist = SkeletonPointToScreen(wristJoint.Position);
                elbowJoint = skeleton.Joints[JointType.ElbowLeft];
            }
            elbow = SkeletonPointToScreen(elbowJoint.Position);
            newPoint = new Point(wrist.X, wrist.Y);

            newTheta = (float)Math.Acos((wrist.X - elbow.X) / Point.Subtract(wrist, elbow).Length);
            if (wrist.Y < elbow.Y)
                newTtheta = -newTheta;
            */

            Point handR;
            Point handL;
            Joint handJointR;
            Joint handJointL;

            handJointR = skeleton.Joints[JointType.HandRight];
            handR = SkeletonPointToScreen(handJointR.Position);
            handJointL = skeleton.Joints[JointType.HandLeft];
            handL = SkeletonPointToScreen(handJointL.Position);
            //drawingContext.DrawEllipse(racketBrush, null, handR, 10, 10);
            //drawingContext.DrawEllipse(racketBrush, null, handL, 10, 10);

            newPoint = new Point((handR.X + handL.X) / 2, (handR.Y + handL.Y) / 2);
            newTheta = (float)Math.Acos((handR.Y - handL.Y) / Point.Subtract(handR, handL).Length);
            if (handR.X > handL.X)
                newTheta = -newTheta;


            // Insert new theta and filter it
            racket.theta = newTheta;
            racket.thetaList.Insert(0, newTheta);
            if (racket.thetaList.Count > FIR_SIZE)
            {
                racket.thetaList.RemoveAt(FIR_SIZE);
                racket.theta = 0;
                for (int k = 0; k < FIR_SIZE; ++k)
                {
                    racket.theta += racket.thetaList[k] * (float)fir[k];
                }
            }
            else
            {
                racket.theta = racket.thetaList[0];
            }


            // Insert new position and filter it
            racket.posList.Insert(0, newPoint);
            if (racket.posList.Count > FIR_SIZE)
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


            rightEnd = ToScreenSystem(rightEnd, racket.pos, racket.theta);
            leftEnd = ToScreenSystem(leftEnd, racket.pos, racket.theta);

            drawingContext.DrawLine(new Pen(racketBrush, 5), rightEnd, leftEnd);
        }

        private Point ToScreenSystem(Point point, Point racketPos, float theta)
        {
            Matrix mat = new Matrix(Math.Cos(theta), Math.Sin(theta), 
                                                      -Math.Sin(theta), Math.Cos(theta), 
                                                      racketPos.X, racketPos.Y);
            return mat.Transform(point);
        }

        private Vector ToScreenSystem(Vector vector, Point racketPos, float theta)
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

        private Point ToRacketSystem(Point point, Point racketPos, float theta)
        {
            Matrix mat = new Matrix(Math.Cos(theta), Math.Sin(theta),
                                                      -Math.Sin(theta), Math.Cos(theta),
                                                      racketPos.X, racketPos.Y);
            mat.Invert();
            return mat.Transform(point);
        }

        private Vector ToRacketSystem(Vector vector, Point racketPos, float theta)
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

        /// <summary>
        /// Draws a skeleton's bones and joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawBonesAndJoints(Skeleton skeleton, DrawingContext drawingContext)
        {
            // Render Torso
            this.DrawBone(skeleton, drawingContext, JointType.Head, JointType.ShoulderCenter);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderRight);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.Spine);
            this.DrawBone(skeleton, drawingContext, JointType.Spine, JointType.HipCenter);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipLeft);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipRight);

            // Left Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderLeft, JointType.ElbowLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowLeft, JointType.WristLeft);
            this.DrawBone(skeleton, drawingContext, JointType.WristLeft, JointType.HandLeft);

            // Right Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderRight, JointType.ElbowRight);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowRight, JointType.WristRight);
            this.DrawBone(skeleton, drawingContext, JointType.WristRight, JointType.HandRight);

            // Left Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipLeft, JointType.KneeLeft);
            this.DrawBone(skeleton, drawingContext, JointType.KneeLeft, JointType.AnkleLeft);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleLeft, JointType.FootLeft);

            // Right Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipRight, JointType.KneeRight);
            this.DrawBone(skeleton, drawingContext, JointType.KneeRight, JointType.AnkleRight);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleRight, JointType.FootRight);
 
            // Render Joints
            foreach (Joint joint in skeleton.Joints)
            {
                Brush drawBrush = null;

                if (joint.TrackingState == JointTrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;                    
                }
                else if (joint.TrackingState == JointTrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;                    
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, this.SkeletonPointToScreen(joint.Position), JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Maps a SkeletonPoint to lie within our render space and converts to Point
        /// </summary>
        /// <param name="skelpoint">point to map</param>
        /// <returns>mapped point</returns>
        private Point SkeletonPointToScreen(SkeletonPoint skelpoint)
        {
            // Convert point to depth space.  
            // We are not using depth directly, but we do want the points in our 640x480 output resolution.
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            return new Point(depthPoint.X, depthPoint.Y);
        }

        /// <summary>
        /// Draws a bone line between two joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw bones from</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="jointType0">joint to start drawing from</param>
        /// <param name="jointType1">joint to end drawing at</param>
        private void DrawBone(Skeleton skeleton, DrawingContext drawingContext, JointType jointType0, JointType jointType1)
        {
            Joint joint0 = skeleton.Joints[jointType0];
            Joint joint1 = skeleton.Joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == JointTrackingState.NotTracked ||
                joint1.TrackingState == JointTrackingState.NotTracked)
            {
                return;
            }

            // Don't draw if both points are inferred
            if (joint0.TrackingState == JointTrackingState.Inferred &&
                joint1.TrackingState == JointTrackingState.Inferred)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if (joint0.TrackingState == JointTrackingState.Tracked && joint1.TrackingState == JointTrackingState.Tracked)
            {
                drawPen = this.trackedBonePen;
            }

            drawingContext.DrawLine(drawPen, this.SkeletonPointToScreen(joint0.Position), this.SkeletonPointToScreen(joint1.Position));
        }

        /// <summary>
        /// Handles the checking or unchecking of the seated mode combo box
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void CheckBoxSeatedModeChanged(object sender, RoutedEventArgs e)
        {
            if (null != this.sensor)
            {
                if (this.checkBoxSeatedMode.IsChecked.GetValueOrDefault())
                {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
                }
                else
                {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Default;
                }
            }
        }

        /// Handler for rejected speech events.
        private void SpeechRejected(object sender, SpeechRecognitionRejectedEventArgs e)
        {
            //ClearRecognitionHighlights();
        }
    }
}