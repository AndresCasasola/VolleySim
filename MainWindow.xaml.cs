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

        // Racket
        private readonly float racketSize = 140; // Racket size from on end to the other
        private Point racketPos1; // Racket 1 position
        private Point racketPos2; // Racket 1 position
        private float racketTheta1; // Angle from x axis following the right hand rule
        private float racketTheta2; // Angle from x axis following the right hand rule
        private readonly Brush racketBrush = Brushes.Blue; // Brush used to draw ball

        // Ball
        private readonly double initialVel = 5; // Initial velocity of the ball
        private readonly double gravity = 0.4; // Gravity of the ball
        private Point ballPos; // Ball position
        private Vector ballVel; // Ball velocity
        private bool ballOnScreen = false; // If there is or not a ball on the screen
        private readonly Brush ballBrush = Brushes.Blue; // Brush used to draw ball
        private float ballSize = 20;
        private DrawingGroup drawingGroupBall; // Drawing group for skeleton rendering output
        private DrawingImage imageSourceBall; // Drawing image that we will display

        // RGB
        private WriteableBitmap colorBitmap; // Bitmap that will hold color information
        private byte[] colorPixels; // Intermediate storage for the color data received from the camera


        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();
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
                ballVel = new Vector(0, initialVel);
                ballPos = new Point();

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

                if(ballOnScreen)
                {
                    ballPos = ballPos + ballVel;
                    if (ballPos.Y > RenderHeight + ballSize / 2.0) // If ball is out of screen
                        ballOnScreen = false;
                    else                                           // If not
                        ballVel.Y += gravity;
                }
                else
                {
                    ballPos.X = RenderWidth * 0.75;
                    ballPos.Y = 0;
                    ballVel.X = 0;
                    ballVel.Y = initialVel;
                    ballOnScreen = true;
                }

                // Screen edges collision
                if (ballPos.Y < -ballSize / 2)
                    ballVel.Y = -ballVel.Y;
                if (ballPos.X < -ballSize / 2  ||  ballPos.X > RenderWidth + ballSize / 2)
                    ballVel.X = -ballVel.X;
                
                // Racket 1 collision
                ballVel = ToRacketSystem(ballVel, racketPos1, racketTheta1);
                ballPos = ToRacketSystem(ballPos, racketPos1, racketTheta1);
                if (ballPos.X < ballSize / 2 && ballPos.X > -ballSize / 2 && ballPos.Y < racketSize / 2 && ballPos.Y > -racketSize / 2)
                {
                    ballPos.X = -ballPos.X - ballSize * Math.Sign(ballVel.X); // Reflects Position
                    ballVel.X = -ballVel.X; // Reflects velocity
                }
                ballVel = ToScreenSystem(ballVel, racketPos1, racketTheta1);
                ballPos = ToScreenSystem(ballPos, racketPos1, racketTheta1);

                // Racket 2 collision
                ballVel = ToRacketSystem(ballVel, racketPos2, racketTheta2);
                ballPos = ToRacketSystem(ballPos, racketPos2, racketTheta2);
                if (ballPos.X < ballSize / 2 && ballPos.X > -ballSize / 2 && ballPos.Y < racketSize / 2 && ballPos.Y > -racketSize / 2)
                {
                    ballPos.X = -ballPos.X - ballSize * Math.Sign(ballVel.X); // Reflects Position
                    ballVel.X = -ballVel.X; // Reflects velocity
                }
                ballVel = ToScreenSystem(ballVel, racketPos2, racketTheta2);
                ballPos = ToScreenSystem(ballPos, racketPos2, racketTheta2);

                dc.DrawEllipse(ballBrush, null, ballPos, ballSize, ballSize);
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
                            DrawRacket(skel, dc, ref racketPos1, ref racketTheta1);
                            oneplayer = true;
                        }
                        else
                        {
                            DrawRacket(skel, dc, ref racketPos2, ref racketTheta2);
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

        private void DrawRacket(Skeleton skeleton, DrawingContext drawingContext, ref Point racketPos, ref float racketTheta)
        {
            Point rightEnd = new Point(0, racketSize / 2);
            Point leftEnd = new Point(0, -racketSize / 2);
            Point wrist;
            Point elbow;
            Joint wristJoint;
            Joint elbowJoint;


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

            racketPos.X = wrist.X;
            racketPos.Y = wrist.Y;
            racketTheta = (float)Math.Acos((wrist.X - elbow.X) / Point.Subtract(wrist, elbow).Length);
            if (wrist.Y < elbow.Y)
                racketTheta = -racketTheta;

            rightEnd = ToScreenSystem(rightEnd, racketPos, racketTheta);
            leftEnd = ToScreenSystem(leftEnd, racketPos, racketTheta);

            drawingContext.DrawLine(new Pen(racketBrush, 5), rightEnd, leftEnd);
        }

        private Point ToScreenSystem(Point point, Point racketPos, float racketTheta)
        {
            Matrix mat = new Matrix(Math.Cos(racketTheta), Math.Sin(racketTheta), 
                                                      -Math.Sin(racketTheta), Math.Cos(racketTheta), 
                                                      racketPos.X, racketPos.Y);
            return mat.Transform(point);
        }

        private Vector ToScreenSystem(Vector vector, Point racketPos, float racketTheta)
        {
            Point point;

            Matrix mat = new Matrix(Math.Cos(racketTheta), Math.Sin(racketTheta),
                                                      -Math.Sin(racketTheta), Math.Cos(racketTheta),
                                                      0, 0);
            point = new Point(vector.X, vector.Y);
            point = mat.Transform(point);
            vector.X = point.X;
            vector.Y = point.Y;
            return vector;
        }

        private Point ToRacketSystem(Point point, Point racketPos, float racketTheta)
        {
            Matrix mat = new Matrix(Math.Cos(racketTheta), Math.Sin(racketTheta),
                                                      -Math.Sin(racketTheta), Math.Cos(racketTheta),
                                                      racketPos.X, racketPos.Y);
            mat.Invert();
            return mat.Transform(point);
        }

        private Vector ToRacketSystem(Vector vector, Point racketPos, float racketTheta)
        {
            Point point;

            Matrix mat = new Matrix(Math.Cos(racketTheta), Math.Sin(racketTheta),
                                                      -Math.Sin(racketTheta), Math.Cos(racketTheta),
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
    }
}