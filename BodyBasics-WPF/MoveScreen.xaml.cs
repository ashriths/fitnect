//------------------------------------------------------------------------------
// <copyright file="MoveScreen.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using System.Windows.Media.Media3D;
    using Microsoft.Kinect;

    /// <summary>
    /// Interaction logic for MoveScreen
    /// </summary>
    public partial class MoveScreen : Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 10;

        /// <summary>
        /// Radius of drawn joint circles
        /// </summary>
        private const double jointSize = 20;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing joints that are moving at wrong angle
        /// </summary>
        private readonly Brush jointWrongBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        private string exerciseType;

        const int TEXTWIDTH = 30;

        public static int prevExerciseState = 0;
        public static int exerciseState = 0;
        public static int reps = 0;

        public static int restFramecount = 50;

        // stop watch for static squat countdown
        private DateTime startTime;
        private TimeSpan elapsedTime;

        // track if countdown already started
        private Boolean watchStarted;

        // track if squat in correct form
        private Boolean inPosition;

        // total exercise time for squat
        private double totalSquatTime;

        /// <summary>
        /// Initializes a new instance of the MoveScreen class.
        /// </summary>
        public MoveScreen(string exerciseType)
        {
            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            
            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // set the exercise type
            this.exerciseType = exerciseType;

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MoveScreen_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MoveScreen_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }

            if (dataReceived)
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    int penIndex = 0;
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {
                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();
                            List<JointType> interestedJoints = new List<JointType>();
                            interestedJoints.Add(JointType.ElbowLeft);
                            interestedJoints.Add(JointType.ElbowRight);
                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)

                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }
                            IDictionary<string, double> interestedJointAngles = this.GetInterstedJointAngles(joints);
                            this.DrawBody(joints, jointPoints, dc, drawPen, interestedJointAngles);

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                            exerciseState = this.getExerciseState(joints, jointPoints, dc, drawPen, interestedJointAngles);
                            if (exerciseState != prevExerciseState)
                            {
                                reps += 1;
                                prevExerciseState = exerciseState;
                                restFramecount = 0;
                            }
                            else {
                                restFramecount += 1;
                            }
                        }
                    }

                    // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }
        }

        public double AngleBetweenTwoVectors(Vector3D vectorA, Vector3D vectorB)
        {
            double dotProduct;
            vectorA.Normalize();
            vectorB.Normalize();
            dotProduct = Vector3D.DotProduct(vectorA, vectorB);

            return Math.Round((double)Math.Acos(dotProduct) / Math.PI * 180, 2);
        }

        private int getExerciseState(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints,
            DrawingContext drawingContext, Pen drawingPen, IDictionary<string, double> interestedJointAngles)
        {
            int e;
            if (interestedJointAngles["RightShoulder"] < 140.00 && interestedJointAngles["LeftShoulder"] < 140.00)
            {
                return 0;
            }
            else if (interestedJointAngles["RightShoulder"] > 140.00 && interestedJointAngles["LeftShoulder"] > 140.00)
            {
                return 1;
            }
            else return exerciseState;
        }

        private double GetAngleBetweenJoints(JointType a, JointType b, JointType c, IReadOnlyDictionary<JointType, Joint> joints)
        {
            if (joints.ContainsKey(a) && joints.ContainsKey(b) && joints.ContainsKey(c))
            {
                Vector3D boneA = new Vector3D(joints[a].Position.X, joints[a].Position.Y, joints[a].Position.Z);
                Vector3D boneB = new Vector3D(joints[b].Position.X, joints[b].Position.Y, joints[b].Position.Z);
                Vector3D boneC = new Vector3D(joints[c].Position.X, joints[c].Position.Y, joints[c].Position.Z);
                return this.AngleBetweenTwoVectors(boneA - boneB, boneA - boneC);
            }
            return 0.0;
        }

        private IDictionary<string, double> GetInterstedJointAngles(IReadOnlyDictionary<JointType, Joint> joints)
        {
            IDictionary<string, List<JointType>> interestedJoints = new Dictionary<string, List<JointType>>{
                { "RightArm", new List<JointType>{JointType.ElbowRight, JointType.WristRight, JointType.ShoulderRight}},
                { "LeftArm", new List<JointType>{JointType.ElbowLeft, JointType.WristLeft, JointType.ShoulderLeft}} ,
                { "RightLeg", new List<JointType>{JointType.SpineBase, JointType.HipRight, JointType.KneeRight}} ,
                { "LeftLeg", new List<JointType>{JointType.SpineBase, JointType.HipLeft, JointType.KneeLeft}} ,
                { "LeftShoulder", new List<JointType>{JointType.SpineShoulder, JointType.ShoulderLeft, JointType.ElbowRight}} ,
                { "RightShoulder", new List<JointType>{JointType.SpineShoulder, JointType.ShoulderRight, JointType.ElbowLeft}},
                { "LeftHip", new List<JointType>{JointType.SpineBase, JointType.HipLeft, JointType.SpineMid}},
                { "RightHip", new List<JointType>{JointType.SpineBase, JointType.HipLeft, JointType.SpineMid}},
                { "RightUpperArm", new List<JointType>{JointType.ShoulderRight, JointType.SpineShoulder, JointType.ElbowRight}},
                { "LeftUpperArm", new List<JointType> { JointType.ShoulderLeft, JointType.SpineShoulder, JointType.ElbowLeft }},
                { "RightKnee", new List<JointType> { JointType.KneeRight, JointType.HipRight, JointType.AnkleRight }},
                { "LeftKnee", new List<JointType> { JointType.KneeLeft, JointType.HipLeft, JointType.AnkleLeft }},
                { "Back", new List<JointType> { JointType.SpineMid, JointType.SpineBase, JointType.SpineShoulder }},
                { "Groin", new List<JointType> { JointType.SpineBase, JointType.KneeRight, JointType.KneeLeft }}
         
        };

            IDictionary<string, double> interestedJointAngles = new Dictionary<string, double>();
            foreach (KeyValuePair<string, List<JointType>> kv in interestedJoints) {
                interestedJointAngles.Add(kv.Key, this.GetAngleBetweenJoints(kv.Value[0], kv.Value[1], kv.Value[2], joints));
            }
            return interestedJointAngles;
        }
        
        public void PrintJointWarnings(String warnings, DrawingContext drawingContext, Point point)
        {
            int yStart = displayHeight - 100 - TEXTWIDTH;

            drawingContext.DrawText(
                    new FormattedText(warnings,
                    CultureInfo.GetCultureInfo("en-us"),
                    FlowDirection.LeftToRight,
                    new Typeface("Verdana"),
                    10, System.Windows.Media.Brushes.White),
                    new System.Windows.Point(point.X, point.Y + TEXTWIDTH)
                );
            yStart -= TEXTWIDTH;
 
        }


        public void PrintWarnings(List<String> warnings, DrawingContext drawingContext)
        {
            int yStart = displayHeight - 120 - TEXTWIDTH;
            Rect rect = new Rect(new System.Windows.Point(0, 300), new System.Windows.Size(150, 150));
            //drawingContext.DrawRectangle(System.Windows.Media.Brushes.OrangeRed, (System.Windows.Media.Pen)null, rect);
            foreach (string warn in warnings) {
                drawingContext.DrawText(
                        new FormattedText(warn,
                        CultureInfo.GetCultureInfo("en-us"),
                        FlowDirection.LeftToRight,
                        new Typeface("Verdana"),
                        10, System.Windows.Media.Brushes.OrangeRed),
                        new System.Windows.Point(10, yStart + TEXTWIDTH)
                    );
                yStart += TEXTWIDTH;
            }
        }

        public List<string> CheckForWrongPosture(IDictionary<string, double> interestedJointAngles, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext) {
            List<string> warnings = new List<string>();
            if (string.Equals(exerciseType, "Jumping Jacks"))
            {
                if (interestedJointAngles.ContainsKey("RightArm"))
                {
                    if (interestedJointAngles["RightArm"] < 110.00)
                    {
                        warnings.Add("Right Arm is not straight");
                        drawingContext.DrawEllipse(this.jointWrongBrush, null, jointPoints[JointType.ElbowRight], jointSize, jointSize);
                        PrintJointWarnings("Keep Arm straight!", drawingContext, jointPoints[JointType.ElbowRight]);
                    }
                }
                if (interestedJointAngles.ContainsKey("LeftArm"))
                {
                    if (interestedJointAngles["LeftArm"] < 110.00)
                    {
                        warnings.Add("Left Arm is not straight");
                        drawingContext.DrawEllipse(this.jointWrongBrush, null, jointPoints[JointType.ElbowLeft], jointSize, jointSize);
                        PrintJointWarnings("Keep Arm straight!", drawingContext, jointPoints[JointType.ElbowLeft]);
                    }
                }

            }


            //Angle checks for Squat, still need to work on arm angles
            else if (string.Equals(exerciseType, "Squat"))
            {
                if (interestedJointAngles.ContainsKey("RightUpperArm"))
                {
                    if (interestedJointAngles["RightUpperArm"] < 80.00)
                    {
                        warnings.Add("Right Arm is not stretched");
                        drawingContext.DrawEllipse(this.jointWrongBrush, null, jointPoints[JointType.ShoulderLeft], jointSize, jointSize);
                        PrintJointWarnings("Keep Arm stretched out in front!", drawingContext, jointPoints[JointType.ShoulderLeft]);
                        inPosition = false;
                    }
                    else
                    {
                        inPosition = true;
                    }
                }
                if (interestedJointAngles.ContainsKey("LeftUpperArm"))
                {
                    if (interestedJointAngles["LeftUpperArm"] < 80.00)
                    {
                        warnings.Add("Left Arm is not stretched");
                        drawingContext.DrawEllipse(this.jointWrongBrush, null, jointPoints[JointType.ShoulderRight], jointSize, jointSize);
                        PrintJointWarnings("Keep Arm stretched out in front!", drawingContext, jointPoints[JointType.ShoulderRight]);
                        inPosition = false;
                    }
                    else
                    {
                        inPosition = true;
                    }
                }

                if (interestedJointAngles.ContainsKey("RightKnee"))
                {
                    if (interestedJointAngles["RightKnee"] < 75.00 || interestedJointAngles["RightKnee"] > 130.00)
                    {
                        warnings.Add("Right knee not at right angle");
                        drawingContext.DrawEllipse(this.jointWrongBrush, null, jointPoints[JointType.KneeRight], jointSize, jointSize);
                        PrintJointWarnings("Keep knees bended at 90 degree!", drawingContext, jointPoints[JointType.KneeRight]);
                        inPosition = false;
                    }
                    else
                    {
                        inPosition = true;
                    }
                }
                if (interestedJointAngles.ContainsKey("LeftKnee"))
                {
                    if (interestedJointAngles["LeftKnee"] < 75.00 || interestedJointAngles["LeftKnee"] > 130.00)
                    {
                        warnings.Add("Left knee not at right angle");
                        drawingContext.DrawEllipse(this.jointWrongBrush, null, jointPoints[JointType.KneeLeft], jointSize, jointSize);
                        PrintJointWarnings("Keep knees bended at 90 degree!", drawingContext, jointPoints[JointType.KneeLeft]);
                        inPosition = false;
                    }
                    else
                    {
                        inPosition = true;
                    }
                }

                if (interestedJointAngles.ContainsKey("Back"))
                {
                    if (interestedJointAngles["Back"] < 170.00)
                    {
                        warnings.Add("Back NoT Straight");
                        drawingContext.DrawEllipse(this.jointWrongBrush, null, jointPoints[JointType.SpineMid], jointSize, jointSize);
                        PrintJointWarnings("Keep back straight!", drawingContext, jointPoints[JointType.SpineMid]);
                    }
                }

            }

            /*if (interestedJointAngles.ContainsKey("RightShoulder"))
            {
                if (interestedJointAngles["RightShoulder"] < 110.00)
                {
                    warnings.Add("Right Arm is not straight.");
                    drawingContext.DrawEllipse(this.jointWrongBrush, null, jointPoints[JointType.ShoulderRight], jointSize, jointSize);
                    PrintJointWarnings("Move Right Arm!", drawingContext, jointPoints[JointType.ShoulderRight]);
                }
            }
            if (interestedJointAngles.ContainsKey("LeftShoulder"))
            {
                if (interestedJointAngles["LeftShoulder"] < 110.00)
                {
                    warnings.Add("Left Arm is not straight.");
                    drawingContext.DrawEllipse(this.jointWrongBrush, null, jointPoints[JointType.ShoulderLeft], jointSize, jointSize);
                    PrintJointWarnings("Move Left Arm!", drawingContext, jointPoints[JointType.ShoulderLeft]);
                }
            }

            /*if (interestedJointAngles.ContainsKey("RightLeg"))
            {
                if (interestedJointAngles["RightLeg"] > 50.00)
                {
                    warnings.Add("You are not moving your Right Leg");
                    drawingContext.DrawEllipse(this.jointWrongBrush, null, jointPoints[JointType.KneeRight], jointSize, jointSize);
                    PrintJointWarnings("Move Right Leg", drawingContext, jointPoints[JointType.KneeRight]);
                }
            }
            if (interestedJointAngles.ContainsKey("LeftLeg"))
            {
                if (interestedJointAngles["LeftLeg"] > 50.00)
                {
                    warnings.Add("You are not moving your Left Leg");
                    drawingContext.DrawEllipse(this.jointWrongBrush, null, jointPoints[JointType.KneeLeft], jointSize, jointSize);
                    PrintJointWarnings("Move Right Leg", drawingContext, jointPoints[JointType.KneeLeft]);
                }
            }*/

            return warnings;
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, 
            Pen drawingPen, IDictionary<string, double> interestedJointAngles)
        {

            
            int start = -TEXTWIDTH;
            foreach (KeyValuePair<string, double> kv in interestedJointAngles)
            {
                if (kv.Value > 0.0)
                {
                    drawingContext.DrawText(
                        new FormattedText(  kv.Key.PadLeft(10, ' ') + "    " +kv.Value.ToString(),
                        CultureInfo.GetCultureInfo("en-us"),
                        FlowDirection.LeftToRight,
                        new Typeface("Verdana"),
                        6, System.Windows.Media.Brushes.White),
                        new System.Windows.Point(0, start + 6)
                    );
                    start += 6;
                }
            }

            if (String.Equals(exerciseType, "Jumping Jacks"))
            {
                //draw rep count window for Jumping Jacks
                Rect rect2 = new Rect(new System.Windows.Point(370, 30), new System.Windows.Size(120, 60));
                drawingContext.DrawRectangle(System.Windows.Media.Brushes.White, (System.Windows.Media.Pen)null, rect2);
                drawingContext.DrawText(
                            new FormattedText("Reps:",
                            CultureInfo.GetCultureInfo("en-us"),
                            FlowDirection.LeftToRight,
                            new Typeface("Verdana"),
                            18, System.Windows.Media.Brushes.Black),
                            new System.Windows.Point(this.displayWidth - 120, 30)
                        );
                drawingContext.DrawText(
                            new FormattedText(" " + (int)reps / 4,
                            CultureInfo.GetCultureInfo("en-us"),
                            FlowDirection.LeftToRight,
                            new Typeface("Verdana"),
                            24, System.Windows.Media.Brushes.Black),
                            new System.Windows.Point(this.displayWidth - 130, 60)
                        );

                // check for no movement
                if (restFramecount > 100)
                {
                    drawingContext.DrawText(
                            new FormattedText("You are not exercising. Do you want to exit?",
                            CultureInfo.GetCultureInfo("en-us"),
                            FlowDirection.LeftToRight,
                            new Typeface("Verdana"),
                            12, System.Windows.Media.Brushes.White),
                            new System.Windows.Point(150, this.displayHeight - 50)
                        );
                }
            }

            //draw count down window for squat
            else if (String.Equals(exerciseType, "Squat"))
            {
                if (!watchStarted)
                {
                    watchStarted = true;
                    startTime = DateTime.Now;
                    inPosition = true;
                    totalSquatTime = 40.0;
                }
                else
                {
                    drawingContext.DrawText(
                                            new FormattedText("Static Squat",
                                            CultureInfo.GetCultureInfo("en-us"),
                                            FlowDirection.LeftToRight,
                                            new Typeface("Verdana"),
                                            30, System.Windows.Media.Brushes.White),
                                            new System.Windows.Point(this.displayWidth - 400, 10)
                                            );
                    drawingContext.DrawText(
                                            new FormattedText("Swing arms to shoulder level",
                                            CultureInfo.GetCultureInfo("en-us"),
                                            FlowDirection.LeftToRight,
                                            new Typeface("Verdana"),
                                            15, System.Windows.Media.Brushes.White),
                                            new System.Windows.Point(this.displayWidth - 400, 40)
                                            );
                    drawingContext.DrawText(
                                            new FormattedText("Bend knees, keep chest up",
                                            CultureInfo.GetCultureInfo("en-us"),
                                            FlowDirection.LeftToRight,
                                            new Typeface("Verdana"),
                                            15, System.Windows.Media.Brushes.White),
                                            new System.Windows.Point(this.displayWidth - 400, 60)
                                            );
                    drawingContext.DrawText(
                                            new FormattedText("Pause and stay in this position",
                                            CultureInfo.GetCultureInfo("en-us"),
                                            FlowDirection.LeftToRight,
                                            new Typeface("Verdana"),
                                            15, System.Windows.Media.Brushes.White),
                                            new System.Windows.Point(this.displayWidth - 400, 80)
                                            );

                    elapsedTime = DateTime.Now - startTime;
                    double remainTime = totalSquatTime - (elapsedTime.Seconds);
                    double startIn = 10.0 - (elapsedTime.Seconds);

                    // reday info for 10 sec
                    if(elapsedTime.Seconds < 10)
                    {
                         drawingContext.DrawText(
                                                new FormattedText("READY",
                                                CultureInfo.GetCultureInfo("en-us"),
                                                FlowDirection.LeftToRight,
                                                new Typeface("Verdana"),
                                                60, System.Windows.Media.Brushes.White),
                                                new System.Windows.Point(this.displayWidth - 340, 160)
                                                );

                        drawingContext.DrawEllipse(System.Windows.Media.Brushes.LightSkyBlue, (System.Windows.Media.Pen)null,
                                                    new System.Windows.Point(445, 65), 63.0, 63.0);
                        drawingContext.DrawText(
                                    new FormattedText("Start In",
                                    CultureInfo.GetCultureInfo("en-us"),
                                    FlowDirection.LeftToRight,
                                    new Typeface("Verdana"),
                                    20, System.Windows.Media.Brushes.White),
                                    new System.Windows.Point(this.displayWidth - 110, 30)
                                );
                        drawingContext.DrawText(
                                    new FormattedText(" 00:" + startIn + "s",
                                    CultureInfo.GetCultureInfo("en-us"),
                                    FlowDirection.LeftToRight,
                                    new Typeface("Verdana"),
                                    25, System.Windows.Media.Brushes.White),
                                    new System.Windows.Point(this.displayWidth - 120, 60)
                                );

                        inPosition = true;
                        totalSquatTime = 40.0;
                    }

                    //start counting down for 30 sec
                    //else if(elapsedTime.Seconds >= 10 && elapsedTime.Seconds <= 40)
                    else
                    {
                        if (remainTime >= 0)
                        {
                            //increment remaining time if not in position
                            if (!inPosition)
                            {
                                totalSquatTime += 0.05;
                            }

                            remainTime = Math.Round(remainTime, 1);

                            drawingContext.DrawEllipse(System.Windows.Media.Brushes.LightSkyBlue, (System.Windows.Media.Pen)null,
                                                        new System.Windows.Point(445, 65), 63.0, 63.0);
                            drawingContext.DrawText(
                                        new FormattedText("Hold For",
                                        CultureInfo.GetCultureInfo("en-us"),
                                        FlowDirection.LeftToRight,
                                        new Typeface("Verdana"),
                                        20, System.Windows.Media.Brushes.White),
                                        new System.Windows.Point(this.displayWidth - 110, 30)
                                    );
                            drawingContext.DrawText(
                                        new FormattedText(" 00:" + remainTime + "s",
                                        CultureInfo.GetCultureInfo("en-us"),
                                        FlowDirection.LeftToRight,
                                        new Typeface("Verdana"),
                                        22, System.Windows.Media.Brushes.White),
                                        new System.Windows.Point(this.displayWidth - 120, 60)
                                    );
                        }

                        //finish
                        else
                        {
                            drawingContext.DrawEllipse(System.Windows.Media.Brushes.LightSkyBlue, (System.Windows.Media.Pen)null,
                                                        new System.Windows.Point(445, 65), 63.0, 63.0);
                            drawingContext.DrawText(
                                        new FormattedText("Good Job!",
                                        CultureInfo.GetCultureInfo("en-us"),
                                        FlowDirection.LeftToRight,
                                        new Typeface("Verdana"),
                                        20, System.Windows.Media.Brushes.White),
                                        new System.Windows.Point(this.displayWidth - 115, 55)
                                    );

                            //watchStarted = false;
                        }
                    }

                }
            }


            List<string> warnings = this.CheckForWrongPosture(interestedJointAngles, jointPoints, drawingContext);
            this.PrintWarnings(warnings, drawingContext);

          

            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>stra
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }
            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }
    }
}
