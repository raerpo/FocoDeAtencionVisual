using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Kinect;

namespace NeuralNetwork
{
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
        }

        KinectSensor kinect;
        SkeletonPoint head1, head2;
        Joint leftShoulder1, leftShoulder2, rightShoulder1, rightShoulder2;
        
        int playerIndex1, playerIndex2;
        int persons;
        Boolean personDetected = false;

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            kinect = KinectSensor.KinectSensors[0];
            kinect.DepthStream.Enable();
            kinect.SkeletonStream.Enable();
            kinect.ColorStream.Enable();

            kinect.DepthFrameReady += new EventHandler<DepthImageFrameReadyEventArgs>(kinect_DepthFrameReady);
            kinect.SkeletonFrameReady += new EventHandler<SkeletonFrameReadyEventArgs>(kinect_SkeletonFrameReady);
            kinect.ColorFrameReady += new EventHandler<ColorImageFrameReadyEventArgs>(kinect_ColorFrameReady);

            kinect.Start();
        }

        void kinect_ColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
            {
                if (colorFrame == null) return;
                byte[] colorData = new byte[colorFrame.PixelDataLength];
                colorFrame.CopyPixelDataTo(colorData);
                image6.Source = BitmapSource.Create( 
                            colorFrame.Width, colorFrame.Height, // image dimensions 
                            96, 96,  // resolution - 96 dpi for video frames 
                            PixelFormats.Bgr32, // video format 
                            null,               // palette - none 
                            colorData,          // video data 
                            colorFrame.Width * colorFrame.BytesPerPixel // stride 
                            ); 
            }
        }

        void kinect_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    Skeleton[] skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);

                    persons = 0;
                    foreach (Skeleton skeleton in skeletons)
                    {
                        if (persons >= 1 && skeleton.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            head2 = skeleton.Joints[JointType.Head].Position;
                            leftShoulder2 = skeleton.Joints[JointType.ShoulderLeft];
                            rightShoulder2 = skeleton.Joints[JointType.ShoulderRight];
                        }
                        if (persons < 1 && skeleton.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            head1 = skeleton.Joints[JointType.Head].Position;
                            leftShoulder1 = skeleton.Joints[JointType.ShoulderLeft];
                            rightShoulder1 = skeleton.Joints[JointType.ShoulderRight];
                            persons++;
                        }
                    }
                    if (persons != 0)
                    {
                        personDetected = true;
                    }
                }
            }
        }

        DepthImagePixel[] depthPixelData = null;
        byte[] depthColorImage = null;
        WriteableBitmap depthBMP = null;
        byte[] regionData1 = null;
        byte[] regionData2 = null;
        int index = 0;

        void kinect_DepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (depthFrame != null && personDetected == true)
                {
                    if (depthPixelData == null)
                    {
                        depthPixelData = new DepthImagePixel[depthFrame.PixelDataLength];
                    }

                    if (depthColorImage == null)
                    {
                        depthColorImage = new byte[depthFrame.PixelDataLength * 4];
                    }

                    depthFrame.CopyDepthImagePixelDataTo(depthPixelData);

                    findPlayersIndex(depthPixelData);

                    int closerDepthHead1 = closerDepthValueHead1(depthPixelData);
                    int closerDepthHead2 = closerDepthValueHead2(depthPixelData);

                    int depthColorImagePos = 0;

                    for (int depthPos = 0; depthPos < depthPixelData.Length; depthPos++)
                    {
                        int depthValue = depthPixelData[depthPos].Depth;
                         
                        if (depthPixelData[depthPos].PlayerIndex == 0)
                        {
                            depthColorImage[depthColorImagePos++] = 0; // Azul 
                            depthColorImage[depthColorImagePos++] = 0; // Verde 
                            depthColorImage[depthColorImagePos++] = 0; // Rojo 
                        }
                        else
                        {
                            if (depthPixelData[depthPos].PlayerIndex == playerIndex1)
                            {
                                byte depthByte = imageRange(closerDepthHead1, depthValue);
                                depthColorImage[depthColorImagePos++] = depthByte; // Azul 
                                depthColorImage[depthColorImagePos++] = depthByte; // Verde 
                                depthColorImage[depthColorImagePos++] = depthByte; // Rojo 
                            }
                            if (depthPixelData[depthPos].PlayerIndex == playerIndex2)
                            {
                                byte depthByte = imageRange(closerDepthHead2, depthValue);
                                depthColorImage[depthColorImagePos++] = depthByte; // Azul 
                                depthColorImage[depthColorImagePos++] = depthByte; // Verde 
                                depthColorImage[depthColorImagePos++] = depthByte; // Rojo 
                            }
                            
                        }
                        // Transparencia 
                        depthColorImagePos++;
                    }

                    if (depthBMP == null)
                    {
                        depthBMP = new WriteableBitmap(
                            depthFrame.Width,
                            depthFrame.Height,
                            96,
                            96,
                            PixelFormats.Bgr32,
                            null);
                    }

                    depthBMP.WritePixels(new Int32Rect(0, 0, depthFrame.Width, depthFrame.Height), depthColorImage, depthFrame.Width * 4, 0);
                    
                    image1.Source = depthBMP;

                    var head1Depth = kinect.CoordinateMapper.MapSkeletonPointToDepthPoint(head1, DepthImageFormat.Resolution640x480Fps30);
                    var head2Depth = kinect.CoordinateMapper.MapSkeletonPointToDepthPoint(head2, DepthImageFormat.Resolution640x480Fps30);

                    double angle1 = findAngle(leftShoulder1, rightShoulder1);
                    
                    double angle2 = findAngle(leftShoulder2, rightShoulder2);

                    image4.RenderTransform = new RotateTransform(angle1, 50, 50);
                    image5.RenderTransform = new RotateTransform(angle2, 50, 50);

                    label1.Content = angle1.ToString("N"+2) + "°";
                    label2.Content = angle2.ToString("N"+2) + "°";

                    if (head1Depth.X < 50) head1Depth.X = 50;
                    if (head1Depth.Y < 50) head1Depth.Y = 50;

                    if (head2Depth.X < 50) head2Depth.X = 50;
                    if (head2Depth.Y < 50) head2Depth.Y = 50;

                    Canvas.SetLeft(ellipse1, (double)(head1Depth.X - 50));
                    Canvas.SetTop(ellipse1, (double)(head1Depth.Y - 50));

                    Canvas.SetLeft(ellipse2, (double)(head2Depth.X - 50));
                    Canvas.SetTop(ellipse2, (double)(head2Depth.Y - 50));

                    Int32Rect regionCoords1 = new Int32Rect((int)(head1Depth.X - 50), (int)(head1Depth.Y - 50), 100, 100);
                    Int32Rect regionCoords2 = new Int32Rect((int)(head2Depth.X - 50), (int)(head2Depth.Y - 50), 100, 100);

                    if (regionData1 == null)
                    {
                        regionData1 = new byte[100 * 100 * 4];
                    }

                    if (regionData2 == null)
                    {
                        regionData2 = new byte[100 * 100 * 4];
                    }

                    depthBMP.CopyPixels(regionCoords1, regionData1, 400, 0);
                    depthBMP.CopyPixels(regionCoords2, regionData2, 400, 0);
                    
                    BitmapSource regionImage1 = BitmapSource.Create(
                                    100, 100, 96, 96, PixelFormats.Bgr32, null, regionData1, 400);
                    BitmapSource regionImage2 = BitmapSource.Create(
                                    100, 100, 96, 96, PixelFormats.Bgr32, null, regionData2, 400);

                    if (index <= 200)
                    {
                        WriteJpeg("imagen0_" + Convert.ToString(index) + ".jpeg", regionImage1);
                        index++;
                    }
                    else {
                        ellipse3.Fill = new SolidColorBrush(Colors.Green);
                    }
                    image2.Source = regionImage1;
                    image3.Source = regionImage2;

                }
            }
        }


        int closerDepthValueHead1(DepthImagePixel[] depthPixelData)
        {
            int closerValue = 4000;
            for (int depthPos = 0; depthPos < depthPixelData.Length; depthPos++)
            {
                if (depthPixelData[depthPos].PlayerIndex == playerIndex1)
                {
                    if (depthPixelData[depthPos].Depth < closerValue && depthPixelData[depthPos].Depth >= 800)
                    {
                        closerValue = depthPixelData[depthPos].Depth;
                    }
                }
            }
            return closerValue;
        }

        int closerDepthValueHead2(DepthImagePixel[] depthPixelData)
        {
            int closerValue = 4000;
            for (int depthPos = 0; depthPos < depthPixelData.Length; depthPos++)
            {
                if (depthPixelData[depthPos].PlayerIndex == playerIndex2)
                {
                    if (depthPixelData[depthPos].Depth < closerValue && depthPixelData[depthPos].Depth >= 800)
                    {
                        closerValue = depthPixelData[depthPos].Depth;
                    }
                }
            }
            return closerValue;
        }

        double findAngle(Joint hombroIzquierdo, Joint hombroDerecho)
        {

            if (hombroDerecho.TrackingState == JointTrackingState.NotTracked && hombroIzquierdo.TrackingState == JointTrackingState.NotTracked)
            {
                return 0;
            }

            if (hombroDerecho.TrackingState == JointTrackingState.NotTracked ||
                hombroDerecho.TrackingState == JointTrackingState.Inferred)
            {
                return 90;
            }

            else if (hombroIzquierdo.TrackingState == JointTrackingState.NotTracked ||
                hombroIzquierdo.TrackingState == JointTrackingState.Inferred)
            {
                return -90;
            }
            else
            {
                return Math.Atan2(
                    hombroDerecho.Position.Z - hombroIzquierdo.Position.Z,
                    hombroDerecho.Position.X - hombroIzquierdo.Position.X) * 180.0 / Math.PI;
            }
        }

        void findPlayersIndex(DepthImagePixel[] depthPixelData)
        {
            playerIndex1 = 0;
            playerIndex2 = 0;
            var index1Finded = false;
            for (int depthPos = 0; depthPos < depthPixelData.Length; depthPos++)
            {
                if (depthPixelData[depthPos].PlayerIndex != 0 && index1Finded == false)
                {
                    ellipse1.Visibility = System.Windows.Visibility.Visible;
                    playerIndex1 = depthPixelData[depthPos].PlayerIndex;
                    index1Finded = true;
                }
                if (depthPixelData[depthPos].PlayerIndex != 0 && depthPixelData[depthPos].PlayerIndex != playerIndex1)
                {
                    playerIndex2 = depthPixelData[depthPos].PlayerIndex;
                    ellipse2.Visibility = System.Windows.Visibility.Visible;
                }
                if (playerIndex1 != 0 && playerIndex2 != 0)
                {
                    return;
                }
            }
            if (playerIndex2 == 0)
            {
                ellipse2.Visibility = System.Windows.Visibility.Hidden;
            }
            if (playerIndex1 == 0)
            {
                ellipse1.Visibility = System.Windows.Visibility.Hidden;
            }
        }

        byte imageRange(int closer, int depthValue)
        {
            int farther = closer + 500;
            if (depthValue < closer)
            {
                return Convert.ToByte(255);
            }
            if(depthValue > farther)
            {
                return Convert.ToByte(0);
            }
            else
            {
                var m = -255.0 / (farther - closer);
                return Convert.ToByte((m * depthValue) - (m * farther));
            }
        }

        void WriteJpeg(string fileName, BitmapSource bmp)
        {
            JpegBitmapEncoder encoder = new JpegBitmapEncoder();
            BitmapFrame outputFrame = BitmapFrame.Create(bmp);
            encoder.Frames.Add(outputFrame);

            using (FileStream file = File.OpenWrite(fileName))
            {
                encoder.Save(file);
            }
        }
       
    }
}
