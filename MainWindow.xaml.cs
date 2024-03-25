using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.UI;
using Emgu.CV.Util;
using Emgu.CV.Structure;
using Emgu.CV.Features2D;

using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
//using Emgu.CV.CvEnum;
using Emgu.CV.XObjdetect;
using System.IO;
using System.Drawing;
using Emgu.CV.Stitching;
using System.Drawing.Drawing2D;
using System.Data;
using System.Drawing.Imaging;

namespace WpfApp5
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
            //Mat templateImage = CvInvoke.Imread(@"D:\Foxconn dataset\dataset\Image_20220328171941644.jpg");
            //Mat inputImage = CvInvoke.Imread(@"D:\Foxconn dataset\dataset\OK\Image_20220328171643368.jpg");

            //// Create feature detector and descriptor extractor
            //using (var detector = new ORB())
            //using (var matcher = new BFMatcher(DistanceType.Hamming))
            //{
            //    // Detect keypoints and compute descriptors for the template image
            //    VectorOfKeyPoint templateKeypoints = new VectorOfKeyPoint();
            //    Matrix<float> templateDescriptors = new Matrix<float>(100, 32); // Adjust matrix size based on detector
            //    detector.DetectAndCompute(templateImage, null, templateKeypoints, templateDescriptors, false);

            //    // Detect keypoints and compute descriptors for the input image
            //    VectorOfKeyPoint inputKeypoints = new VectorOfKeyPoint();
            //    Matrix<float> inputDescriptors = new Matrix<float>(100, 32); // Adjust matrix size based on detector
            //    detector.DetectAndCompute(inputImage, null, inputKeypoints, inputDescriptors, false);

            //    // Match keypoints between the template and input images
            //    var matches = new VectorOfVectorOfDMatch();
            //    matcher.Add(templateDescriptors);
            //    matcher.KnnMatch(inputDescriptors, matches, k: 2);

            //    // Filter good matches using ratio test
            //    float ratioThreshold = 0.7f;
            //    var goodMatches = new VectorOfDMatch();
            //    for (int i = 0; i < matches.Size; i++)
            //    {
            //        var match = matches[i];
            //        if (match.Size == 2 && match[0].Distance < ratioThreshold * match[1].Distance)
            //        {
            //            // Create a new array containing the single match
            //            MDMatch[] goodMatch = { match[0] };

            //            // Add the current match to the vector of good matches
            //            goodMatches.Push(goodMatch);
            //        }
            //    }

            //    // Extract matched keypoints from the template and input images
            //    var templatePoints = new List<PointF>();
            //    var inputPoints = new List<PointF>();
            //    for (int i = 0; i < goodMatches.Size; i++)
            //    {
            //        MDMatch match = goodMatches[i];
            //        templatePoints.Add(templateKeypoints[match.QueryIdx].Point);
            //        inputPoints.Add(inputKeypoints[match.TrainIdx].Point);
            //    }
            //    // Convert List<PointF> to VectorOfPointF
            //    VectorOfPointF templatePointsVector = new VectorOfPointF(templatePoints.ToArray());
            //    VectorOfPointF inputPointsVector = new VectorOfPointF(inputPoints.ToArray());


            //    // Estimate homography matrix using RANSAC
            //    Mat homography = CvInvoke.FindHomography(inputPointsVector, templatePointsVector, Emgu.CV.CvEnum.RobustEstimationAlgorithm.LMEDS);
            //    // Warp the input image to align with the template image
            //    Mat alignedImage = new Mat();
            //    CvInvoke.WarpPerspective(inputImage, alignedImage, homography, templateImage.Size);



            //    // Display or further process the aligned image
            //    BitmapSource bitmapSource_alignedImage = ToBitmapSource(alignedImage);
            //    Edges.Source = bitmapSource_alignedImage;
            //}

            //BitmapSource bitmapSource = ToBitmapSource(inputImage);
            //InputImage.Source = bitmapSource;

            //Mat grayImage = new Mat();
            //CvInvoke.CvtColor(inputImage, grayImage, ColorConversion.Bgr2Gray); // Convert to grayscale
            //BitmapSource bitmapSource_grayImage = ToBitmapSource(grayImage);
            //ProcessedImage.Source = bitmapSource_grayImage;
            //Mat Canny_edges = new Mat();
            //CvInvoke.Canny(grayImage, Canny_edges, 100, 200);
            ////BitmapSource bitmapSource_edges = ToBitmapSource(Canny_edges);
            ////Edges.Source = bitmapSource_edges;

            //----------------------------------------
            // Load input and template images
            /*
            Mat inputImage = CvInvoke.Imread(@"D:\Foxconn dataset\dataset\Image_20220328171941644 - Copy.jpg", ImreadModes.Color); 
            Mat templateImage = CvInvoke.Imread(@"D:\Foxconn dataset\dataset\Image_20220328171941644.jpg", ImreadModes.Color); 

            // Convert images to grayscale
            Mat inputGray = new Mat();
            Mat templateGray = new Mat();
            CvInvoke.CvtColor(inputImage, inputGray, ColorConversion.Bgr2Gray);
            CvInvoke.CvtColor(templateImage, templateGray, ColorConversion.Bgr2Gray);

            // Detect keypoints and compute descriptors for the template image
            VectorOfKeyPoint templateKeyPoints = new VectorOfKeyPoint();
            Mat templateDescriptors = new Mat();
            using (var detector = new ORB())
                detector.DetectAndCompute(templateGray, null, templateKeyPoints, templateDescriptors, false);

            // Detect keypoints and compute descriptors for the input image
            VectorOfKeyPoint inputKeyPoints = new VectorOfKeyPoint();
            Mat inputDescriptors = new Mat();
            using (var detector = new ORB())
                detector.DetectAndCompute(inputGray, null, inputKeyPoints, inputDescriptors, false);

            // Match descriptors between template and input images
            BFMatcher matcher = new BFMatcher(DistanceType.Hamming);
            VectorOfVectorOfDMatch matches = new VectorOfVectorOfDMatch();
            matcher.Add(templateDescriptors);
            matcher.KnnMatch(inputDescriptors, matches, k: 2);

            // Filter good matches
            float ratioThreshold = 0.7f;
            VectorOfDMatch goodMatches = new VectorOfDMatch();
            for (int i = 0; i < matches.Size; i++)
            {
                var match = matches[i];
                if (match.Size == 2 && match[0].Distance < ratioThreshold * match[1].Distance)
                {
                    // Create a new array containing the single match
                    MDMatch[] goodMatch = { match[0] };

                    // Add the current match to the vector of good matches
                    goodMatches.Push(goodMatch);
                }
            }

            // Extract matched keypoints from the template and input images
            VectorOfPointF templatePoints = new VectorOfPointF();
            VectorOfPointF inputPoints = new VectorOfPointF();
            for (int i = 0; i < goodMatches.Size; i++)
            {
                MDMatch match = goodMatches[i];
                PointF templatePoint = templateKeyPoints[match.QueryIdx].Point;
                PointF inputPoint = inputKeyPoints[match.TrainIdx].Point;
                PointF[] templatePointArray = { templatePoint };
                PointF[] inputPointArray = { inputPoint };
                templatePoints.Push(templatePointArray);
                inputPoints.Push(inputPointArray);
            }

            // Estimate homography matrix
            Mat homography = CvInvoke.FindHomography(inputPoints, templatePoints, RobustEstimationAlgorithm.Ransac);

            // Warp the input image to align with the template image
            Mat alignedImage = new Mat();
            //CvInvoke.WarpPerspective(inputImage, alignedImage, homography, templateImage.Size);
            CvInvoke.WarpPerspective(inputGray, alignedImage, homography, templateImage.Size);
            // Convert aligned image to BitmapImage
            BitmapSource bitmapSource = alignedImage.ToBitmapSource();

            // Display aligned image in the Image control
            Edges.Source = bitmapSource;
            */
            // Load input and template images
            Mat inputImage = CvInvoke.Imread(@"D:\Foxconn dataset\dataset\OK\Image_20220328171633946.jpg", ImreadModes.Color); // Replace "input_image.jpg" with the path to your input image
            Mat templateImage = CvInvoke.Imread(@"D:\Foxconn dataset\dataset\Image_20220328171941644.jpg", ImreadModes.Color); // Replace "template_image.jpg" with the path to your template image

            // Align input image to template image
            Mat alignedImage = AlignImages(inputImage, templateImage);

            // Display aligned image in WPF
            BitmapSource bitmapSource_alignedImage = alignedImage.ToBitmapSource();
            alignedImageShow.Source = bitmapSource_alignedImage;

        }

        // Function to align input image to template image using template matching
        private Mat AlignImages(Mat inputImage, Mat templateImage)
        {
            // Convert images to grayscale
            Mat grayInputImage = new Mat();
            Mat grayTemplateImage = new Mat();
            CvInvoke.CvtColor(inputImage, grayInputImage, ColorConversion.Bgr2Gray);
            CvInvoke.CvtColor(templateImage, grayTemplateImage, ColorConversion.Bgr2Gray);

            // Perform template matching
            Mat result = new Mat();
            CvInvoke.MatchTemplate(grayInputImage, grayTemplateImage, result, TemplateMatchingType.CcoeffNormed);

            // Find best match location
            double minVal = 0, maxVal = 0; // Initialize minVal and maxVal
            System.Drawing.Point minLoc = new System.Drawing.Point(), maxLoc = new System.Drawing.Point(); // Initialize Point variables
            CvInvoke.MinMaxLoc(result, ref minVal, ref maxVal, ref minLoc, ref maxLoc);


            // Calculate offset for alignment
            int offsetX = maxLoc.X - grayTemplateImage.Width / 2;
            int offsetY = maxLoc.Y - grayTemplateImage.Height / 2;

            // Define the rotation center
            Mat alignedImage = new Mat();

            PointF centerInput = new PointF(inputImage.Width / 2, inputImage.Height / 2);
            Matrix<double> rotationMatrixInput = new Matrix<double>(new double[,] { { 1, 0, 0 }, { 0, 1, 0 } }); // Placeholder rotation matrix

            PointF centerAligned = new PointF(alignedImage.Width / 2, alignedImage.Height / 2);
            Matrix<double> rotationMatrixAligned = new Matrix<double>(new double[,] { { 1, 0, 0 }, { 0, 1, 0 } }); // Placeholder rotation matrix


            // Apply the rotation transformation to the input image
            Mat rotatedInputImage = new Mat();
            CvInvoke.WarpAffine(inputImage, rotatedInputImage, rotationMatrixInput, inputImage.Size);

            // Apply the rotation transformation to the aligned image
            Mat rotatedAlignedImage = new Mat();


            // Check if the source image dimensions are valid
            if (alignedImage.Width > 0 && alignedImage.Height > 0)
            {
                CvInvoke.WarpAffine(alignedImage, rotatedAlignedImage, rotationMatrixAligned, alignedImage.Size);
            }
            else
            {
                // Handle the case where the source image dimensions are not valid
                // For example, you can log an error message or take appropriate action
                Console.WriteLine("Error: Source image dimensions are not valid.");
            }

            return alignedImage;

        }


        private BitmapSource ToBitmapSource(Mat image)
        {
            using (var bitmap = image.ToBitmap())
            {
                IntPtr ptr = bitmap.GetHbitmap(); // Get the Hbitmap

                // Create BitmapSource
                BitmapSource bitmapSource = System.Windows.Interop.Imaging.CreateBitmapSourceFromHBitmap(
                    ptr,
                    IntPtr.Zero,
                    System.Windows.Int32Rect.Empty,
                    BitmapSizeOptions.FromEmptyOptions());

                // Release Hbitmap
                DeleteObject(ptr);

                return bitmapSource;
            }
        }

        // Import DeleteObject from gdi32.dll to release Hbitmap
        [System.Runtime.InteropServices.DllImport("gdi32.dll")]
        [return: System.Runtime.InteropServices.MarshalAs(System.Runtime.InteropServices.UnmanagedType.Bool)]
        internal static extern bool DeleteObject(IntPtr hObject);

        private void Start_Click(object sender, RoutedEventArgs e)
        {

        }


            
        

    }
}