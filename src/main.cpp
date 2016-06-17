#include <sys/stat.h>
#include "stdlib.h"

#include "GoalPostDetector/GoalPostDetector.h"

using namespace cv;
using namespace std;

String exactPos = "./Exact Goal Positions/exactGoalPosition";
String approxPos = "./Approximate Goal Positions/approximateGoalPosition";
String algoPos = "algorithmGoalPosition";
String algoPosResults= "GoalPositionAlgorithmResults";

String imgFile = "./Images/im";
string imagesOutFolder = "ImagesWithGoal/";
String outImgFile = "imageWithGoal";


void createDirectory(String folderName)
{
	   struct stat sb;

	    if (!(stat(folderName.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)))
	    {

	     string folderCreateCommand = "mkdir " + folderName;
	     system(folderCreateCommand.c_str());
	    }
}

int main( int argc, char** argv )
{

    createDirectory(imagesOutFolder);

    ofstream resultsFile;

    resultsFile.open((imagesOutFolder+algoPosResults+".txt").c_str(), ios_base::out);

    for(int i=1; i<=20; i++)
    {

    stringstream ss;
    ss << setw(4) << setfill('0') << i;
    string number = ss.str();

    Mat image = imread(imgFile+number+".png", CV_LOAD_IMAGE_COLOR);   // Read the file
       	    if(! image.data )                              // Check for invalid input
       	    {
       	        cout <<  "Could not open or find the image" << std::endl ;
       	        return -1;
       	    }

   vector<Point2f> exactGoalPos=GoalPostDetector::readPositions(exactPos+number+".txt");
   vector<Point2f> approxGoalPos=GoalPostDetector::readPositions(approxPos+number+".txt");


   GoalPostDetector  goalPostDetector(image,approxGoalPos);


   Point2f shift= goalPostDetector.findTheShift();


   for(int i=0; i< approxGoalPos.size(); i++) approxGoalPos.at(i)+=shift;


   GoalPostDetector::drawTheGoalBar(image,approxGoalPos, Scalar(0,255,0));
   //GoalPostDetector::drawTheGoalBar(image,exactGoalPos, Scalar(0,0,255));

   resultsFile<<number<<" ("<<floor(shift.x*10.0+0.5)/10.0<<" "<<floor(shift.y*10.0+0.5)/10.0<<") "<<floor(cv::norm(shift)*10.0+0.5)/10.0<<endl;

   GoalPostDetector::writePositions(imagesOutFolder+algoPos+number+".txt",approxGoalPos);
   imwrite(imagesOutFolder+outImgFile+number+".png",image);

    }

    resultsFile.close();

    cout<<"Finished"<<endl;
//  namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
//  imshow( "Display window", image );                   // Show our image inside it.

    waitKey(0);                                          // Wait for a keystroke in the window
    return 0;
}
