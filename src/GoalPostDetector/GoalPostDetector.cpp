#include "GoalPostDetector.h"


void GoalPostDetector::createPaddedImg()
{
	paddedImg.create(img.rows + 4.0f*windowSearchSize, img.cols + 4.0f*windowSearchSize, img.type());
	paddedImg.setTo(cv::Scalar::all(255));
	img.copyTo(paddedImg(Rect(2.0*windowSearchSize, 2.0*windowSearchSize, img.cols, img.rows)));
}

 void GoalPostDetector::myLine(Mat img, Point2f start, Point2f end, Scalar color )
{
    int thickness = 4;
    int lineType = 8;
    line( img, start, end,
          color,
          thickness,
          lineType );
}

 void GoalPostDetector::drawTheGoalBar (Mat img,vector<Point2f> goalPos, Scalar color )
{
	   for(int i=0; i< goalPos.size() - 1; i++)
	      myLine(img,goalPos.at(i),goalPos.at(i+1),color);
}


void GoalPostDetector::getPointsAlongTheLine(int iteration )
{



	 LineIterator it(paddedImg, approxGoalPos.at(iteration),  approxGoalPos.at(iteration+1), 8);
	 int stepPos = 0;
	   // it.count-1 because we don't want to include the last point, it will be included in the next segment
      for(int i = 0; i < it.count-1; i++, ++it)
		   {
	           if(stepPos!=i) continue; //sample points only with interval lineStep

	           samplePointsAlongThePost.push_back(it.pos());

		       stepPos+=lineStep;

		   }

}
void GoalPostDetector::getPointsAlongTheGoalBar()
		{

	  for(int i = 0; i < approxGoalPos.size()-1; i++) getPointsAlongTheLine(i);

	  samplePointsAlongThePost.push_back(approxGoalPos.at(approxGoalPos.size()-1)); // put the last point
 }

 vector<Point2f> GoalPostDetector::readPositions(String filename)
{
	   ifstream inputFile;
	   vector<Point2f> goalPos;
	   inputFile.open(filename.c_str(), ios_base::in);
	   float x,y;
	   while(!inputFile.eof())
	     {
		   inputFile>>x;
		   inputFile>>y;

	       goalPos.push_back(Point(x,y));
	     }

	   inputFile.close();
	   return goalPos;
}
 void GoalPostDetector::writePositions(String filename, const vector<Point2f> &goalPos)
{
	   ofstream outputFile;

	   outputFile.open(filename.c_str(), ios_base::out);

	   for(int i=0;i<goalPos.size(); i++)
	   {

		outputFile<<goalPos.at(i).x<<" ";

		outputFile<<goalPos.at(i).y;

		outputFile<<endl;
	   }

	   outputFile.close();

}

void GoalPostDetector::CannyThreshold()
{
  /// Reduce noise with a kernel 3x3

  Mat greyMat;
  cv::cvtColor(originalImg, greyMat, CV_BGR2GRAY);

  cv::equalizeHist(greyMat, greyMat); //normalize the image, if lighting conditions are changing

  Mat detected_edges;
  blur( greyMat, detected_edges, Size(3,3) );

  /// Canny detector
  Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernelSize );

  /// Using Canny's output as a mask, we display our result
   img = Scalar::all(0);

   greyMat.copyTo( img, detected_edges);

   cv::threshold(img, img, 0, 255, cv::THRESH_BINARY);

}

double GoalPostDetector::objectiveFunction(float const *x)

{
       double finalSum =0.0;
       double currentSum =0.0;
       float xLeft,yLeft;
	   for(int i=0; i< samplePointsAlongThePost.size(); i++)
	   {

		  float stepSquare = squareSize / numOfInnerSquares;
          for(float j = stepSquare, it=0; it<numOfInnerSquares; j+=stepSquare, it++)
          {	   currentSum=0.0;


          xLeft= samplePointsAlongThePost.at(i).x-j/2.0+ (x[0]*scale)+ 2.0*windowSearchSize;
          yLeft= samplePointsAlongThePost.at(i).y-j/2.0+ (x[1]*scale)+ 2.0*windowSearchSize;
	       for(int k= xLeft; k< stepSquare+xLeft; k++)
	    	   for(int l=yLeft; l< stepSquare+yLeft; l++)
	    	   {
	    		   if(paddedImg.at<uchar>(Point(k, l)))
	    			   currentSum+=1.0;

	    	   }

	          currentSum/=j*j; //normalize, compress the information

	          if(currentSum<EPS&&it==numOfInnerSquares-1) currentSum*=penalization; //penalize trivial solution

          finalSum+=currentSum;
	   }

	   }


   return finalSum;
}

Point2f GoalPostDetector::findTheShift ()
{

	CannyThreshold();
	createPaddedImg();
	getPointsAlongTheGoalBar();

	 CMAES<float> evo;


		    float *arFunvals, *xfinal, *const*pop;
		    // Initialize everything
		    const int dim = 2;
		    float xstart[dim];


		    for(int i=0; i<dim; i++) xstart[i] = 0.0;
		    float stddev[dim];
		    for(int i=0; i<dim; i++) stddev[i] = 0.05;
		    Parameters<float> parameters;

		    parameters.init(dim, xstart, stddev);
		    arFunvals = evo.init(parameters);

		    // Iterate until stop criterion holds
		    while(!evo.testForTermination())
		    {
		      // Generate lambda new search points, sample population
		      pop = evo.samplePopulation();

		      // condition: solution dx and dy should not be bigger than squareSize
		       for (int i = 0; i < evo.get(CMAES<float>::PopSize); ++i)
		           while (abs(pop[i][0])*scale>=windowSearchSize||abs(pop[i][1])*scale>=windowSearchSize)
		            evo.reSampleSingle(i);

		      // evaluate the new search points using objectiveFunction from above
		      for (int i = 0; i < evo.get(CMAES<float>::Lambda); ++i)
		        {
		          arFunvals[i]=  objectiveFunction(pop[i]);
		        }
		      evo.updateDistribution(arFunvals);

		    }

	       return Point2f(evo.getNew(CMAES<float>::XMean)[0]*scale,
	    		          evo.getNew(CMAES<float>::XMean)[1]*scale);

}
