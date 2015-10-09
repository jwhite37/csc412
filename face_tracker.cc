///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2010, Jason Mora Saragih, all rights reserved.
//
// This file is part of FaceTracker.
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//
//     * The software is provided under the terms of this licence stricly for
//       academic, non-commercial, not-for-profit purposes.
//     * Redistributions of source code must retain the above copyright notice, 
//       this list of conditions (licence) and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright 
//       notice, this list of conditions (licence) and the following disclaimer 
//       in the documentation and/or other materials provided with the 
//       distribution.
//     * The name of the author may not be used to endorse or promote products 
//       derived from this software without specific prior written permission.
//     * As this software depends on other libraries, the user must adhere to 
//       and keep in place any licencing terms of those libraries.
//     * Any publications arising from the use of this software, including but
//       not limited to academic journal and conference publications, technical
//       reports and manuals, must cite the following work:
//
//       J. M. Saragih, S. Lucey, and J. F. Cohn. Face Alignment through 
//       Subspace Constrained Mean-Shifts. International Conference of Computer 
//       Vision (ICCV), September, 2009.
//
// THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED 
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO 
// EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF 
// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
///////////////////////////////////////////////////////////////////////////////

// Hugely Modified by Md. Iftekhar Tanveer for Blind Emotion Project

#include <Tracker.h>
//#include <highgui.h>
#include <iostream>
//=============================================================================
void Draw(cv::Mat &image,cv::Mat &shape,cv::Mat &con,cv::Mat &tri,cv::Mat &visi)
{
  int i,n = shape.rows/2; cv::Point p1,p2; cv::Scalar c;

  //draw triangulation
  c = CV_RGB(0,0,0);
  for(i = 0; i < tri.rows; i++){
    if(visi.at<int>(tri.at<int>(i,0),0) == 0 ||
       visi.at<int>(tri.at<int>(i,1),0) == 0 ||
       visi.at<int>(tri.at<int>(i,2),0) == 0)continue;
    p1 = cv::Point(shape.at<double>(tri.at<int>(i,0),0),
		   shape.at<double>(tri.at<int>(i,0)+n,0));
    p2 = cv::Point(shape.at<double>(tri.at<int>(i,1),0),
		   shape.at<double>(tri.at<int>(i,1)+n,0));
    cv::line(image,p1,p2,c);
    p1 = cv::Point(shape.at<double>(tri.at<int>(i,0),0),
		   shape.at<double>(tri.at<int>(i,0)+n,0));
    p2 = cv::Point(shape.at<double>(tri.at<int>(i,2),0),
		   shape.at<double>(tri.at<int>(i,2)+n,0));
    cv::line(image,p1,p2,c);
    p1 = cv::Point(shape.at<double>(tri.at<int>(i,2),0),
		   shape.at<double>(tri.at<int>(i,2)+n,0));
    p2 = cv::Point(shape.at<double>(tri.at<int>(i,1),0),
		   shape.at<double>(tri.at<int>(i,1)+n,0));
    cv::line(image,p1,p2,c);
  }
  //draw connections
  c = CV_RGB(0,0,255);
  for(i = 0; i < con.cols; i++){
    if(visi.at<int>(con.at<int>(0,i),0) == 0 ||
       visi.at<int>(con.at<int>(1,i),0) == 0)continue;
    p1 = cv::Point(shape.at<double>(con.at<int>(0,i),0),
		   shape.at<double>(con.at<int>(0,i)+n,0));
    p2 = cv::Point(shape.at<double>(con.at<int>(1,i),0),
		   shape.at<double>(con.at<int>(1,i)+n,0));
    cv::line(image,p1,p2,c,1);
  }
  //draw points
  for(i = 0; i < n; i++){    
    if(visi.at<int>(i,0) == 0)continue;
    p1 = cv::Point(shape.at<double>(i,0),shape.at<double>(i+n,0));

	// USE THIS TO HIGHLIGHT POINTS FOR FURTHER REFERENCE
	if (i == 57)
		c = CV_RGB(0, 255, 0);
	else
		c = CV_RGB(255, 0, 0);

	cv::circle(image,p1,2,c);
  }return;
}
//=============================================================================
int parse_cmd(int argc, const char** argv,
	      char* ftFile,char* conFile,char* triFile,
	      bool &fcheck,double &scale,int &fpd)
{
  int i; fcheck = false; scale = 1; fpd = -1;
  for(i = 1; i < argc; i++){
    if((std::strcmp(argv[i],"-?") == 0) ||
       (std::strcmp(argv[i],"--help") == 0)){
      std::cout << "track_face:- Written by Jason Saragih 2010" << std::endl
	   << "Performs automatic face tracking" << std::endl << std::endl
	   << "#" << std::endl 
	   << "# usage: ./face_tracker [options]" << std::endl
	   << "#" << std::endl << std::endl
	   << "Arguments:" << std::endl
	   << "-m <string> -> Tracker model (default: ../model/face2.tracker)"
	   << std::endl
	   << "-c <string> -> Connectivity (default: ../model/face.con)"
	   << std::endl
	   << "-t <string> -> Triangulation (default: ../model/face.tri)"
	   << std::endl
	   << "-s <double> -> Image scaling (default: 1)" << std::endl
	   << "-d <int>    -> Frames/detections (default: -1)" << std::endl
	   << "--check     -> Check for failure" << std::endl;
      return -1;
    }
  }
  for(i = 1; i < argc; i++){
    if(std::strcmp(argv[i],"--check") == 0){fcheck = true; break;}
  }
  if(i >= argc)fcheck = false;
  for(i = 1; i < argc; i++){
    if(std::strcmp(argv[i],"-s") == 0){
      if(argc > i+1)scale = std::atof(argv[i+1]); else scale = 1;
      break;
    }
  }
  if(i >= argc)scale = 1;
  for(i = 1; i < argc; i++){
    if(std::strcmp(argv[i],"-d") == 0){
      if(argc > i+1)fpd = std::atoi(argv[i+1]); else fpd = -1;
      break;
    }
  }
  if(i >= argc)fpd = -1;
  for(i = 1; i < argc; i++){
    if(std::strcmp(argv[i],"-m") == 0){
      if(argc > i+1)std::strcpy(ftFile,argv[i+1]);
      else strcpy(ftFile,"../model/face2.tracker");
      break;
    }
  }
  if(i >= argc)std::strcpy(ftFile,"../model/face2.tracker");
  for(i = 1; i < argc; i++){
    if(std::strcmp(argv[i],"-c") == 0){
      if(argc > i+1)std::strcpy(conFile,argv[i+1]);
      else strcpy(conFile,"../model/face.con");
      break;
    }
  }
  if(i >= argc)std::strcpy(conFile,"../model/face.con");
  for(i = 1; i < argc; i++){
    if(std::strcmp(argv[i],"-t") == 0){
      if(argc > i+1)std::strcpy(triFile,argv[i+1]);
      else strcpy(triFile,"../model/face.tri");
      break;
    }
  }
  if(i >= argc)std::strcpy(triFile,"../model/face.tri");
  return 0;
}
float getSystemTime(){
	return cv::getTickCount()/cv::getTickFrequency()*1000;
}
//=============================================================================
int main(int argc, const char** argv)
{
  //parse command line arguments
  char ftFile[256],conFile[256],triFile[256];
  bool fcheck = false; double scale = 1; int fpd = -1; bool show = true;
  if(parse_cmd(argc,argv,ftFile,conFile,triFile,fcheck,scale,fpd)<0)return 0;

  //set other tracking parameters
  std::vector<int> wSize1(1); wSize1[0] = 7;
  std::vector<int> wSize2(3); wSize2[0] = 11; wSize2[1] = 9; wSize2[2] = 7;
  int nIter = 5; double clamp=3,fTol=0.01; 
  FACETRACKER::Tracker model(ftFile);
  cv::Mat tri=FACETRACKER::IO::LoadTri(triFile);
  cv::Mat con=FACETRACKER::IO::LoadCon(conFile);
  
  //initialize camera and display window
  cv::Mat frame,gray,im; double fps=0; char sss[256]; std::string text; 
  CvCapture* camera = cvCreateCameraCapture(CV_CAP_ANY); if(!camera)return -1;
  int64 t1,t0 = cvGetTickCount(); int fnum=0;
  cvNamedWindow("Face Tracker",1);
  std::cout << "Hot keys: "        << std::endl
	    << "\t ESC - quit"     << std::endl
	    << "\t d   - Redetect" << std::endl;

  //loop until quit (i.e user presses ESC)
  bool failed = true;
  double pitch = 0, yaw = 0, roll = 0;

  // Main loop

  //***********************************************************************
  // HOMEWORK GLOBALS - Jeffery A. White
  //***********************************************************************

  // Keeping track of the previous variables so we can compute deltas on each loop
  double previous_pitch = 0;
  double previous_yaw = 0;
  double previous_roll = 0;

  // Keeping track of distance measurements for smile and surprise detection
  int previous_corner_distance = 0;
  int previous_center_distance = 0;

  // Variables to keep track of triggers hit
  bool face_down = false;
  bool face_up = false;
  bool face_left = false;
  bool face_right = false;
  bool face_roll_left = false;
  bool face_roll_right = false;

  // Hold when these triggers set for interest decay
  float face_down_set = 0.0f;
  float face_up_set = 0.0f;
  float face_left_set = 0.0f;
  float face_right_set = 0.0f;
  float face_roll_left_set = 0.0f;
  float face_roll_right_set = 0.0f;

  // Threshold for a change of interest
  // Noise cancellation for small motion
  double delta_threshold = 0.12;
  int delta_smile_threshold = 9;
  int delta_surprise_threshold = 20;

  // Maximum time we let a trigger be alive before we forget that it happened
  // Noise cancellation for jerky behavior
  float trigger_time_to_live = 1500.0f;

  // Tick Counts for creating a window of observation
  int max_tick_count = 5;
  int max_tick_count_other = 2; // Surprise and Smile are quicker actions
  int cur_tick_count = 0;
  int cur_tick_count_other = 0;

  bool debug = false;
  bool detection = false;

  while(1){ 
			//grab image, resize and flip
			IplImage* I = cvQueryFrame(camera); if(!I)continue; frame = I;
			if(scale == 1)im = frame; 
			else cv::resize(frame,im,cv::Size(scale*frame.cols,scale*frame.rows));
			cv::flip(im,im,1); cv::cvtColor(im,gray,CV_BGR2GRAY);

			//track this image
			std::vector<int> wSize; if(failed)wSize = wSize2; else wSize = wSize1; 
			if(model.Track(gray,wSize,fpd,nIter,clamp,fTol,fcheck) == 0){
			  int idx = model._clm.GetViewIdx(); failed = false;

			  // Extract pitch, yaw and roll movements 
			  pitch = model._clm._pglobl.at<double>(1);
			  yaw = model._clm._pglobl.at<double>(2);
			  roll = model._clm._pglobl.at<double>(3);

			  // =================================================================
			  //printf("time: %f, Pitch = %0.2f  Yaw = %0.2f  Roll = %0.2f\n",getSystemTime(),pitch,yaw,roll);

			  //***********************************************************************************
			  // HOMEWORK - Detection Code - Jeffery A. White
			  // - Don't forget the globals up above the main loop
			  //***********************************************************************************

			  // Calculate the change in movements up to this point
			  double delta_pitch = pitch - previous_pitch;
			  double delta_yaw = yaw - previous_yaw;
			  double delta_roll = roll - previous_roll;

			  // Checking thresholds and setting triggers
			  if (delta_pitch > delta_threshold)
			  {
				  face_down = true;
				  face_down_set = getSystemTime();
				  if (debug)
					  printf("FACE DOWN\n");
			  }

			  if (-delta_pitch > delta_threshold)
			  {
				  face_up = true;
				  face_up_set = getSystemTime();
				  if (debug)
					  printf("FACE UP\n");
			  }

			  if (delta_yaw > delta_threshold)
			  {
				  face_left = true;
				  face_left_set = getSystemTime();
				  if (debug)
					  printf("FACE LEFT\n");
			  }

			  if (-delta_yaw > delta_threshold)
			  {
				  face_right = true;
				  face_right_set = getSystemTime();
				  if (debug)
					  printf("FACE RIGHT\n");
			  }

			  if (-delta_roll > delta_threshold)
			  {
				  face_roll_left = true;
				  face_roll_left_set = getSystemTime();
				  if (debug)
					  printf("FACE ROLL LEFT\n");
			  }

			  if (delta_roll > delta_threshold)
			  {
				  face_roll_right = true;
				  face_roll_right_set = getSystemTime();
				  if (debug)
					  printf("FACE ROLL RIGHT\n");
			  }

			  // Analysis of triggers to determine movements
			  if (face_down && face_up)
			  {
				  printf("INDICATED YES!\n");
				  face_down = false;
				  face_up = false;
				  face_down_set = 0.0f;
				  face_up_set = 0.0f;
				  detection = true;
			  }

			  if (face_left && face_right)
			  {
				  printf("INDICATED NO!\n");
				  face_left = false;
				  face_right = false;
				  face_left_set = 0.0f;
				  face_right_set = 0.0f;
				  detection = true;
			  }

			  if (face_roll_left && face_roll_right)
			  {
				  printf("INDICATED INDIAN YES!\n");
				  face_roll_left = false;
				  face_roll_right = false;
				  face_roll_left_set = 0.0f;
				  face_roll_right_set = 0.0f;
				  detection = true;
			  }

			  // Analysis for smile/surprise detection
			  int j = model._shape.rows / 2;
			  cv::Point l_corner = cv::Point(model._shape.at<double>(48, 0), model._shape.at<double>(48 + j, 0));
			  cv::Point r_corner = cv::Point(model._shape.at<double>(54, 0), model._shape.at<double>(54 + j, 0));
			  cv::Point top = cv::Point(model._shape.at<double>(51, 0), model._shape.at<double>(51 + j, 0));
			  cv::Point bottom = cv::Point(model._shape.at<double>(57, 0), model._shape.at<double>(57 + j, 0));

			  int center_distance = bottom.y - top.y;
			  int corner_distance = r_corner.x - l_corner.x;

			  //printf("Prev: %d Center Distance: %d\n", previous_corner_distance, corner_distance);

			  if ((center_distance - previous_center_distance) > delta_surprise_threshold)
			  {
				  printf("SURPRISE DETECTED!\n");
				  detection = true;
			  }

			  if ((corner_distance - previous_corner_distance) > delta_smile_threshold)
			  {
				  printf("SMILE DETECTED!\n");
				  detection = true;
			  }


			  // Checking decay of triggers and resetting as necessary
			  // This is noise cancellation of random jerky bahavior, or an attempt at it.
			  if (face_down_set != 0.0f && (getSystemTime() - face_down_set) > trigger_time_to_live)
				  face_down_set = 0.0f;
			  if (face_up_set != 0.0f && (getSystemTime() - face_up_set) > trigger_time_to_live)
				  face_up_set = 0.0f;
			  if (face_left_set != 0.0f && (getSystemTime() - face_left_set) > trigger_time_to_live)
				  face_left_set = 0.0f;
			  if (face_right_set != 0.0f && (getSystemTime() - face_right_set) > trigger_time_to_live)
				  face_right_set = 0.0f;
			  if (face_roll_left_set != 0.0f && (getSystemTime() - face_roll_left_set) > trigger_time_to_live)
				  face_roll_left_set = 0.0f;
			  if (face_roll_right_set != 0.0f && (getSystemTime() - face_roll_right_set) > trigger_time_to_live)
				  face_roll_right_set = 0.0f;

			  // Update our previous values if we've reached enough ticks, otherwise just up the current tick count
			  // Also do a reset after detection so we can reset the delta, helps remove some of the extra 'false' hits
			  if ((cur_tick_count == max_tick_count) || detection)
			  {
				  previous_pitch = pitch;
				  previous_yaw = yaw;
				  previous_roll = roll;
				  cur_tick_count = 0;
				  detection = false;
			  }
			  else
			  {
				  cur_tick_count++;
			  }

			  // Similar resets for Surprise and Smile detection, on a different tick counter
			  if ((cur_tick_count_other == max_tick_count_other) || detection)
			  {
				  int n = model._shape.rows / 2;
				  cv::Point l_corner = cv::Point(model._shape.at<double>(48, 0), model._shape.at<double>(48 + n, 0));
				  cv::Point r_corner = cv::Point(model._shape.at<double>(54, 0), model._shape.at<double>(54 + n, 0));
				  cv::Point top = cv::Point(model._shape.at<double>(51, 0), model._shape.at<double>(51 + n, 0));
				  cv::Point bottom = cv::Point(model._shape.at<double>(57, 0), model._shape.at<double>(57 + n, 0));

				  previous_center_distance = bottom.y - top.y;
				  previous_corner_distance = r_corner.x - l_corner.x;

				  //printf("Center Distance: %d\n", previous_center_distance);
				  //printf("Corner Distance: %d\n", previous_corner_distance);
				  //printf("TOP: %d BOTTOM: %d\n",top.y,bottom.y); // 60 bottom - top seems decent to start
				  //printf("RIGHT: %d LEFT: %d\n", r_corner.x, l_corner.x); // 70 right - left seems decent to start


				  cur_tick_count_other = 0;
				  detection = false;
			  }
			  else
			  {
				  cur_tick_count_other++;
			  }
			  // =================================================================
	  
			  Draw(im,model._shape,con,tri,model._clm._visi[idx]); 
			}else{
			  if(show){cv::Mat R(im,cvRect(0,0,150,50)); R = cv::Scalar(0,0,255);}
			  model.FrameReset(); failed = true;
			}     
			//draw framerate on display image 
			if(fnum >= 9){      
			  t1 = cvGetTickCount();
			  fps = 10.0/((double(t1-t0)/cvGetTickFrequency())/1e+6); 
			  t0 = t1; fnum = 0;
			}else fnum += 1;
			if(show){
			  sprintf(sss,"%d frames/sec",(int)ceil(fps)); text = sss;
			  cv::putText(im,text,cv::Point(10,20),
				  CV_FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(255,255,255));
			}
			//show image and check for user input
			imshow("Face Tracker",im); 
			int c = cvWaitKey(5);
			if(c == 27)
				break; 
			else 
				if(char(c) == 'd')model.FrameReset();
  } // End of main loop

  return 0;
}
//=============================================================================
