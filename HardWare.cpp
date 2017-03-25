#include "stdafx.h"
#include "time.h"
#include "math.h"
#include "Hardware.h"

ImagingResources	CTCSys::IR;

CTCSys::CTCSys()
{
	EventEndProcess = TRUE;
	EventEndMove = TRUE;
	IR.Acquisition = TRUE;
	IR.UpdateImage = TRUE;
	IR.CaptureSequence = FALSE;
	IR.DisplaySequence = FALSE;
	IR.PlayDelay = 30;
	IR.CaptureDelay = 30;
	IR.FrameID = 0;
	IR.CatchBall = FALSE;
	OPENF("c:\\Projects\\RunTest.txt");
}

CTCSys::~CTCSys()
{
	CLOSEF;
}

void CTCSys::QSStartThread()
{
	EventEndProcess = FALSE;
	EventEndMove = FALSE;
	QSMoveEvent = CreateEvent(NULL, TRUE, FALSE, NULL);		// Create a manual-reset and initially nosignaled event handler to control move event
	ASSERT(QSMoveEvent != NULL);

	// Image Processing Thread
	QSProcessThreadHandle = CreateThread(NULL, 0L,
		(LPTHREAD_START_ROUTINE)QSProcessThreadFunc,
		this, NULL, (LPDWORD)&QSProcessThreadHandleID);
	ASSERT(QSProcessThreadHandle != NULL);
	SetThreadPriority(QSProcessThreadHandle, THREAD_PRIORITY_HIGHEST);

	QSMoveThreadHandle = CreateThread(NULL, 0L,
		(LPTHREAD_START_ROUTINE)QSMoveThreadFunc,
		this, NULL, (LPDWORD)&QSMoveThreadHandleID);
	ASSERT(QSMoveThreadHandle != NULL);
	SetThreadPriority(QSMoveThreadHandle, THREAD_PRIORITY_HIGHEST);

	FileStorage fs_left("C:\Projects\BaseballCatcher - 10\calibration\calib_left.yaml", FileStorage::READ);
	fs_left["CameraMatrix"] >> cameraMatrix_left;
	fs_left["DistortionCoefficients"] >> distCoeffs_left;
	FileStorage fs_right("C:\Projects\BaseballCatcher - 10\calibration\calib_right.yaml", FileStorage::READ);
	fs_right["CameraMatrix"] >> cameraMatrix_right;
	fs_right["DistortionCoefficients"] >> distCoeffs_right;
	FileStorage fs_rectify("C:\Projects\BaseballCatcher - 10\calibration\Rectification_baseball_2.xml", FileStorage::READ);
	fs_rectify["R1"] >> R1;
	fs_rectify["P1"] >> P1;
	fs_rectify["R2"] >> R2;
	fs_rectify["P2"] >> P2;
	fs_rectify["Q"] >> Q;

}

void CTCSys::QSStopThread()
{
	// Must close the move event first
	EventEndMove = TRUE;				// Set the falg to true first
	SetEvent(QSMoveEvent);				// must set event to complete the while loop so the flag can be checked
	do {
		Sleep(100);
		// SetEvent(QSProcessEvent);
	} while (EventEndProcess == TRUE);
	CloseHandle(QSMoveThreadHandle);

	// need to make sure camera acquisiton has stopped
	EventEndProcess = TRUE;
	do {
		Sleep(100);
		// SetEvent(QSProcessEvent);
	} while (EventEndProcess == TRUE);
	CloseHandle(QSProcessThreadHandle);
}

long QSMoveThreadFunc(CTCSys *QS)
{
	while (QS->EventEndMove == FALSE) {
		WaitForSingleObject(QS->QSMoveEvent, INFINITE);
		if (QS->EventEndMove == FALSE) QS->Move(QS->Move_X, QS->Move_Y);
		ResetEvent(QS->QSMoveEvent);
	}
	QS->EventEndMove = FALSE;
	return 0;
}


long QSProcessThreadFunc(CTCSys *QS)
{
	int     i;
	int		j = 1; // This iterator is meant to be used to grab the first frame when catchball is grabbed. Dallin.
	int     BufID = 0;
	char    str[32];
	long	FrameStamp;

	//int x_left = 333;
	//int y_left = 88;
	//int x_right = 233;
	//int y_right = 88;
	//int width = 70;
	//int height = 70;

	FrameStamp = 0;
	while (QS->EventEndProcess == FALSE) {
#ifdef PTGREY		// Image Acquisition
		if (QS->IR.Acquisition == TRUE) {
			for (i = 0; i < QS->IR.NumCameras; i++) {
				QS->IR.PGRError = QS->IR.pgrCamera[i]->RetrieveBuffer(&QS->IR.PtGBuf[i]);
				// Get frame timestamp if exact frame time is needed.  Divide FrameStamp by 32768 to get frame time stamp in mSec
				QS->IR.metaData[i] = QS->IR.PtGBuf[i].GetMetadata();
				FrameStamp = QS->IR.metaData[i].embeddedTimeStamp;
				if (QS->IR.PGRError == PGRERROR_OK){
					QS->QSSysConvertToOpenCV(&QS->IR.AcqBuf[i], QS->IR.PtGBuf[i]);		// copy image data pointer to OpenCV Mat structure
				}
			}
			for (i = 0; i < QS->IR.NumCameras; i++) {
				if (QS->IR.CaptureSequence) {
#ifdef PTG_COLOR
					mixChannels(&QS->IR.AcqBuf[i], 1, &QS->IR.SaveBuf[i][QS->IR.FrameID], 1, QS->IR.from_to, 3); // Swap B and R channels anc=d copy out the image at the same time.
#else
					QS->IR.AcqBuf[i].copyTo(QS->IR.SaveBuf[i][QS->IR.FrameID]);
#endif
				}
				else {
#ifdef PTG_COLOR
					mixChannels(&QS->IR.AcqBuf[i], 1, &QS->IR.ProcBuf[i][BufID], 1, QS->IR.from_to, 3); // Swap B and R channels anc=d copy out the image at the same time.
#else
					QS->IR.AcqBuf[i].copyTo(QS->IR.ProcBuf[i]);	// Has to be copied out of acquisition buffer before processing
#endif
				}
			}
		}
#else
		Sleep (100);
#endif
		// Process Image ProcBuf
		if (QS->IR.CatchBall) {  	// Click on "Catch" button to toggle the CatchBall flag when done catching
			// Images are acquired into ProcBuf[0] for left and ProcBuf[1] for right camera


			for (i = 0; i < QS->IR.NumCameras; i++) {
#ifdef PTG_COLOR


				cvtColor(QS->IR.ProcBuf[i][BufID], QS->IR.OutBuf1[i], CV_RGB2GRAY, 0);
#else			// Example using Canny.  Input is ProcBuf.  Output is OutBuf1
				// Create ROI for each camera
				// ****************************************************************************************************************************************************************
				//store the first frames to absolute diff against (to do)
				if (j < 3) // This if statement is meant to grab the first frame when catchball is clicked. Dallin.
				{
					// Need to create child image or small region of interest for processing to exclude background and speed up processing
					if (i == 0)
					{
						QS->IR.ProcBuf[i].copyTo(QS->first_frame_left);
					}
					else
					{
						QS->IR.ProcBuf[i].copyTo(QS->first_frame_right);
					}
				}
				j = j + 1;
				if (i == 0)
				{
					QS->IR.ProcBuf[i].copyTo(QS->image_left);
				}
				else
				{
					QS->IR.ProcBuf[i].copyTo(QS->image_right);
				}

				//Canny(QS->IR.ProcBuf[i], QS->IR.OutBuf1[i], 70, 100);
#endif
				// remove the Canny function above and add your ball detection and trajectory estimation code here
				// calculate your estimated ball x, y location in inches and assigned them to moveX, and moveY below
			}

			QS->image_left(QS->rectangle_left).copyTo(QS->roi_current_left);
			QS->image_right(QS->rectangle_right).copyTo(QS->roi_current_right);
			QS->roi_first_left = QS->first_frame_left(QS->rectangle_left);
			QS->roi_first_right = QS->first_frame_right(QS->rectangle_right);

			QS->image_left.copyTo(QS->image_left_test);


			QS->roi_current_left.copyTo(QS->roi_previous_left);
			QS->roi_current_right.copyTo(QS->roi_previous_right);

			QS->IR.OutBuf1[0] = QS->image_left;
			QS->IR.OutBuf1[1] = QS->image_right;
			if (QS->roi_previous_left.empty() || QS->roi_previous_right.empty())
			{
				QS->roi_previous_left = Mat::zeros(QS->roi_current_left.size(), QS->roi_current_left.type()); // prev frame as black
				QS->roi_previous_right = Mat::zeros(QS->roi_current_right.size(), QS->roi_current_right.type()); // prev frame as black
				//signed 16bit mat to receive signed difference
			}
			absdiff(QS->roi_current_left, QS->roi_first_left, QS->motion_left);
			absdiff(QS->roi_current_right, QS->roi_first_right, QS->motion_right);
			GaussianBlur(QS->motion_left, QS->motion_left, Size(11, 11), 0, 0);
			GaussianBlur(QS->motion_right, QS->motion_right, Size(11, 11), 0, 0);
			threshold(QS->motion_left, QS->frame_thresh_left, 4, 255, 0);
			threshold(QS->motion_right, QS->frame_thresh_right, 4, 255, 0);
			findContours(QS->frame_thresh_left, QS->contours_left, QS->hierarchy_left, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
			findContours(QS->frame_thresh_right, QS->contours_right, QS->hierarchy_right, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));


			for (int k = 0; k < QS->contours_left.size(); k++)
			{
				QS->area_left = contourArea(QS->contours_left[k], false);
				if (QS->area_left > QS->largest_area_left)
				{
					QS->largest_area_left = QS->area_left;
					QS->area_left_iter = k;

				}
			}
			for (int k = 0; k < QS->contours_right.size(); k++)
			{
				QS->area_right = contourArea(QS->contours_right[k], false);
				if (QS->area_right > QS->largest_area_right)
				{
					QS->largest_area_right = QS->area_right;
					QS->area_right_iter = k;
				}
			}
			//QS->contours_left.erase(QS->contours_left.begin() + 0);

			drawContours(QS->frame_thresh_left, QS->contours_left, 0, Scalar(255, 255, 255), 2, LINE_8, 0x7fffffff);

			vector<Moments> contour_moments_left(QS->contours_left.size());
			vector<Moments> contour_moments_right(QS->contours_right.size());
			for (int i = 0; i < QS->contours_left.size(); i++)
			{
				contour_moments_left[i] = moments(QS->contours_left[i], true);
			}
			for (int i = 0; i < QS->contours_right.size(); i++)
			{
				contour_moments_right[i] = moments(QS->contours_right[i], true);
			}

			///  Get the mass centers:
			vector<Point2f> mc_left(QS->contours_left.size());
			vector<Point2f> mc_right(QS->contours_right.size());
			for (int i = 0; i < QS->contours_left.size(); i++)
			{
				mc_left[0] = Point2f(contour_moments_left[i].m10 / contour_moments_left[i].m00, contour_moments_left[i].m01 / contour_moments_left[i].m00);
			}
			for (int i = 0; i < QS->contours_right.size(); i++)
			{
				mc_right[0] = Point2f(contour_moments_right[i].m10 / contour_moments_right[i].m00, contour_moments_right[i].m01 / contour_moments_right[i].m00);
			}



			if (!QS->contours_left.empty())
			{
				QS->x_left = (int)mc_left[0].x + QS->x_left - QS->width / 2;
				QS->y_left = (int)mc_left[0].y + QS->y_left - QS->height / 2;

				if (QS->x_left < 0 || QS->y_left < 0 || QS->x_left > 640 - QS->width || QS->y_left > 480 - QS->height)
				{
					QS->rectangle_left = Rect(QS->x_left_home, QS->y_left_home, QS->width, QS->height);
					QS->perspective_left.clear();
				}
				QS->rectangle_left = Rect(QS->x_left, QS->y_left, QS->width, QS->height);
				circle(QS->IR.OutBuf1[0], Point2f(QS->x_left + QS->width / 2, QS->y_left + QS->height / 2), 5, Scalar(255, 255, 255), 2, LINE_8, 0);
				//QS->IR.OutBuf1[0] = QS->image_left_test;
			}

			if (!QS->contours_right.empty())
			{
				QS->x_right = (int)mc_right[1].x + QS->x_right - QS->width / 2;
				QS->y_right = (int)mc_right[1].y + QS->y_right - QS->height / 2;
				if (QS->x_right < 0 || QS->y_right < 0 || QS->x_right > 640 - QS->width || QS->y_right > 480 - QS->height)
				{
					QS->rectangle_right = Rect(QS->x_right_home, QS->y_right_home, QS->width, QS->height);
					QS->perspective_right.clear();
				}
				QS->rectangle_right = Rect(QS->x_right, QS->y_right, QS->width, QS->height);
				circle(QS->IR.OutBuf1[1], Point2f(QS->x_right + QS->width / 2, QS->y_right + QS->height / 2), 5, Scalar(255, 255, 255), 2, LINE_8, 0);
			}

			rectangle(QS->IR.OutBuf1[0], QS->rectangle_left, Scalar(255, 255, 255), 2, LINE_8, 0);
			rectangle(QS->IR.OutBuf1[1], QS->rectangle_right, Scalar(255, 255, 255), 2, LINE_8, 0);

			if (!QS->contours_left.empty() && !QS->contours_right.empty())
			{
				QS->point_left = { Point2f(QS->x_left + QS->width / 2, QS->y_left + QS->height / 2) };
				undistortPoints(QS->point_left, QS->point_left, QS->cameraMatrix_left, QS->distCoeffs_left, QS->R1, QS->P1);

				QS->point_right = { Point2f(QS->x_right + QS->width / 2, QS->y_right + QS->height / 2) };
				undistortPoints(QS->point_right, QS->point_right, QS->cameraMatrix_right, QS->distCoeffs_right, QS->R2, QS->P2);

				QS->corners_diff_left.push_back(Point3f(QS->point_left[0].x, QS->point_left[0].y,
					QS->point_left[0].x - QS->point_right[0].x));

				QS->corners_diff_right.push_back(Point3f(QS->point_right[0].x, QS->point_right[0].y,
					QS->point_left[0].x - QS->point_right[0].x));

				perspectiveTransform(QS->corners_diff_left, QS->perspective_left, QS->Q);
				perspectiveTransform(QS->corners_diff_right, QS->perspective_right, QS->Q);
				//QS->ball_left.push_back(Point3f(QS->perspective_left[0].x, QS->perspective_left[0].y, QS->perspective_left[0].z));
				//QS->ball_right.push_back(Point3f(QS->perspective_right[0].x, QS->perspective_right[0].y, QS->perspective_right[0].z));
			}

			if(perspective_left.size() > 20)
			{
			for(int i=0;i<leftCam3DBallCoords.size();i++)
    {
        leftToCatcher = cv::Point3f(perspective_left[i].x + x_offset, perspective_left[i].y + y_offset, perspective_left[i].z + z_offset);
        //rightToCatcher = cv::Point3f(rightCam3DBallCoordsInLeftCoordFrame[i].x + x_offset, rightCam3DBallCoordsInLeftCoordFrame[i].y + y_offset, rightCam3DBallCoordsInLeftCoordFrame[i].z + z_offset);
        //ballLeftCatcher.push_back(leftToCatcher);
        //ballRigthCatcher.push_back(rightToCatcher);
        catcher_points.push_back(leftToCatcher);
        // catcher_points.push_back(rightToCatcher);

				//____________OPENCV_METHOD for Least Squares ___________________
	 //y = Ax^3 + Bx^2 + Cx + D

	 //create A_y Mat and b_z Mat
	 cv::Mat A(catcher_points.size(),4, CV_32F);
	 cv::Mat b_yz(catcher_points.size(),1, CV_32F);
	 cv::Mat b_xz(catcher_points.size(),1, CV_32F);

	 //populate the A matrix and the b matrix for the yz data
	 for(int i=0;i<catcher_points.size();i++)
	 {
			 //y = Ax^3 + Bx^2 + Cx + D
			 //.at<float>(y,x)
			 A.at<float>(i,0) = catcher_points[i].z * catcher_points[i].z * catcher_points[i].z;
			 A.at<float>(i,1) = catcher_points[i].z * catcher_points[i].z;
			 A.at<float>(i,2) = catcher_points[i].z;
			 A.at<float>(i,3) = 1;

			 b_yz.at<float>(i,0) = catcher_points[i].y;
			 b_xz.at<float>(i,0) = catcher_points[i].x;
	 }

	 //use Mat::inv to calculate inv(A)
	 cv::Mat A_inv = A.inv(cv::DECOMP_SVD);
	 cv::Mat x_yz(4,1, CV_32F);
	 cv::Mat x_xz(4,1, CV_32F);

	 x_yz = A_inv*b_yz;
	 x_xz = A_inv*b_xz;


	 //generate catcher commands
	 float catcherX_cmd = x_xz.at<float>(3,0);
	 float catcherY_cmd = x_yz.at<float>(3,0);


	 //_____________END OPENCV METHOD for least squares
    }
	}
			QS->area_left_iter = 0;
			QS->area_right_iter = 0;
			QS->largest_area_left = 0;
			QS->largest_area_right = 0;
		} // End if Catchball
		else // This happens when you hit stop catch. Dallin.
		{
			j = 1; // Reset the iterator so when catch ball is clicked again, a new first frame will be grabbed. Dallin.
			QS->perspective_left.clear();
			QS->perspective_right.clear();

		}
		// Display Image
		if (QS->IR.UpdateImage) {
			for (i = 0; i < QS->IR.NumCameras; i++) {
				if (QS->IR.CaptureSequence || QS->IR.DisplaySequence) {
#ifdef PTG_COLOR
					QS->IR.SaveBuf[i][QS->IR.FrameID].copyTo(QS->IR.DispBuf[i]);
#else
					QS->IR.OutBuf[0] = QS->IR.OutBuf[1] = QS->IR.OutBuf[2] = QS->IR.SaveBuf[i][QS->IR.FrameID];
					merge(QS->IR.OutBuf, 3, QS->IR.DispBuf[i]);
#endif
					sprintf_s(str, "%d", QS->IR.FrameID);
					putText(QS->IR.DispBuf[0], str, Point(10, 30), FONT_HERSHEY_SIMPLEX, 1, CV_RGB(0, 255, 0), 2);
					if (QS->IR.PlayDelay) Sleep(QS->IR.PlayDelay);
				}
				else {
#ifdef PTG_COLOR
					QS->IR.ProcBuf[i][BufID].copyTo(QS->IR.DispBuf[i]);
#else
					// Display OutBuf1 when Catch Ball, otherwise display the input image
					QS->IR.OutBuf[0] = QS->IR.OutBuf[1] = QS->IR.OutBuf[2] = (QS->IR.CatchBall) ? QS->IR.OutBuf1[i] : QS->IR.ProcBuf[i];
					merge(QS->IR.OutBuf, 3, QS->IR.DispBuf[i]);
					line(QS->IR.DispBuf[i], Point(320, 0), Point(320, 480), Scalar(0, 255, 0), 1, 8, 0);
					line(QS->IR.DispBuf[i], Point(0, 240), Point(640, 240), Scalar(0, 255, 0), 1, 8, 0);
#endif
				}
				QS->QSSysDisplayImage();
			}
		}
		if (QS->IR.CaptureSequence || QS->IR.DisplaySequence) {
			QS->IR.FrameID++;
			if (QS->IR.FrameID == MAX_BUFFER) {				// Sequence if filled
				QS->IR.CaptureSequence = FALSE;
				QS->IR.DisplaySequence = FALSE;
			}
			else {
				QS->IR.FrameID %= MAX_BUFFER;
			}
		}
		BufID = 1 - BufID;
	}

	QS->EventEndProcess = FALSE;
	return 0;
}

void CTCSys::QSSysInit()
{
	long i, j;
	IR.DigSizeX = 640;
	IR.DigSizeY = 480;
	initBitmapStruct(IR.DigSizeX, IR.DigSizeY);

	// Camera Initialization
#ifdef PTGREY
	IR.cameraConfig.asyncBusSpeed = BUSSPEED_S800;
	IR.cameraConfig.isochBusSpeed = BUSSPEED_S800;
	IR.cameraConfig.grabMode = DROP_FRAMES;			// take the last one, block grabbing, same as flycaptureLockLatest
	IR.cameraConfig.grabTimeout = TIMEOUT_INFINITE;	// wait indefinitely
	IR.cameraConfig.numBuffers = 4;					// really does not matter since DROP_FRAMES is set not to accumulate buffers
	char ErrorMsg[64];

	// How many cameras are on the bus?
	if (IR.busMgr.GetNumOfCameras((unsigned int *)&IR.NumCameras) != PGRERROR_OK){	// something didn't work correctly - print error message
		sprintf_s(ErrorMsg, "Connect Failure: %s", IR.PGRError.GetDescription());
		AfxMessageBox(CA2W(ErrorMsg), MB_ICONSTOP);
	}
	else {
		IR.NumCameras = (IR.NumCameras > MAX_CAMERA) ? MAX_CAMERA : IR.NumCameras;
		for (i = 0; i < IR.NumCameras; i++) {
			// Get PGRGuid
			if (IR.busMgr.GetCameraFromIndex(i, &IR.prgGuid[i]) != PGRERROR_OK) {    // change to 1-i is cameras are swapped after powered up
				sprintf_s(ErrorMsg, "PGRGuID Failure: %s", IR.PGRError.GetDescription());
				AfxMessageBox(CA2W(ErrorMsg), MB_ICONSTOP);
			}
			IR.pgrCamera[i] = new Camera;
			if (IR.pgrCamera[i]->Connect(&IR.prgGuid[i]) != PGRERROR_OK) {
				sprintf_s(ErrorMsg, "PConnect Failure: %s", IR.PGRError.GetDescription());
				AfxMessageBox(CA2W(ErrorMsg), MB_ICONSTOP);
			}
			// Set video mode and frame rate
			if (IR.pgrCamera[i]->SetVideoModeAndFrameRate(VIDEO_FORMAT, CAMERA_FPS) != PGRERROR_OK) {
				sprintf_s(ErrorMsg, "Video Format Failure: %s", IR.PGRError.GetDescription());
				AfxMessageBox(CA2W(ErrorMsg), MB_ICONSTOP);
			}
			// Set all camera configuration parameters
			if (IR.pgrCamera[i]->SetConfiguration(&IR.cameraConfig) != PGRERROR_OK) {
				sprintf_s(ErrorMsg, "Set Configuration Failure: %s", IR.PGRError.GetDescription());
				AfxMessageBox(CA2W(ErrorMsg), MB_ICONSTOP);
			}
			// Sets the onePush option off, Turns the control on/off on, disables auto control.  These are applied to all properties.
			IR.cameraProperty.onePush = false;
			IR.cameraProperty.autoManualMode = false;
			IR.cameraProperty.absControl = true;
			IR.cameraProperty.onOff = true;
			// Set shutter sppeed
			IR.cameraProperty.type = SHUTTER;
			IR.cameraProperty.absValue = SHUTTER_SPEED;
			if (IR.pgrCamera[i]->SetProperty(&IR.cameraProperty, false) != PGRERROR_OK){
				sprintf_s(ErrorMsg, "Shutter Failure: %s", IR.PGRError.GetDescription());
				AfxMessageBox(CA2W(ErrorMsg), MB_ICONSTOP);
			}
			// Set gamma value
			IR.cameraProperty.type = GAMMA;
			IR.cameraProperty.absValue = 1.0;
			if (IR.pgrCamera[i]->SetProperty(&IR.cameraProperty, false) != PGRERROR_OK){
				sprintf_s(ErrorMsg, "Gamma Failure: %s", IR.PGRError.GetDescription());
				AfxMessageBox(CA2W(ErrorMsg), MB_ICONSTOP);
			}
			// Set sharpness value
			IR.cameraProperty.type = SHARPNESS;
			IR.cameraProperty.absControl = false;
			IR.cameraProperty.valueA = 2000;
			if (IR.pgrCamera[i]->SetProperty(&IR.cameraProperty, false) != PGRERROR_OK){
				sprintf_s(ErrorMsg, "Sharpness Failure: %s", IR.PGRError.GetDescription());
				AfxMessageBox(CA2W(ErrorMsg), MB_ICONSTOP);
			}
#ifdef  PTG_COLOR
			// Set white balance (R and B values)
			IR.cameraProperty = WHITE_BALANCE;
			IR.cameraProperty.absControl = false;
			IR.cameraProperty.onOff = true;
			IR.cameraProperty.valueA = WHITE_BALANCE_R;
			IR.cameraProperty.valueB = WHITE_BALANCE_B;
			if (IR.pgrCamera[i]->SetProperty(&IR.cameraProperty, false) != PGRERROR_OK){
				ErrorMsg.Format("White Balance Failure: %s", IR.PGRError.GetDescription());
				AfxMessageBox(ErrorMsg, MB_ICONSTOP);
			}
#endif
			// Set gain values (350 here gives 12.32dB, varies linearly)
			IR.cameraProperty = GAIN;
			IR.cameraProperty.absControl = false;
			IR.cameraProperty.onOff = true;
			IR.cameraProperty.valueA = GAIN_VALUE_A;
			IR.cameraProperty.valueB = GAIN_VALUE_B;
			if (IR.pgrCamera[i]->SetProperty(&IR.cameraProperty, false) != PGRERROR_OK){
				sprintf_s(ErrorMsg, "Gain Failure: %s", IR.PGRError.GetDescription());
				AfxMessageBox(CA2W(ErrorMsg), MB_ICONSTOP);
			}
			// Set trigger state
			IR.cameraTrigger.mode = 0;
			IR.cameraTrigger.onOff = TRIGGER_ON;
			IR.cameraTrigger.polarity = 0;
			IR.cameraTrigger.source = 0;
			IR.cameraTrigger.parameter = 0;
			if (IR.pgrCamera[i]->SetTriggerMode(&IR.cameraTrigger, false) != PGRERROR_OK){
				sprintf_s(ErrorMsg, "Trigger Failure: %s", IR.PGRError.GetDescription());
				AfxMessageBox(CA2W(ErrorMsg), MB_ICONSTOP);
			}
			IR.embeddedInfo[i].frameCounter.onOff = true;
			IR.embeddedInfo[i].timestamp.onOff = true;
			IR.pgrCamera[i]->SetEmbeddedImageInfo(&IR.embeddedInfo[i]);
			// Start Capture Individually
			if (IR.pgrCamera[i]->StartCapture() != PGRERROR_OK) {
				sprintf_s(ErrorMsg, "Start Capture Camera %d Failure: %s", i, IR.PGRError.GetDescription());
				AfxMessageBox(CA2W(ErrorMsg), MB_ICONSTOP);
			}
		}
		// Start Sync Capture (only need to do it with one camera)
		//		if (IR.pgrCamera[0]->StartSyncCapture(IR.NumCameras, (const Camera**)IR.pgrCamera, NULL, NULL) != PGRERROR_OK) {
		//				sprintf_s(ErrorMsg, "Start Sync Capture Failure: %s", IR.PGRError.GetDescription());
		//				AfxMessageBox(CA2W(ErrorMsg), MB_ICONSTOP );
		//		}
	}

#else
	IR.NumCameras = MAX_CAMERA;
#endif
	Rect R = Rect(0, 0, 640, 480);
	// create openCV image
	for (i = 0; i < IR.NumCameras; i++) {
#ifdef PTG_COLOR
		IR.AcqBuf[i].create(IR.DigSizeY, IR.DigSizeX, CV_8UC3);
		IR.DispBuf[i].create(IR.DigSizeY, IR.DigSizeX, CV_8UC3);
		IR.ProcBuf[i].create(IR.DigSizeY, IR.DigSizeX, CV_8UC3);
		for (j = 0; j < MAX_BUFFER; j++)
			IR.SaveBuf[i][j].create(IR.DigSizeY, IR.DigSizeX, CV_8UC3);
#else
		IR.AcqBuf[i].create(IR.DigSizeY, IR.DigSizeX, CV_8UC1);
		IR.DispBuf[i].create(IR.DigSizeY, IR.DigSizeX, CV_8UC1);
		IR.ProcBuf[i].create(IR.DigSizeY, IR.DigSizeX, CV_8UC1);
		for (j = 0; j < MAX_BUFFER; j++)
			IR.SaveBuf[i][j].create(IR.DigSizeY, IR.DigSizeX, CV_8UC1);
#endif
		IR.AcqPtr[i] = IR.AcqBuf[i].data;
		IR.DispROI[i] = IR.DispBuf[i](R);
		IR.ProcROI[i] = IR.ProcBuf[i](R);

		IR.OutBuf1[i].create(IR.DigSizeY, IR.DigSizeX, CV_8UC1);
		IR.OutBuf2[i].create(IR.DigSizeY, IR.DigSizeX, CV_8UC1);
		IR.OutROI1[i] = IR.OutBuf1[i](R);
		IR.OutROI2[i] = IR.OutBuf2[i](R);
		IR.DispBuf[i] = Scalar(0);
		IR.ProcBuf[i] = Scalar(0);
	}
	IR.from_to[0] = 0;
	IR.from_to[1] = 2;
	IR.from_to[2] = 1;
	IR.from_to[3] = 1;
	IR.from_to[4] = 2;
	IR.from_to[5] = 0;
	QSStartThread();
}

void CTCSys::QSSysFree()
{
	QSStopThread(); // Move to below PTGREY if on Windows Vista
#ifdef PTGREY
	for (int i = 0; i < IR.NumCameras; i++) {
		if (IR.pgrCamera[i]) {
			IR.pgrCamera[i]->StopCapture();
			IR.pgrCamera[i]->Disconnect();
			delete IR.pgrCamera[i];
		}
	}
#endif
}

void CTCSys::initBitmapStruct(long iCols, long iRows)
{
	m_bitmapInfo.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
	m_bitmapInfo.bmiHeader.biPlanes = 1;
	m_bitmapInfo.bmiHeader.biCompression = BI_RGB;
	m_bitmapInfo.bmiHeader.biXPelsPerMeter = 120;
	m_bitmapInfo.bmiHeader.biYPelsPerMeter = 120;
	m_bitmapInfo.bmiHeader.biClrUsed = 0;
	m_bitmapInfo.bmiHeader.biClrImportant = 0;
	m_bitmapInfo.bmiHeader.biWidth = iCols;
	m_bitmapInfo.bmiHeader.biHeight = -iRows;
	m_bitmapInfo.bmiHeader.biBitCount = 24;
	m_bitmapInfo.bmiHeader.biSizeImage =
		m_bitmapInfo.bmiHeader.biWidth * m_bitmapInfo.bmiHeader.biHeight * (m_bitmapInfo.bmiHeader.biBitCount / 8);
}

void CTCSys::QSSysDisplayImage()
{
	for (int i = 0; i < 2; i++) {
		::SetDIBitsToDevice(
			ImageDC[i]->GetSafeHdc(), 1, 1,
			m_bitmapInfo.bmiHeader.biWidth,
			::abs(m_bitmapInfo.bmiHeader.biHeight),
			0, 0, 0,
			::abs(m_bitmapInfo.bmiHeader.biHeight),
			IR.DispBuf[i].data,
			&m_bitmapInfo, DIB_RGB_COLORS);
	}
}

#ifdef PTGREY
void CTCSys::QSSysConvertToOpenCV(Mat* openCV_image, Image PGR_image)
{
	openCV_image->data = PGR_image.GetData();	// Pointer to image data
	openCV_image->cols = PGR_image.GetCols();	// Image width in pixels
	openCV_image->rows = PGR_image.GetRows();	// Image height in pixels
	openCV_image->step = PGR_image.GetStride(); // Size of aligned image row in bytes
}
#endif
