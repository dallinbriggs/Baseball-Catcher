#include <windows.h>
#ifndef TCHW_H
#define TCHW_H

#define PTGREY
//#define PTG_COLOR
#define DEVELOPMENT

#ifdef PTGREY
#include "FlyCapture2.h"
using namespace FlyCapture2;
#endif

#include "RoboteqWrapper.h"
#include "opencv2/opencv.hpp"	// openCV headers
using namespace std;
using namespace cv;

#define APP_DIRECTORY		"C:\\Projects\\BaseballCatcher - 10\\"
#define	SHUTTER_SPEED		12.0					// Shutter speed in mSec
#define WHITE_BALANCE_R		560;
#define WHITE_BALANCE_B		740;
#define GAIN_VALUE_A		200;
#define GAIN_VALUE_B		0;
#ifdef PTG_COLOR
#define VIDEO_FORMAT		VIDEOMODE_640x480RGB	// VIDEOMODE_640x480RGB for color or VIDEOMODE_640x480Y8 for B/W
#define	CAMERA_FPS			FRAMERATE_30			// on 30 FPS for color camera
#else
#define VIDEO_FORMAT		VIDEOMODE_640x480Y8	// VIDEOMODE_640x480RGB for color or VIDEOMODE_640x480Y8 for B/W
#define	CAMERA_FPS			FRAMERATE_60			// or FRAMERATE_30
#endif
#define TRIGGER_ON			0						// Is the camera on an external trigger or not

#define MAX_CAMERA		2
#define MAX_BUFFER		100

#define SQUARE(a) ((a)*(a))
#define DISTANCE_SQ(x1,y1,x2,y2) (SQUARE((x1)-(x2))+SQUARE((y1)-(y2)))

#if defined(DEVELOPMENT) || defined(XLPORT)
#define OPENF(txt) fopen_s(&m_fp, txt,"wt")
#define	FLUSHF _flushall()
#define CLOSEF fclose(m_fp)
#define DECLARE_F FILE *m_fp;
#else
#define OPENF(txt)
#define	FLUSHF
#define CLOSEF
#define DECLARE_F
#endif

#ifdef DEVELOPMENT
#define FTRACE(txt) fprintf(m_fp,txt)
#define FTDTRACE(txt) fprintf(pt->m_fp,txt)
#define FGTDTRACE(txt) fprintf(TP->m_fp,txt)
#else
#define FTRACE(txt)
#define FTDTRACE(txt)
#define FGTDTRACE(txt)
#endif

#ifdef XLPORT
#define XLTRACE(txt) fprintf(m_fp,txt)
#else
#define XLTRACE(txt)
#endif

#define CMA ,

typedef struct
{
	int			DigSizeX;
	int			DigSizeY;
	int			NumCameras;
	int			from_to[6];

	int			FrameID;
	int			CaptureDelay;
	int			PlayDelay;
	BOOL		Acquisition;
	BOOL		UpdateImage;
	BOOL		CaptureSequence;
	BOOL		DisplaySequence;
	BOOL		CatchBall;
	// OpenCV Stuff
	Mat			AcqBuf[MAX_CAMERA];				// Processing Buffer in OpenCV
	Mat			DispBuf[MAX_CAMERA];			// Display Buffer in OpenCV
	Mat			DispROI[MAX_CAMERA];			// Display Buffer in OpenCV
	Mat			SaveBuf[MAX_CAMERA][MAX_BUFFER];// Save Buffer in OpenCV
	Mat			ProcBuf[MAX_CAMERA];			// process Buffer in OpenCV
	Mat			ProcROI[MAX_CAMERA];			// process Buffer in OpenCV
	Mat			OutBuf1[MAX_CAMERA];			// Binary Process Buffer in OpenCV
	Mat			OutROI1[MAX_CAMERA];			// Binary Process Buffer in OpenCV
	Mat			OutBuf2[MAX_CAMERA];			// Binary Process Buffer in OpenCV
	Mat			OutROI2[MAX_CAMERA];			// Binary Process Buffer in OpenCV
	uchar*		AcqPtr[MAX_CAMERA];				// imageData pointer to be saved for later to release imageData
	Mat			OutBuf[3];						// vector of Mats for display


#ifdef PTGREY
	FlyCapture2::Error PGRError;				// PGR error object used for PGR error codes returned from PGR functions.
	FC2Config	cameraConfig;					// Camera Configuration
	Property	cameraProperty;					// current camera property being queried or modified
	TriggerMode	cameraTrigger;					// Camera trigger mode settings
	BusManager	busMgr;
	PGRGuid		prgGuid[MAX_CAMERA];
	Camera*		pgrCamera[MAX_CAMERA];
	Image		PtGBuf[MAX_CAMERA];			// Array of most current images
	EmbeddedImageInfo	embeddedInfo[MAX_CAMERA];
	ImageMetadata		metaData[MAX_CAMERA];
#endif
} ImagingResources;		// Imaging Resources

class CTCSys : public RoboteqWrapper
{
protected:
	CWnd*	MainWnd;
	CWnd*	ParentWnd[MAX_CAMERA];	// Image Display Window
	CDC*	ImageDC[MAX_CAMERA];
	BOOL	EventEndProcess;		// Ends Process Program Execution
	HANDLE	QSProcessThreadHandle;
	HANDLE	QSProcessThreadHandleID;
	HANDLE	QSProcessEvent;
	BOOL	EventEndMove;			// Ends Move Program Execution
	HANDLE	QSMoveThreadHandle;
	HANDLE	QSMoveThreadHandleID;
	HANDLE	QSMoveEvent;
	DWORD	ExitCode;
	BITMAPINFO  m_bitmapInfo;

	CTCSys(void);
	~CTCSys(void);
	void	QSStartThread();
	void	QSStopThread();
	friend	static long	QSProcessThreadFunc(CTCSys *QS);
	friend	static long	QSMoveThreadFunc(CTCSys *QS);
	void	initBitmapStruct(long iCols, long iRows);
	void	QSSysInit();
	void	QSSysFree();
	void	QSSysDisplayImage();
#ifdef PTGREY
	void	QSSysConvertToOpenCV(Mat* openCV_image, Image PGR_image);
#endif

public:
	static  ImagingResources IR;
	RECT	ImageRect[MAX_CAMERA];
	CRect	DispRect[MAX_CAMERA];
	CRect	SelectRect;
	CDC*	SelectDC;			// Drawing Device Context
	CPen    DrawPen;			// CPEN variables
	CPen    ClearPen;			// CPEN variables
	DECLARE_F

	// Our parameters
	// int x_left = 333;
	// int y_left = 88;
	// int x_right = 233;
	// int y_right = 88;
	// int width = 70;
	// int height = 70;
	// Rect rectangle_left = Rect(x_left, y_left, width, height);
	// Rect rectangle_right = Rect(x_right, y_right, width, height);
	// Rect rectangle_small[MAX_CAMERA] = {rectangle_left, rectangle_right};
	//
	//
	// Mat R[MAX_CAMERA];
	// Mat P[MAX_CAMERA];
	// Mat Q;
	// Mat distCoeffs[MAX_CAMERA];
	// Mat distCoeffs_right;
	// Mat cameraMatrix[MAX_CAMERA];
	// Mat cameraMatrix_right;
	//
	// Mat image_left;
	// Mat image_right;
	// Mat image_left_color;
	// Mat image_right_color;
	// Mat roi_first;
	// Mat frame_thresh_left;
	// Mat frame_thresh_right;
	// Mat motion_right;
	// Mat motion_left;
	// Mat first_frame[MAX_CAMERA];
	// Mat roi_previous;
	// Mat roi_current;
	// Mat child_first[MAX_CAMERA];
	// Mat image_first;
	// Mat motion;
	// Mat image_thresh;
	// vector<vector<Point>> contours;
	// vector<Vec4i>> hierarchy;
	// vector<Point2f> point_disp;
	// Point3f corner_diff;
	// Point3f perspective_left;
	// vector<Point3f> ball_positions;

	Mat image_left;
	Mat image_right;
	Mat image_left_color;
	Mat image_right_color;
	Mat roi_first_left;
	Mat roi_first_right;
	Mat frame_thresh_left;
	Mat frame_thresh_right;
	Mat motion_right;
	Mat motion_left;
	Mat first_frame_left;
	Mat first_frame_right;
	Mat roi_current_left;
	Mat roi_current_right;
	Mat roi_previous_left;
	Mat roi_previous_right;
	string filename_left;
	string filename_right;
	string header;
	string tail;
	int x_left_home = 323;
	int y_left_home = 88;
	int x_right_home = 255;
	int y_right_home = 88;
	int x_left;
	int y_left;
	int x_right;
	int y_right;
	int width = 70;
	int height = 70;
	Rect rectangle_left = Rect(x_left_home, y_left_home, width, height);
	Rect rectangle_right = Rect(x_right_home, y_right_home, width, height);

	Mat image_left_test;


	vector<Point3f> corners_diff_left;
	vector<Point3f> corners_diff_right;
	vector<Point3f> perspective_left;
	vector<Point3f> ball_left;
	vector<Point3f> ball_right;
	vector<Point3f> perspective_right;
	vector<Point2f> point_left, point_right;

	Mat R1, P1, R2, P2, Q;
	Mat distCoeffs_left;
	Mat distCoeffs_right;
	Mat cameraMatrix_left;
	Mat cameraMatrix_right;


	vector<vector<Point> > contours_left;
	vector<vector<Point> > contours_right;
	vector<Vec4i> hierarchy_left;
	vector<Vec4i> hierarchy_right;

	int largest_area_left = 0;
	int largest_area_right = 0;
	int area_left_iter = 0;
	int area_right_iter = 0;

	double area_left = 0;
	double area_right = 0;

	int x_offset = 10;
    int y_offset = -36;
    int z_offset = -5;

	Point3f leftToCatcher;
	vector<Point3f> catcher_points;


};
#endif
