/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - User Viewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.										   *
*																			   *
*   Integration with Mindstorm - Micha³ 'kwiateusz' Kwiatek					   *
*	Copyright (C) 2014 Ko³o naukowe robotyki UWM Olsztyn					   *
*																			   *
*   Amendment of code and steering extension -								   *
*	Adam Augustyniak, £ukasz ¯mudziñski										   *
*	Copyright (C) 2014-15 Ko³o naukowe robotyki UWM Olsztyn					   *
*																			   *
*******************************************************************************/

#pragma region Definitions
#include "NXT++.h"
#include "Viewer.h"
#include<map>

#if (defined _WIN32)
	#define PRIu64 "llu"
#else
	#define __STDC_FORMAT_MACROS
	#include <inttypes.h>
#endif

#if (ONI_PLATFORM == ONI_PLATFORM_MACOSX)
        #include <GLUT/glut.h>
#else
        #include <GL/glut.h>
#endif

#include <NiteSampleUtilities.h>

#define GL_WIN_SIZE_X							1024
#define GL_WIN_SIZE_Y							768
#define TEXTURE_SIZE							512
#define DEFAULT_DISPLAY_MODE					DISPLAY_MODE_DEPTH

#define MIN_NUM_CHUNKS(data_size, chunk_size)	((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size)	(MIN_NUM_CHUNKS(data_size, chunk_size) * (chunk_size))

#define USER_MESSAGE(msg) \
{\
	sprintf_s(g_userStatusLabels[user.getId()], "%s", msg);\
	printf("[%08" PRIu64 "] User #%d:\t%s\n", ts, user.getId(), msg);\
}

// Max camera users
#define MAX_USERS 5

using namespace std;
#pragma endregion
#pragma region Constants
// time to hold in pose to exit program. In milliseconds.
const int g_poseTimeoutToExit = 2000;

// Steering constants
const float precisionX = 100;
const float precisionY = 50;
const int speed = 40;
#pragma endregion
#pragma region Variables
// NXT variables
Comm::NXTComm comm;
bool mindstrom_connection_open = false;
int steering_mode = 0; // User selected steering method

// Skeleton variables
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};
bool g_drawSkeleton = true;
bool g_drawCenterOfMass = false;
bool g_drawStatusLabel = true;
bool g_drawBoundingBox = false;
bool g_drawBackground = true;
bool g_drawDepth = true;
bool g_drawFrameId = false;
bool g_visibleUsers[MAX_USERS] = {false};

// Camera variables
int colorCount = 3; // Number of colors
float Colors[][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1}};
int g_nXRes = 0, g_nYRes = 0;

// General variables
char g_userStatusLabels[MAX_USERS][100] = {{0}};
char g_generalMessage[100] = {0};
SampleViewer* SampleViewer::ms_self = NULL;

#pragma endregion

#pragma region Constructor
SampleViewer::SampleViewer(const char* strSampleName) : m_poseUser(0)
{
	ms_self = this;
	strncpy_s(m_strSampleName, strSampleName, ONI_MAX_STR);
	m_pUserTracker = new nite::UserTracker;
}

void SampleViewer::Finalize()
{
	delete m_pUserTracker;
	nite::NiTE::shutdown();
	openni::OpenNI::shutdown();
}

openni::Status SampleViewer::Run()	//Does not return
{
	glutMainLoop();
	return openni::STATUS_OK;
}

void updateUserState(const nite::UserData& user, uint64_t ts)
{
	if (user.isNew())
	{
		USER_MESSAGE("New");
	}
	else if (user.isVisible() && !g_visibleUsers[user.getId()])
		printf("[%08" PRIu64 "] User #%d:\tVisible\n", ts, user.getId());
	else if (!user.isVisible() && g_visibleUsers[user.getId()])
		printf("[%08" PRIu64 "] User #%d:\tOut of Scene\n", ts, user.getId());
	else if (user.isLost())
	{
		USER_MESSAGE("Lost");
	}
	g_visibleUsers[user.getId()] = user.isVisible();

	if(g_skeletonStates[user.getId()] != user.getSkeleton().getState())
	{
		switch(g_skeletonStates[user.getId()] = user.getSkeleton().getState())
		{
			case nite::SKELETON_NONE:
				USER_MESSAGE("Stopped tracking.")
				break;
			case nite::SKELETON_CALIBRATING:
				USER_MESSAGE("Calibrating...")
				break;
			case nite::SKELETON_TRACKED:
				USER_MESSAGE("Tracking!")
				break;
			case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
			case nite::SKELETON_CALIBRATION_ERROR_HANDS:
			case nite::SKELETON_CALIBRATION_ERROR_LEGS:
			case nite::SKELETON_CALIBRATION_ERROR_HEAD:				
			case nite::SKELETON_CALIBRATION_ERROR_TORSO:				
				USER_MESSAGE("Calibration Failed... :|")
				break;
		}
	}
}

openni::Status SampleViewer::Init(int argc, char **argv)
{
	m_pTexMap = NULL;

	#pragma region Camera initialization
	printf("Initialization, please wait...\n");	
	openni::Status rc = openni::OpenNI::initialize();

	// Check if camera connection is established
	if (rc != openni::STATUS_OK)
	{
		printf("Failed to initialize OpenNI\n%s\n", openni::OpenNI::getExtendedError());
		return rc;
	}
	
	const char* deviceUri = openni::ANY_DEVICE;
	for (int i = 1; i < argc-1; ++i)
	{
		if (strcmp(argv[i], "-device") == 0)
		{
			deviceUri = argv[i+1];
			break;
		}
	}

	// Check if selected camera connection is established
	rc = m_device.open(deviceUri);
	if (rc != openni::STATUS_OK)
	{
		printf("Failed to open device\n%s\n", openni::OpenNI::getExtendedError());
		return rc;
	}

	nite::NiTE::initialize();

	if (m_pUserTracker->create(&m_device) != nite::STATUS_OK)
	{
		return openni::STATUS_ERROR;
	}
	#pragma endregion
	#pragma region Mindstorm initialization	
	printf("Connecting to Mindstorm...\n");
	if(NXT::OpenBT(&comm)) //initialize the NXT and continue if it succeeds
	{
		NXT::StartProgram(&comm,"program1");
		mindstrom_connection_open = true;
	} 
	else 
	{
		printf("Can't connect to Mindstorm");
		return openni::STATUS_ERROR;
	}
	#pragma endregion
	#pragma region Menu
	system("cls");
	printf("Please select steering mode:\n");
	printf("0. Simple steering\n");
	printf("1. Depth steering\n");
	printf("2. Steering with clutches support\n");
	printf("Enter number: ");
	cin >> steering_mode; // Get user value for steering
	#pragma endregion

	return InitOpenGL(argc, argv);
}

SampleViewer::~SampleViewer()
{
	Finalize();

	delete[] m_pTexMap;
	ms_self = NULL;

	//Mindstorm end of program
	if(mindstrom_connection_open)
	{
		NXT::Motor::Stop(&comm, OUT_B, true);
		NXT::Motor::Stop(&comm, OUT_C, true);
		NXT::StopProgram(&comm);	
		NXT::Close(&comm); //close communication with NXT
	}
}
#pragma endregion
#pragma region Methods
#pragma region Skeleton drawing
#ifndef USE_GLES
void glPrintString(void *font, const char *str)
{
	int i,l = (int)strlen(str);

	for(i=0; i<l; i++)
	{   
		glutBitmapCharacter(font,*str++);
	}   
}
#endif

void DrawStatusLabel(nite::UserTracker* pUserTracker, const nite::UserData& user)
{
	int color = user.getId() % colorCount;
	glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);

	float x,y;
	pUserTracker->convertJointCoordinatesToDepth(user.getCenterOfMass().x, user.getCenterOfMass().y, user.getCenterOfMass().z, &x, &y);
	x *= GL_WIN_SIZE_X/(float)g_nXRes;
	y *= GL_WIN_SIZE_Y/(float)g_nYRes;
	char *msg = g_userStatusLabels[user.getId()];
	glRasterPos2i(x-((strlen(msg)/2)*8),y);
	glPrintString(GLUT_BITMAP_HELVETICA_18, msg);
}

void DrawFrameId(int frameId)
{
	char buffer[80] = "";
	sprintf_s(buffer, "%d", frameId);
	glColor3f(1.0f, 0.0f, 0.0f);
	glRasterPos2i(20, 20);
	glPrintString(GLUT_BITMAP_HELVETICA_18, buffer);
}

void DrawCenterOfMass(nite::UserTracker* pUserTracker, const nite::UserData& user)
{
	glColor3f(1.0f, 1.0f, 1.0f);

	float coordinates[3] = {0};

	pUserTracker->convertJointCoordinatesToDepth(user.getCenterOfMass().x, user.getCenterOfMass().y, user.getCenterOfMass().z, &coordinates[0], &coordinates[1]);

	coordinates[0] *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[1] *= GL_WIN_SIZE_Y/(float)g_nYRes;
	glPointSize(8);
	glVertexPointer(3, GL_FLOAT, 0, coordinates);
	glDrawArrays(GL_POINTS, 0, 1);

}
void DrawBoundingBox(const nite::UserData& user)
{
	glColor3f(1.0f, 1.0f, 1.0f);

	float coordinates[] =
	{
		user.getBoundingBox().max.x, user.getBoundingBox().max.y, 0,
		user.getBoundingBox().max.x, user.getBoundingBox().min.y, 0,
		user.getBoundingBox().min.x, user.getBoundingBox().min.y, 0,
		user.getBoundingBox().min.x, user.getBoundingBox().max.y, 0,
	};
	coordinates[0]  *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[1]  *= GL_WIN_SIZE_Y/(float)g_nYRes;
	coordinates[3]  *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[4]  *= GL_WIN_SIZE_Y/(float)g_nYRes;
	coordinates[6]  *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[7]  *= GL_WIN_SIZE_Y/(float)g_nYRes;
	coordinates[9]  *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[10] *= GL_WIN_SIZE_Y/(float)g_nYRes;

	glPointSize(2);
	glVertexPointer(3, GL_FLOAT, 0, coordinates);
	glDrawArrays(GL_LINE_LOOP, 0, 4);

}
void DrawLimb(nite::UserTracker* pUserTracker, const nite::SkeletonJoint& joint1, const nite::SkeletonJoint& joint2, int color)
{
	float coordinates[6] = {0};
	pUserTracker->convertJointCoordinatesToDepth(joint1.getPosition().x, joint1.getPosition().y, joint1.getPosition().z, &coordinates[0], &coordinates[1]);
	pUserTracker->convertJointCoordinatesToDepth(joint2.getPosition().x, joint2.getPosition().y, joint2.getPosition().z, &coordinates[3], &coordinates[4]);

	coordinates[0] *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[1] *= GL_WIN_SIZE_Y/(float)g_nYRes;
	coordinates[3] *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[4] *= GL_WIN_SIZE_Y/(float)g_nYRes;

	if (joint1.getPositionConfidence() == 1 && joint2.getPositionConfidence() == 1)
	{
		glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);
	}
	else if (joint1.getPositionConfidence() < 0.5f || joint2.getPositionConfidence() < 0.5f)
	{
		return;
	}
	else
	{
		glColor3f(.5, .5, .5);
	}
	glPointSize(2);
	glVertexPointer(3, GL_FLOAT, 0, coordinates);
	glDrawArrays(GL_LINES, 0, 2);

	glPointSize(10);
	if (joint1.getPositionConfidence() == 1)
	{
		glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);
	}
	else
	{
		glColor3f(.5, .5, .5);
	}
	glVertexPointer(3, GL_FLOAT, 0, coordinates);
	glDrawArrays(GL_POINTS, 0, 1);

	if (joint2.getPositionConfidence() == 1)
	{
		glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);
	}
	else
	{
		glColor3f(.5, .5, .5);
	}
	glVertexPointer(3, GL_FLOAT, 0, coordinates+3);
	glDrawArrays(GL_POINTS, 0, 1);
}
void DrawSkeleton(nite::UserTracker* pUserTracker, const nite::UserData& userData)
{
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_HEAD), userData.getSkeleton().getJoint(nite::JOINT_NECK), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW), userData.getSkeleton().getJoint(nite::JOINT_LEFT_HAND), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), userData.getId() % colorCount);


	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP), userData.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE), userData.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT), userData.getId() % colorCount);


	//sprintf_s(g_generalMessage, "test");
}
#pragma endregion
#pragma region Main function
#pragma region Steering methods
void RunSteeringMethodOne(map<string, map<char, float>> positions)
{
	if(positions["right_hand"]['y'] > positions["right_shoulder"]['y'])
	{
		if(positions["right_hand"]['x'] > (positions["right_shoulder"]['x'] + precisionX)) 
		{
			NXT::Motor::SetReverse(&comm, OUT_B, speed);
			NXT::Motor::SetForward(&comm, OUT_C, speed);
		} 
		else if(positions["right_hand"]['x'] < (positions["right_shoulder"]['x'] - precisionX)) 
		{
			NXT::Motor::SetForward(&comm, OUT_B, speed);
			NXT::Motor::SetReverse(&comm, OUT_C, speed);
		}  
		else 
		{
			NXT::Motor::SetForward(&comm, OUT_B, speed);
			NXT::Motor::SetForward(&comm, OUT_C, speed);
		}
	} 
	else if(positions["left_hand"]['y'] > positions["left_shoulder"]['y'])
	{
		NXT::Motor::SetReverse(&comm, OUT_B, speed);
		NXT::Motor::SetReverse(&comm, OUT_C, speed); 
	} 
	else 
	{
		NXT::Motor::Stop(&comm, OUT_B, true);
		NXT::Motor::Stop(&comm, OUT_C, true);
	}
}

void RunSteeringMethodTwo(map<string, map<char, float>> positions)
{
	if(positions["left_hand"]['z'] > positions["right_hand"]['z'] + precisionX 
		&& positions["right_hand"]['y'] > positions["torso"]['y'] + precisionY)
	{
		NXT::Motor::SetForward(&comm, OUT_B, speed);
		NXT::Motor::SetForward(&comm, OUT_C, speed);
	}
						
	else if(positions["right_hand"]['z'] > positions["left_hand"]['z'] + precisionX 
		&& positions["left_hand"]['y'] > positions["torso"]['y'] + precisionY)
	{
		NXT::Motor::SetReverse(&comm, OUT_B, speed);
		NXT::Motor::SetReverse(&comm, OUT_C, speed);
	}
	else if(positions["right_hand"]['x'] > positions["right_shoulder"]['x'] + precisionX 
		&& positions["right_hand"]['y'] < positions["right_shoulder"]['y'] - precisionY 
		&& positions["right_hand"]['y'] > positions["left_hip"]['y'] + precisionY)
	{
		NXT::Motor::SetReverse(&comm, OUT_B, speed);
		NXT::Motor::SetForward(&comm, OUT_C, speed);
	}
	else if(positions["left_hand"]['x'] < positions["left_shoulder"]['x'] - precisionX 
		&& positions["left_hand"]['y'] < positions["left_shoulder"]['y'] - precisionY 
		&& positions["left_hand"]['y'] > positions["left_hip"]['y'] + precisionY)
	{
		NXT::Motor::SetForward(&comm, OUT_B, speed);
		NXT::Motor::SetReverse(&comm, OUT_C, speed);
	}
	else
	{
		NXT::Motor::Stop(&comm, OUT_B, true);
		NXT::Motor::Stop(&comm, OUT_C, true);
	}
}

void RunSteeringMethodThree(map<string, map<char, float>> positions)
{
	if(positions["right_hand"]['y'] > positions["right_shoulder"]['y'])
	{
		NXT::Motor::SetForward(&comm, OUT_A, 10);
	} 
	else if(positions["left_hand"]['y'] > (positions["left_shoulder"]['y'] - precisionX)) 
	{
		NXT::Motor::SetReverse(&comm, OUT_A, 10);
	}  
	else if(positions["right_hand"]['x'] > positions["right_hip"]['x']+100)
	{
		NXT::Motor::SetForward(&comm, OUT_B, 10);
		NXT::Motor::SetForward(&comm, OUT_C, 10);
	}
	else if(positions["left_hand"]['x'] + 100 < positions["left_hip"]['x'])
	{
		NXT::Motor::SetReverse(&comm, OUT_B, 10);
		NXT::Motor::SetReverse(&comm, OUT_C, 10);
	}
	else 
	{
		NXT::Motor::Stop(&comm, OUT_A, true);
		NXT::Motor::Stop(&comm, OUT_B, true);
		NXT::Motor::Stop(&comm, OUT_C, true);
	}
}

#pragma endregion
void SampleViewer::Display()
{
	nite::UserTrackerFrameRef userTrackerFrame;
	openni::VideoFrameRef depthFrame;
	nite::Status rc = m_pUserTracker->readFrame(&userTrackerFrame);
	if (rc != nite::STATUS_OK)
	{
		printf("GetNextData failed\n");
		return;
	}

	depthFrame = userTrackerFrame.getDepthFrame();

	if (m_pTexMap == NULL)
	{
		// Texture map init
		m_nTexMapX = MIN_CHUNKS_SIZE(depthFrame.getVideoMode().getResolutionX(), TEXTURE_SIZE);
		m_nTexMapY = MIN_CHUNKS_SIZE(depthFrame.getVideoMode().getResolutionY(), TEXTURE_SIZE);
		m_pTexMap = new openni::RGB888Pixel[m_nTexMapX * m_nTexMapY];
	}

	const nite::UserMap& userLabels = userTrackerFrame.getUserMap();

	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, GL_WIN_SIZE_X, GL_WIN_SIZE_Y, 0, -1.0, 1.0);

	if (depthFrame.isValid() && g_drawDepth)
	{
		calculateHistogram(m_pDepthHist, MAX_DEPTH, depthFrame);
	}

	memset(m_pTexMap, 0, m_nTexMapX*m_nTexMapY*sizeof(openni::RGB888Pixel));

	float factor[3] = {1, 1, 1};
	// check if we need to draw depth frame to texture
	if (depthFrame.isValid() && g_drawDepth)
	{
		const nite::UserId* pLabels = userLabels.getPixels();

		const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)depthFrame.getData();
		openni::RGB888Pixel* pTexRow = m_pTexMap + depthFrame.getCropOriginY() * m_nTexMapX;
		int rowSize = depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel);

		for (int y = 0; y < depthFrame.getHeight(); ++y)
		{
			const openni::DepthPixel* pDepth = pDepthRow;
			openni::RGB888Pixel* pTex = pTexRow + depthFrame.getCropOriginX();

			for (int x = 0; x < depthFrame.getWidth(); ++x, ++pDepth, ++pTex, ++pLabels)
			{
				if (*pDepth != 0)
				{
					if (*pLabels == 0)
					{
						if (!g_drawBackground)
						{
							factor[0] = factor[1] = factor[2] = 0;

						}
						else
						{
							factor[0] = Colors[colorCount][0];
							factor[1] = Colors[colorCount][1];
							factor[2] = Colors[colorCount][2];
						}
					}
					else
					{
						factor[0] = Colors[*pLabels % colorCount][0];
						factor[1] = Colors[*pLabels % colorCount][1];
						factor[2] = Colors[*pLabels % colorCount][2];
					}
//					// Add debug lines - every 10cm
// 					else if ((*pDepth / 10) % 10 == 0)
// 					{
// 						factor[0] = factor[2] = 0;
// 					}

					int nHistValue = m_pDepthHist[*pDepth];
					pTex->r = nHistValue*factor[0];
					pTex->g = nHistValue*factor[1];
					pTex->b = nHistValue*factor[2];

					factor[0] = factor[1] = factor[2] = 1;
				}
			}

			pDepthRow += rowSize;
			pTexRow += m_nTexMapX;
		}
	}

	glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_nTexMapX, m_nTexMapY, 0, GL_RGB, GL_UNSIGNED_BYTE, m_pTexMap);

	// Display the OpenGL texture map
	glColor4f(1,1,1,1);

	glEnable(GL_TEXTURE_2D);
	glBegin(GL_QUADS);

	g_nXRes = depthFrame.getVideoMode().getResolutionX();
	g_nYRes = depthFrame.getVideoMode().getResolutionY();

	// upper left
	glTexCoord2f(0, 0);
	glVertex2f(0, 0);
	// upper right
	glTexCoord2f((float)g_nXRes/(float)m_nTexMapX, 0);
	glVertex2f(GL_WIN_SIZE_X, 0);
	// bottom right
	glTexCoord2f((float)g_nXRes/(float)m_nTexMapX, (float)g_nYRes/(float)m_nTexMapY);
	glVertex2f(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	// bottom left
	glTexCoord2f(0, (float)g_nYRes/(float)m_nTexMapY);
	glVertex2f(0, GL_WIN_SIZE_Y);

	glEnd();
	glDisable(GL_TEXTURE_2D);

	const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
	for (int i = 0; i < users.getSize(); ++i)
	{
		const nite::UserData& user = users[i];

		updateUserState(user, userTrackerFrame.getTimestamp());
		if (user.isNew())
		{
			m_pUserTracker->startSkeletonTracking(user.getId());
			m_pUserTracker->startPoseDetection(user.getId(), nite::POSE_CROSSED_HANDS);
		}
		else if (!user.isLost())
		{
			if (g_drawStatusLabel)
			{
				DrawStatusLabel(m_pUserTracker, user);
			}
			if (g_drawCenterOfMass)
			{
				DrawCenterOfMass(m_pUserTracker, user);
			}
			if (g_drawBoundingBox)
			{
				DrawBoundingBox(user);
			}

			if (users[i].getSkeleton().getState() == nite::SKELETON_TRACKED && g_drawSkeleton)
			{
				DrawSkeleton(m_pUserTracker, user);
			}

			//Mindstorm main program
			if (users[i].getSkeleton().getState() == nite::SKELETON_TRACKED && mindstrom_connection_open && user.getId() == 1) //&& user.getId() == 1
			{	
				#pragma region Assigning joints
				// Steering variables
				map<string, const nite::SkeletonJoint> joints;
				map<string, map<char, float>> positions;
				
				// Reading joint values
				joints.insert(pair<string, const nite::SkeletonJoint>("right_hand", user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND)));
				joints.insert(pair<string, const nite::SkeletonJoint>("right_shoulder", user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER)));
				joints.insert(pair<string, const nite::SkeletonJoint>("left_hand", user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND)));
				joints.insert(pair<string, const nite::SkeletonJoint>("left_shoulder", user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER)));
				joints.insert(pair<string, const nite::SkeletonJoint>("right_elbow", user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW)));
				joints.insert(pair<string, const nite::SkeletonJoint>("torso", user.getSkeleton().getJoint(nite::JOINT_TORSO)));
				joints.insert(pair<string, const nite::SkeletonJoint>("left_hip", user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP)));

				if (joints["right_hand"].getPositionConfidence() > .5){
				positions["right_hand"]['x'] = joints["right_hand"].getPosition().x;
				positions["right_hand"]['y'] = joints["right_hand"].getPosition().y;
				positions["right_hand"]['z'] = joints["right_hand"].getPosition().z;
				}
				if (joints["right_shoulder"].getPositionConfidence() > .5){
				positions["right_shoulder"]['x'] = joints["right_shoulder"].getPosition().x;
				positions["right_shoulder"]['y'] = joints["right_shoulder"].getPosition().y;
				positions["right_shoulder"]['z'] = joints["right_shoulder"].getPosition().z;
				}
				if (joints["left_hand"].getPositionConfidence() > .5){
				positions["left_hand"]['x'] = joints["left_hand"].getPosition().x;
				positions["left_hand"]['y'] = joints["left_hand"].getPosition().y;
				positions["left_hand"]['z'] = joints["left_hand"].getPosition().z;
				}
				if (joints["left_shoulder"].getPositionConfidence() > .5){
				positions["left_shoulder"]['x'] = joints["left_shoulder"].getPosition().x;
				positions["left_shoulder"]['y'] = joints["left_shoulder"].getPosition().y;
				positions["left_shoulder"]['z'] = joints["left_shoulder"].getPosition().z;
				}
				if (joints["right_elbow"].getPositionConfidence() > .5)
				{
				positions["right_elbow"]['x'] = joints["right_elbow"].getPosition().x;
				positions["right_elbow"]['y'] = joints["right_elbow"].getPosition().y;
				positions["right_elbow"]['z'] = joints["right_elbow"].getPosition().z;
				}
				if (joints["left_elbow"].getPositionConfidence() > .5)
				{
				positions["left_elbow"]['x'] = joints["left_elbow"].getPosition().x;
				positions["left_elbow"]['y'] = joints["left_elbow"].getPosition().y;
				positions["left_elbow"]['z'] = joints["left_elbow"].getPosition().z;
				}
				if (joints["torso"].getPositionConfidence() > .5)
				{
				positions["torso"]['x'] = joints["torso"].getPosition().x;
				positions["torso"]['y'] = joints["torso"].getPosition().y;
				positions["torso"]['z'] = joints["torso"].getPosition().z;
				}
				if (joints["left_hip"].getPositionConfidence() > .5)
				{
				positions["left_hip"]['x'] = joints["left_hip"].getPosition().x;
				positions["left_hip"]['y'] = joints["left_hip"].getPosition().y;
				positions["left_hip"]['z'] = joints["left_hip"].getPosition().z;
				}	
				#pragma endregion				
				switch (steering_mode)
				{
					case 0:
						RunSteeringMethodOne(positions);
						break;
					case 1:
						RunSteeringMethodTwo(positions);						
						break;
					case 2:
						RunSteeringMethodThree(positions);						
						break;
				}
			}
		}

		if (m_poseUser == 0 || m_poseUser == user.getId())
		{
			const nite::PoseData& pose = user.getPose(nite::POSE_CROSSED_HANDS);
			
			if (pose.isEntered())
			{
				// Start timer
				sprintf_s(g_generalMessage, "In exit pose. Keep it for %d second%s to exit\n", g_poseTimeoutToExit/1000, g_poseTimeoutToExit/1000 == 1 ? "" : "s");
				printf("Counting down %d second to exit\n", g_poseTimeoutToExit/1000);
				m_poseUser = user.getId();
				m_poseTime = userTrackerFrame.getTimestamp();
			}
			else if (pose.isExited())
			{
				memset(g_generalMessage, 0, sizeof(g_generalMessage));
				printf("Count-down interrupted\n");
				m_poseTime = 0;
				m_poseUser = 0;
			}
			else if (pose.isHeld())
			{
				// Timer tick
				if (userTrackerFrame.getTimestamp() - m_poseTime > g_poseTimeoutToExit * 1000)
				{
					printf("Count down complete. Exit...\n");
					Finalize();
					exit(2);
				}
			}
		}
	}

	if (g_drawFrameId)
	{
		DrawFrameId(userTrackerFrame.getFrameIndex());
	}

	if (g_generalMessage[0] != '\0')
	{
		char *msg = g_generalMessage;
		glColor3f(1.0f, 0.0f, 0.0f);
		glRasterPos2i(100, 20);		
	}
	// Swap the OpenGL display buffers
	glutSwapBuffers();
}
#pragma endregion
#pragma region OpenGL
void SampleViewer::glutIdle()
{
	glutPostRedisplay();
}

void SampleViewer::glutDisplay()
{
	SampleViewer::ms_self->Display();
}

void SampleViewer::glutKeyboard(unsigned char key, int x, int y)
{
	SampleViewer::ms_self->OnKey(key, x, y);
}

// Application features using keyboard shortcuts :)
void SampleViewer::OnKey(unsigned char key, int /*x*/, int /*y*/)
{
	switch (key)
	{
	case 27:
		Finalize();
		exit (1);
	case 's':
		g_drawSkeleton = !g_drawSkeleton;
		break;
	case 'l':
		g_drawStatusLabel = !g_drawStatusLabel;
		break;
	case 'c':
		g_drawCenterOfMass = !g_drawCenterOfMass;
		break;
	case 'x':		
		g_drawBoundingBox = !g_drawBoundingBox;
		break;
	case 'b':
		g_drawBackground = !g_drawBackground;
		break;
	case 'd':
		g_drawDepth = !g_drawDepth;
		break;
	case 'f':
		g_drawFrameId = !g_drawFrameId;
		break;
	}

}

openni::Status SampleViewer::InitOpenGL(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	glutCreateWindow (m_strSampleName);
	glutFullScreen();
	glutSetCursor(GLUT_CURSOR_NONE);

	InitOpenGLHooks();

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);

	glEnableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

	return openni::STATUS_OK;
}
void SampleViewer::InitOpenGLHooks()
{
	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);
}
#pragma endregion
#pragma endregion
