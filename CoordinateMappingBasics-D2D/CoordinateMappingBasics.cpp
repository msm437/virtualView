//------------------------------------------------------------------------------
// <copyright file="CoordinateMappingBasics.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "stdafx.h"
#include <strsafe.h>
#include <math.h>
#include <Wincodec.h>
#include <conio.h>
#include <iostream>
#include <fstream>
#include "resource.h"
#include "CoordinateMappingBasics.h"

#ifndef HINST_THISCOMPONENT
EXTERN_C IMAGE_DOS_HEADER __ImageBase;
#define HINST_THISCOMPONENT ((HINSTANCE)&__ImageBase)
#endif

/// <summary>
/// Entry point for the application
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="hPrevInstance">always 0</param>
/// <param name="lpCmdLine">command line arguments</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
/// <returns>status</returns>
int APIENTRY wWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPWSTR lpCmdLine, int nCmdShow)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    CCoordinateMappingBasics application;
    application.Run(hInstance, nCmdShow);
}

/// <summary>
/// Constructor
/// </summary>
CCoordinateMappingBasics::CCoordinateMappingBasics() :
    m_hWnd(NULL),
    m_nStartTime(0),
    m_nLastCounter(0),
    m_nFramesSinceUpdate(0),
    m_fFreq(0),
    m_nNextStatusTime(0),
    m_bSaveScreenshot(false),
    m_pKinectSensor(NULL),
    m_pCoordinateMapper(NULL),
    m_pMultiSourceFrameReader(NULL),
    m_pColorCoordinates(NULL),
	m_pColorCoordinatesDeformation(NULL),
	m_pColorCoordinatesTemp(NULL),
	m_pColorCoordinatesColor(NULL),
	m_pDepthCoordiantes(NULL),
	m_pCameraCoordinates(NULL),
	m_pColorCameraCoordinates(NULL),
    m_pD2DFactory(NULL),
	m_pD2DFactoryDeformation(NULL),
    m_pDrawCoordinateMapping(NULL),
	m_pDrawDeformation(NULL),
	m_originalRGBX(NULL),
    m_pOutputRGBX(NULL),
	m_pOutputRGBXInterpolation(NULL),
	m_pDepthBuffer(NULL),
    m_pBackgroundRGBX(NULL),
    m_pColorRGBX(NULL),
	m_pBothColorSpaceValid(NULL),
	m_pValidColorMapping(NULL),
	m_pDepthBufferSrc(NULL),
	m_pColorBufferSrc(NULL),
	m_pBodyIndexBufferSrc(NULL),
	m_rotationX(0),
	m_rotationY(0),
	m_rotationZ(0),
	m_updateNewframe(true),
	m_computeDeformation(false),
	m_enableSimpleInterpolation(true),
	m_enableMLSDeformation(false),
	m_saveBackground(false),
	m_saveHumanbody(false),
	m_hMapFile(NULL),
	m_pBuf(NULL)
{
    LARGE_INTEGER qpf = {0};
    if (QueryPerformanceFrequency(&qpf))
    {
        m_fFreq = double(qpf.QuadPart);
    }

	m_pDepthBuffer = new UINT16[cDepthWidth * cDepthHeight];
    // create heap storage for composite image pixel data in RGBX format
	m_pOutputRGBX = new RGBQUAD[cColorWidth * cColorHeight];
	m_originalRGBX = new RGBQUAD[cColorWidth * cColorHeight];
	m_pOutputRGBXInterpolation = new RGBQUAD[cColorWidth * cColorHeight];
	memset(m_pOutputRGBXInterpolation, 0, sizeof(RGBQUAD)*cColorWidth*cColorHeight);
    // create heap storage for background image pixel data in RGBX format
	m_pBackgroundRGBX = new RGBQUAD[cColorWidth * cColorHeight];

    // create heap storage for color pixel data in RGBX format
    m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];

    // create heap storage for the coorinate mapping from depth to color
	m_pColorCoordinatesTemp = new ColorSpacePoint[cDepthWidth * cDepthHeight];
    m_pColorCoordinates = new ColorSpacePoint[cDepthWidth * cDepthHeight];
	m_pColorCoordinatesDeformation = new ColorSpacePoint[cDepthWidth*cDepthHeight];
	m_pColorCoordinatesColor = new ColorSpacePoint[cColorWidth * cColorHeight];
	m_pDepthCoordiantes = new DepthSpacePoint[cDepthHeight*cDepthWidth];
	m_pValidColorMapping = new bool[cColorWidth * cColorHeight];
	m_pBothColorSpaceValid = new bool[cDepthWidth * cDepthHeight];
	m_colorSpaceStatus = new int[cColorWidth* cColorHeight];

	//save the image from every frame
	m_pDepthBufferSrc = new UINT16[cDepthWidth * cDepthHeight];
	m_pColorBufferSrc = new RGBQUAD[cColorWidth * cColorHeight];
	m_pBodyIndexBufferSrc = new BYTE[cDepthWidth * cDepthHeight];
	//create heap storage for the coorinate mapping from depth and color to camera(point cloud)
	m_pCameraCoordinates = new CameraSpacePoint[cDepthWidth * cDepthHeight];
	m_pColorCameraCoordinates = new CameraSpacePoint[cColorWidth * cColorHeight];

	//init the transformatino of the virtual camera
	m_cameraTranslation.x() = 0;
	m_cameraTranslation.y() = 0;
	m_cameraTranslation.z() = 0;
	m_cameraTransformation = Eigen::AngleAxisd(m_rotationX, Eigen::Vector3d::UnitX())*  Eigen::AngleAxisd(m_rotationY, Eigen::Vector3d::UnitY())* Eigen::AngleAxisd(m_rotationZ, Eigen::Vector3d::UnitZ())* m_cameraTranslation;

	//init the shared memory
	TCHAR szName[] = TEXT("NewKinectPointCloud");
	m_sizeOfFeaturePoints = sizeof(int)+sizeof(float)* 3 * 100;//max feature points= 100
	m_sizeOfPointClouds = sizeof(int)* 4+ sizeof(float)* 4 * cDepthWidth * cDepthHeight + sizeof(int) + sizeof(float)*3 * 100;
	m_bufSize = m_sizeOfPointClouds + m_sizeOfFeaturePoints;
	m_hMapFile = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, m_bufSize, szName);
	if (m_hMapFile == NULL){
		std::cout << "Init shared memory error\n";
		return;
	}
	m_pBuf = (void*)MapViewOfFile(m_hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, m_bufSize);
	if(m_pBuf == NULL){
		std::cout << "Mapping shared memory error\n";
		return;
	}
	int* pInt = reinterpret_cast<int*>(m_pBuf);
	//first int 0  XYZ 1 XYZ RGB
	//second int: width of the point cloud
	//third int : height of the point cloud
	//fouth: empty
	*pInt = 1;
	*(pInt + 1) = cDepthWidth;
	*(pInt + 2) = cDepthHeight;
}
  

/// <summary>
/// Destructor
/// </summary>
CCoordinateMappingBasics::~CCoordinateMappingBasics()
{
    // clean up Direct2D renderer
    if (m_pDrawCoordinateMapping)
    {
        delete m_pDrawCoordinateMapping;
        m_pDrawCoordinateMapping = NULL;
    }
	if (m_pDrawDeformation)
	{
		delete m_pDrawDeformation;
		m_pDrawDeformation = NULL;
	}
    if (m_pOutputRGBX)
    {
        delete [] m_pOutputRGBX;
        m_pOutputRGBX = NULL;
    }

    if (m_pBackgroundRGBX)
    {
        delete [] m_pBackgroundRGBX;
        m_pBackgroundRGBX = NULL;
    }

    if (m_pColorRGBX)
    {
        delete [] m_pColorRGBX;
        m_pColorRGBX = NULL;
    }

    if (m_pColorCoordinates)
    {
        delete [] m_pColorCoordinates;
        m_pColorCoordinates = NULL;
    }
	if (m_pCameraCoordinates)
	{
		delete[] m_pCameraCoordinates;
		m_pCameraCoordinates = NULL;
	}
	if (m_pColorCameraCoordinates)
	{
		delete[] m_pColorCameraCoordinates;
		m_pColorCameraCoordinates = NULL;
	}
    // clean up Direct2D
    SafeRelease(m_pD2DFactory);

    // done with frame reader
    SafeRelease(m_pMultiSourceFrameReader);

    // done with coordinate mapper
    SafeRelease(m_pCoordinateMapper);

    // close the Kinect Sensor
    if (m_pKinectSensor)
    {
        m_pKinectSensor->Close();
    }
	if (m_pBuf){
		UnmapViewOfFile(m_pBuf);
		m_pBuf = NULL;
	}
	if (m_hMapFile){
		CloseHandle(m_hMapFile);
		m_hMapFile = NULL;
	}
    SafeRelease(m_pKinectSensor);
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
int CCoordinateMappingBasics::Run(HINSTANCE hInstance, int nCmdShow)
{
    if (m_pBackgroundRGBX)
    {
        if (FAILED(LoadResourceImage(L"Background", L"Image", cDepthWidth, cDepthHeight, m_pBackgroundRGBX)))
        {
            delete [] m_pBackgroundRGBX;
            m_pBackgroundRGBX = NULL;
        }
    }

    MSG       msg = {0};
    WNDCLASS  wc;

    // Dialog custom window class
    ZeroMemory(&wc, sizeof(wc));
    wc.style         = CS_HREDRAW | CS_VREDRAW;
    wc.cbWndExtra    = DLGWINDOWEXTRA;
    wc.hCursor       = LoadCursorW(NULL, IDC_ARROW);
    wc.hIcon         = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
    wc.lpfnWndProc   = DefDlgProcW;
    wc.lpszClassName = L"CoordinateMappingBasicsAppDlgWndClass";

    if (!RegisterClassW(&wc))
    {
        return 0;
    }

    // Create main application window
    HWND hWndApp = CreateDialogParamW(
        NULL,
        MAKEINTRESOURCE(IDD_APP),
        NULL,
        (DLGPROC)CCoordinateMappingBasics::MessageRouter, 
        reinterpret_cast<LPARAM>(this));
    // Show window
    ShowWindow(hWndApp, nCmdShow);
    // Main message loop
    while (WM_QUIT != msg.message)
    {
        Update();

        while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
        {
			if(msg.message == WM_KEYDOWN){
								switch (msg.wParam){
								case VK_UP:
									m_rotationX += 0.01;
									break;
								case VK_DOWN:
									m_rotationX -= 0.01; break;
								case VK_LEFT:
									m_rotationY += 0.01; break;
								case VK_RIGHT:
									m_rotationY -= 0.01; break;
								case 'A': m_cameraTranslation.x() += 0.01; break;
								case 'D': m_cameraTranslation.x() -= 0.01; break;
								case 'W': m_cameraTranslation.y() += 0.01; break;
								case 'S': m_cameraTranslation.y() -= 0.01; break;
								case 'Q': m_cameraTranslation.z() += 0.01; break;
								case 'E': m_cameraTranslation.z() -= 0.01; break;
								case 'I': m_enableSimpleInterpolation = !m_enableSimpleInterpolation; break;
								case 'M': m_enableMLSDeformation = true; m_enableSimpleInterpolation = true; break;
								case 'B': m_saveBackground = true; break;
								case 'H': m_saveHumanbody = true; break;
								case 'R': 
									m_rotationX = 0; m_rotationY = 0; m_rotationZ = 0;
									m_cameraTranslation.x() = 0; m_cameraTranslation.y() = 0; m_cameraTranslation.z() = 0; break;
								case 'U': m_updateNewframe = true; break;
								case 'F': m_computeDeformation = true; break;
								default: break;
								}
								m_cameraTransformation = Eigen::AngleAxisd(m_rotationX, Eigen::Vector3d::UnitX())*  Eigen::AngleAxisd(m_rotationY, Eigen::Vector3d::UnitY())* Eigen::AngleAxisd(m_rotationZ, Eigen::Vector3d::UnitZ())* m_cameraTranslation;
			}
            // If a dialog message will be taken care of by the dialog proc
            if (hWndApp && IsDialogMessageW(hWndApp, &msg))
            {
                continue;
            }

            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }

    return static_cast<int>(msg.wParam);
}

/// <summary>
/// Main processing function
/// </summary>
void CCoordinateMappingBasics::Update()
{
    if (!m_pMultiSourceFrameReader)
    {
        return;
    }

    IMultiSourceFrame* pMultiSourceFrame = NULL;
    IDepthFrame* pDepthFrame = NULL;
    IColorFrame* pColorFrame = NULL;
    IBodyIndexFrame* pBodyIndexFrame = NULL;

    HRESULT hr = m_pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame);

    if (SUCCEEDED(hr))
    {
        IDepthFrameReference* pDepthFrameReference = NULL;

        hr = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
        if (SUCCEEDED(hr))
        {
            hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);
        }

        SafeRelease(pDepthFrameReference);
    }

    if (SUCCEEDED(hr))
    {
        IColorFrameReference* pColorFrameReference = NULL;

        hr = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
        if (SUCCEEDED(hr))
        {
            hr = pColorFrameReference->AcquireFrame(&pColorFrame);
        }

        SafeRelease(pColorFrameReference);
    }

    if (SUCCEEDED(hr))
    {
        IBodyIndexFrameReference* pBodyIndexFrameReference = NULL;

        hr = pMultiSourceFrame->get_BodyIndexFrameReference(&pBodyIndexFrameReference);
        if (SUCCEEDED(hr))
        {
            hr = pBodyIndexFrameReference->AcquireFrame(&pBodyIndexFrame);
        }

        SafeRelease(pBodyIndexFrameReference);
    }

    if (SUCCEEDED(hr))
    {
        INT64 nDepthTime = 0;
        IFrameDescription* pDepthFrameDescription = NULL;
        int nDepthWidth = 0;
        int nDepthHeight = 0;
        UINT nDepthBufferSize = 0;
        UINT16 *pDepthBuffer = NULL;

        IFrameDescription* pColorFrameDescription = NULL;
        int nColorWidth = 0;
        int nColorHeight = 0;
        ColorImageFormat imageFormat = ColorImageFormat_None;
        UINT nColorBufferSize = 0;
        RGBQUAD *pColorBuffer = NULL;

        IFrameDescription* pBodyIndexFrameDescription = NULL;
        int nBodyIndexWidth = 0;
        int nBodyIndexHeight = 0;
        UINT nBodyIndexBufferSize = 0;
        BYTE *pBodyIndexBuffer = NULL;

        // get depth frame data

        hr = pDepthFrame->get_RelativeTime(&nDepthTime);

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrame->get_FrameDescription(&pDepthFrameDescription);
        }

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrameDescription->get_Width(&nDepthWidth);
        }

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrameDescription->get_Height(&nDepthHeight);
        }

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pDepthBuffer);            
        }

        // get color frame data

        if (SUCCEEDED(hr))
        {
            hr = pColorFrame->get_FrameDescription(&pColorFrameDescription);
        }

        if (SUCCEEDED(hr))
        {
            hr = pColorFrameDescription->get_Width(&nColorWidth);
        }

        if (SUCCEEDED(hr))
        {
            hr = pColorFrameDescription->get_Height(&nColorHeight);
        }

        if (SUCCEEDED(hr))
        {
            hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
        }

        if (SUCCEEDED(hr))
        {
            if (imageFormat == ColorImageFormat_Bgra)
            {
                hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
            }
            else if (m_pColorRGBX)
            {
                pColorBuffer = m_pColorRGBX;
                nColorBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
                hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat_Bgra);
            }
            else
            {
                hr = E_FAIL;
            }
        }

        // get body index frame data

        if (SUCCEEDED(hr))
        {
            hr = pBodyIndexFrame->get_FrameDescription(&pBodyIndexFrameDescription);
        }

        if (SUCCEEDED(hr))
        {
            hr = pBodyIndexFrameDescription->get_Width(&nBodyIndexWidth);
        }

        if (SUCCEEDED(hr))
        {
            hr = pBodyIndexFrameDescription->get_Height(&nBodyIndexHeight);
        }

        if (SUCCEEDED(hr))
        {
            hr = pBodyIndexFrame->AccessUnderlyingBuffer(&nBodyIndexBufferSize, &pBodyIndexBuffer);            
        }

        if (SUCCEEDED(hr))
        {
			//image deformation processing
			//if (m_updateNewframe){
			//	memcpy(m_pDepthBufferSrc, pDepthBuffer, sizeof(UINT16)* cDepthWidth * cDepthHeight);
			//	memcpy(m_pColorBufferSrc, pColorBuffer, sizeof(RGBQUAD)*cColorWidth * cColorHeight);
			//	memcpy(m_pBodyIndexBufferSrc, pBodyIndexBuffer, sizeof(BYTE)*cDepthWidth * cDepthHeight);
			//	memcpy(m_originalRGBX, pColorBuffer, cColorWidth*cColorHeight * sizeof(RGBQUAD));
			//	m_updateNewframe = false;
			//	//write the point cloud into the shared memory
			//	//writePointCloudSharedMemory(pDepthBuffer, cDepthWidth, cDepthHeight, pColorBuffer, cColorWidth, cColorHeight, pBodyIndexBuffer, nBodyIndexWidth, nBodyIndexHeight);
			//}
			/*ProcessFrame(nDepthTime, m_pDepthBufferSrc, nDepthWidth, nDepthHeight,
				m_pColorBufferSrc, nColorWidth, nColorHeight,
				m_pBodyIndexBufferSrc, nBodyIndexWidth, nBodyIndexHeight);*/
			/**/
			//ProcessFrameInterpolation(nDepthTime, m_pDepthBufferSrc, nDepthWidth, nDepthHeight,
			//	m_pColorBufferSrc, nColorWidth, nColorHeight,
			//	m_pBodyIndexBufferSrc, nBodyIndexWidth, nBodyIndexHeight);
			/*ProcessFrameByFeaturePoints(nDepthTime, m_pDepthBufferSrc, nDepthWidth, nDepthHeight,
				m_pColorBufferSrc, nColorWidth, nColorHeight,
				m_pBodyIndexBufferSrc, nBodyIndexWidth, nBodyIndexHeight);*/
			/*for (int i = 0; i < cColorWidth * cColorHeight; i++)
			{
				if (reinterpret_cast<INT32*>(m_pOutputRGBXInterpolation)[i])
					m_pOutputRGBX[i] = m_pOutputRGBXInterpolation[i];
			}*/
			////show and check the saved image 
			if (m_saveBackground){
				loadDepthColorImg("backGround", m_pColorBufferSrc, cColorWidth, cColorHeight, m_pDepthBufferSrc, cDepthWidth, cDepthHeight);
				m_pDrawCoordinateMapping->Draw(reinterpret_cast<BYTE*>(m_pColorBufferSrc), cColorWidth*cColorHeight * sizeof(RGBQUAD));
				m_saveBackground = false;
			}
			if (m_saveHumanbody){
				loadDepthColorImg("user", m_pColorBufferSrc, cColorWidth, cColorHeight, m_pDepthBufferSrc, cDepthWidth, cDepthHeight);
				m_pDrawCoordinateMapping->Draw(reinterpret_cast<BYTE*>(m_pColorBufferSrc), cColorWidth*cColorHeight * sizeof(RGBQUAD));
				m_saveHumanbody = false;
			}
			//save frame information
			/*m_pDrawCoordinateMapping->Draw(reinterpret_cast<BYTE*>(pColorBuffer), cColorWidth*cColorHeight * sizeof(RGBQUAD));
			if (m_saveBackground){
			saveDepthColorImg("backGround", pColorBuffer, cColorWidth, cColorHeight, pDepthBuffer, cDepthWidth, cDepthHeight);
			m_saveBackground = false;
			}
			if (m_saveHumanbody){
			saveDepthColorImg("user", pColorBuffer, cColorWidth, cColorHeight, pDepthBuffer, cDepthWidth, cDepthHeight);
			m_saveHumanbody = false;
			}*/
			//static image gaze correcting
			//correctedGazeofStaticImage();
			//m_pDrawCoordinateMapping->Draw(reinterpret_cast<BYTE*>(m_pColorBufferSrc), cColorWidth*cColorHeight * sizeof(RGBQUAD)); 
			//m_pDrawDeformation->Draw(reinterpret_cast<BYTE*>(m_pOutputRGBX), cColorWidth*cColorHeight * sizeof(RGBQUAD));
			//m_pDrawCoordinateMapping->Draw(reinterpret_cast<BYTE*>(m_originalRGBX), cColorWidth*cColorHeight * sizeof(RGBQUAD));
			//m_pDrawDeformation->Draw(reinterpret_cast<BYTE*>(m_pOutputRGBXInterpolation), cColorWidth*cColorHeight * sizeof(RGBQUAD));
			//save the current pointCloud
			m_pDrawDeformation->Draw(reinterpret_cast<BYTE*>(pColorBuffer), cColorWidth*cColorHeight * sizeof(RGBQUAD));
			if (m_updateNewframe){
				SavePointCloud(pDepthBuffer, cDepthWidth, cDepthHeight, pColorBuffer, cColorWidth, cColorHeight);
				m_updateNewframe = false;
			}
			//save the current color point cloud
			//saveColorPointCloud(pDepthBuffer, cDepthWidth, cDepthHeight, pColorBuffer, cColorWidth, cColorHeight, pBodyIndexBuffer, nBodyIndexWidth, nBodyIndexHeight);

			////write the point cloud into the shared memory
			//m_pDrawDeformation->Draw(reinterpret_cast<BYTE*>(pColorBuffer), cColorWidth*cColorHeight * sizeof(RGBQUAD));
			//writePointCloudSharedMemory(pDepthBuffer, cDepthWidth, cDepthHeight, pColorBuffer, cColorWidth, cColorHeight, pBodyIndexBuffer, nBodyIndexWidth, nBodyIndexHeight);
        }

        SafeRelease(pDepthFrameDescription);
        SafeRelease(pColorFrameDescription);
        SafeRelease(pBodyIndexFrameDescription);
    }

    SafeRelease(pDepthFrame);
    SafeRelease(pColorFrame);
    SafeRelease(pBodyIndexFrame);
    SafeRelease(pMultiSourceFrame);
}

/// <summary>
/// Handles window messages, passes most to the class instance to handle
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CCoordinateMappingBasics::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    CCoordinateMappingBasics* pThis = NULL;
    
    if (WM_INITDIALOG == uMsg)
    {
        pThis = reinterpret_cast<CCoordinateMappingBasics*>(lParam);
        SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
    }
    else
    {
        pThis = reinterpret_cast<CCoordinateMappingBasics*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
    }

    if (pThis)
    {
        return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
    }

    return 0;
}

/// <summary>
/// Handle windows messages for the class instance
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CCoordinateMappingBasics::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(wParam);
    UNREFERENCED_PARAMETER(lParam);

    switch (message)
    {
        case WM_INITDIALOG:
        {
            // Bind application window handle
            m_hWnd = hWnd;

            // Init Direct2D
            D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

            // Create and initialize a new Direct2D image renderer (take a look at ImageRenderer.h)
            // We'll use this to draw the data we receive from the Kinect to the screen
            m_pDrawCoordinateMapping = new ImageRenderer(); 
			HRESULT hr = m_pDrawCoordinateMapping->Initialize(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), m_pD2DFactory, cColorWidth, cColorHeight, cColorWidth * sizeof(RGBQUAD));
			D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactoryDeformation);
			m_pDrawDeformation = new ImageRenderer();
			hr = m_pDrawDeformation->Initialize(GetDlgItem(m_hWnd, IDC_VIDEOVIEW1), m_pD2DFactoryDeformation, cColorWidth, cColorHeight, cColorWidth * sizeof(RGBQUAD));
            if (FAILED(hr))
            {
                SetStatusMessage(L"Failed to initialize the Direct2D draw device.", 10000, true);
            }

            // Get and initialize the default Kinect sensor
            InitializeDefaultSensor();
        }
        break;

        // If the titlebar X is clicked, destroy app
        case WM_CLOSE:
            DestroyWindow(hWnd);
            break;

        case WM_DESTROY:
            // Quit the main message pump
            PostQuitMessage(0);
            break;

        // Handle button press
        case WM_COMMAND:
            // If it was for the screenshot control and a button clicked event, save a screenshot next frame 
            if (IDC_BUTTON_SCREENSHOT == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
            {
                m_bSaveScreenshot = true;
            }
            break;
    }

    return FALSE;
}

/// <summary>
/// Initializes the default Kinect sensor
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT CCoordinateMappingBasics::InitializeDefaultSensor()
{
    HRESULT hr;

    hr = GetDefaultKinectSensor(&m_pKinectSensor);
	/*IKinectSensorCollection* kinectCollection;
	IEnumKinectSensor* kinectEnumerator;
	IKinectSensor* tempSensor;
	hr = GetKinectSensorCollection(&kinectCollection);
	hr = kinectCollection->get_Enumerator(&kinectEnumerator);
	hr = kinectEnumerator->GetNext(&m_pKinectSensor);
	hr = kinectEnumerator->GetNext(&tempSensor);*/
    if (FAILED(hr))
    {
        return hr;
    }

    if (m_pKinectSensor)
    {
        // Initialize the Kinect and get coordinate mapper and the frame reader

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
        }

        hr = m_pKinectSensor->Open();

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->OpenMultiSourceFrameReader(
                FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color | FrameSourceTypes::FrameSourceTypes_BodyIndex,
                &m_pMultiSourceFrameReader);
        }
    }

    if (!m_pKinectSensor || FAILED(hr))
    {
        SetStatusMessage(L"No ready Kinect found!", 10000, true);
        return E_FAIL;
    }
	IDepthFrameSource* pDepthSource;
	hr = m_pKinectSensor->get_DepthFrameSource(&pDepthSource);
	hr = pDepthSource->get_DepthMaxReliableDistance(&cMaxDepthDistance);
	hr = pDepthSource->get_DepthMinReliableDistance(&cMinDepthDistance);
    return hr;
}

bool CCoordinateMappingBasics::invalidColorSpace(ColorSpacePoint c)
{
	if (c.X < 0 || c.X >= cColorWidth-0.5 || c.Y < 0 || c.Y >= cColorHeight-0.5)
		return true;
	return false;
}

void CCoordinateMappingBasics::fillTheHoleInColorspace(ColorSpacePoint* dst, ColorSpacePoint* src)
{
	for (int y = 1; y < cDepthHeight - 1; ++y){
		for (int x = 1; x < cDepthWidth - 1; ++x){
			const ColorSpacePoint* pColorPoint = src + y * cDepthWidth + x;
			if (invalidColorSpace(*pColorPoint)){
				//find the average of the eight adjacencys
				ColorSpacePoint sum;
				sum.X = 0;
				sum.Y = 0;
				int count = 0;
				for (int j = -1; j <= 1; j++){
					for (int i = -1; i <= 1; i++){
						ColorSpacePoint temp = *(pColorPoint + j*cDepthWidth + i);
						if (invalidColorSpace(temp))
							continue;
						sum.X += temp.X;
						sum.Y += temp.Y;
						count++;
					}
				}
				if (count == 0)
					continue;
				ColorSpacePoint* d = dst + y * cDepthWidth + x;
				d->X = sum.X / count;
				d->Y = sum.Y / count;
			}
		}
	}
}
void CCoordinateMappingBasics::colorDeformationMapping(RGBQUAD* pDstColorBuffer, const RGBQUAD* pSrcColorBuffer, int width, int height, ColorSpacePoint dUL, ColorSpacePoint dUR, ColorSpacePoint dDL, ColorSpacePoint dDR, \
	ColorSpacePoint sUL, ColorSpacePoint sUR, ColorSpacePoint sDL, ColorSpacePoint sDR)
{
	if (invalidColorSpace(sUL) || invalidColorSpace(sUR) || invalidColorSpace(sDL) || invalidColorSpace(sDR))
		return;
	if (invalidColorSpace(dUL) || invalidColorSpace(dUR) || invalidColorSpace(dDL) || invalidColorSpace(dDR))
		return;
	int x1 = (int)((sUL.X + sDL.X) / 2 + 0.5), x2 = (int)((sUR.X + sDR.X) / 2 + 0.5);
	int y1 = (int)((sUL.Y + sUR.Y) / 2 + 0.5), y2 = (int)((sDL.Y + sDR.Y) / 2 + 0.5);
	for (int y = y1; y < y2; ++y){
		double t = (y - y1)*1.0 / (y2 - y1);
		for (int x = x1; x < x2; ++x){
			if (x < 0 || x > width || y < 0 || y > height)
				continue;
			double s = (x - x1)*1.0 / (x2 - x1);
			double xd = ((1 - t)*dUL.X + t*dDL.X)*(1 - s) + s*((1 - t)*dUR.X + t * dDR.X);
			double yd = ((1 - s)*dUL.Y + s*dUR.Y)*(1 - t) + t*((1 - s)*dDL.Y + s * dDR.Y);
			int xDst = (int)(xd + 0.5), yDst = (int)(yd + 0.5);
			if (xDst < 0 || xDst >=width || yDst < 0 || yDst >= height)
				continue;
			pDstColorBuffer[xDst + yDst*width] = pSrcColorBuffer[x + y * width];
		}
	}
}
void CCoordinateMappingBasics::colorDeformationBackMapping(RGBQUAD* pDstColorBuffer, const RGBQUAD* pSrcColorBuffer, int width, int height, ColorSpacePoint dUL, ColorSpacePoint dUR, ColorSpacePoint dDL, ColorSpacePoint dDR, \
	ColorSpacePoint sUL, ColorSpacePoint sUR, ColorSpacePoint sDL, ColorSpacePoint sDR)
{
	if (invalidColorSpace(sUL) || invalidColorSpace(sUR) || invalidColorSpace(sDL) || invalidColorSpace(sDR))
		return;
	if (invalidColorSpace(dUL) || invalidColorSpace(dUR) || invalidColorSpace(dDL) || invalidColorSpace(dDR))
		return;
	int x1 = (int)((dUL.X + dDL.X) / 2 + 0.5), x2 = (int)((dUR.X + dDR.X) / 2 + 0.5);
	int y1 = (int)((dUL.Y + dUR.Y) / 2 + 0.5), y2 = (int)((dDL.Y + dDR.Y) / 2 + 0.5);
	for (int y = y1; y < y2; ++y){
		double t = (y - y1)*1.0 / (y2 - y1);
		for (int x = x1; x < x2; ++x){
			if (x < 0 || x > width || y < 0 || y > height)
				continue;
			double s = (x - x1)*1.0 / (x2 - x1);
			double xs = ((1 - t)*sUL.X + t*sDL.X)*(1 - s) + s*((1 - t)*sUR.X + t * sDR.X);
			double ys = ((1 - s)*sUL.Y + s*sUR.Y)*(1 - t) + t*((1 - s)*sDL.Y + s * sDR.Y);
			int xSrc = (int)(xs + 0.5), ySrc = (int)(ys + 0.5);
			if (xSrc < 0 || xSrc >= width || ySrc < 0 || ySrc >= height)
				continue;
			pDstColorBuffer[x + y*width] = pSrcColorBuffer[xSrc + ySrc * width];
			//add new mapping information
			m_pColorCoordinatesColor[x + y*width].X = xSrc;
			m_pColorCoordinatesColor[x + y*width].Y = ySrc;
			m_pValidColorMapping[x + y*width] = true;
		}
	}
}

//convert current depth image to cameraspace to generate the basic point cloud and save the current point cloud into pcd file
void CCoordinateMappingBasics::SavePointCloud(const UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight,
									 RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight)
{
	//update the color image 
	m_pDrawCoordinateMapping->Draw(reinterpret_cast<BYTE*>(pColorBuffer), cColorWidth*cColorHeight * sizeof(RGBQUAD));
	if (!m_updateNewframe)
		return;
	m_updateNewframe = false;
	//compute the point cloud
	HRESULT hr = m_pCoordinateMapper->MapDepthFrameToCameraSpace(nDepthWidth*nDepthHeight, (UINT16*)pDepthBuffer, nDepthWidth * nDepthHeight, m_pCameraCoordinates);
	//save the point cloud into file
	std::ofstream PCFile("NewKinect.pcd");
	PCFile << "# .PCD v0.7 - Point Cloud Data file format" << std::endl;
	PCFile << "VERSION 0.7" << std::endl;
	PCFile << "FIELDS x y z\n";
	PCFile << "SIZE 4 4 4\n";
	PCFile << "TYPE F F F\n";
	PCFile << "COUNT 1 1 1\n";
	PCFile << "WIDTH " << cDepthWidth << std::endl;
	PCFile << "HEIGHT " << cDepthHeight << std::endl;
	PCFile << "VIEWPOINT 0 0 0 1 0 0 0\n";
	PCFile << "POINTS " << cDepthWidth * cDepthHeight << std::endl;
	PCFile << "DATA ascii\n";
	for (int i = 0; i < nDepthWidth * nDepthHeight; i++){
		PCFile << m_pCameraCoordinates[i].X << "\t" << m_pCameraCoordinates[i].Y << "\t" << m_pCameraCoordinates[i].Z << std::endl;
	}
	PCFile.close();
}
//generate the color point cloud of the user body
void CCoordinateMappingBasics::saveColorPointCloud(const UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight,
	RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight,
	const BYTE* pBodyIndexBuffer, int nBodyIndexWidth, int nBodyIndexHeight)
{
	//updatet the color image video
	m_pDrawCoordinateMapping->Draw(reinterpret_cast<BYTE*>(pColorBuffer), cColorWidth*cColorHeight * sizeof(RGBQUAD));
	if (!m_updateNewframe)
		return;
	m_updateNewframe = false;
	////generate the color point cloud
	//HRESULT hr = m_pCoordinateMapper->MapColorFrameToCameraSpace(nDepthWidth* nDepthHeight, (UINT16*)pDepthBuffer, nColorWidth*nColorHeight, m_pColorCameraCoordinates);
	////save the color point cloud into PCD file
	//std::ofstream PCFile("NewKinectColorWorld.pcd");
	//if (!PCFile.good()){
	//	std::cout << "ERROR: open the PCD file\n";
	//	return;
	//}
	//PCFile << "# .PCD v0.7 - Point Cloud Data file format" << std::endl;
	//PCFile << "VERSION 0.7" << std::endl;
	//PCFile << "FIELDS x y z rgb\n";
	//PCFile << "SIZE 4 4 4 4\n";
	//PCFile << "TYPE F F F F\n";
	//PCFile << "COUNT 1 1 1 1\n";
	//PCFile << "WIDTH " << nColorWidth << std::endl;
	//PCFile << "HEIGHT " << nColorHeight << std::endl;
	//PCFile << "VIEWPOINT 0 0 0 1 0 0 0\n";
	//PCFile << "POINTS " << nColorWidth * nColorHeight << std::endl;
	//PCFile << "DATA ascii\n";
	//UINT32* pRGB = reinterpret_cast<UINT32*>(pColorBuffer);
	//PCFile << std::scientific;
	//for (int i = 0; i < nColorWidth*nColorHeight; i++)
	//{
	//	CameraSpacePoint cP = m_pColorCameraCoordinates[i];
	//	UINT32 rgb = (static_cast<UINT32>((pColorBuffer[i]).rgbRed) << 16 | static_cast<UINT32>(pColorBuffer[i].rgbGreen) << 8 | static_cast<UINT32>(pColorBuffer[i].rgbBlue));
	//	PCFile << cP.X << "\t" << cP.Y << "\t" << cP.Z << "\t" << *reinterpret_cast<float*>(&rgb) << std::endl;
	//}
	//PCFile.close();
	
	//the method mapping all the color information to camera space does not work well. 
	//now try just mapping the valid depth point with rgb information to the camera space
	HRESULT hr = m_pCoordinateMapper->MapDepthFrameToCameraSpace(nDepthWidth*nDepthHeight, (UINT16*)pDepthBuffer, nDepthWidth*nDepthHeight, m_pCameraCoordinates);
	hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(nDepthWidth*nDepthHeight, (UINT16*)pDepthBuffer, nDepthWidth*nDepthHeight, m_pColorCoordinates);

	//save the color point cloud into PCD file
	std::ofstream PCFile("NewKinectColor.pcd");
	if (!PCFile.good()){
		std::cout << "ERROR: open the PCD file\n";
		return;
	}
	PCFile << "# .PCD v0.7 - Point Cloud Data file format" << std::endl;
	PCFile << "VERSION 0.7" << std::endl;
	PCFile << "FIELDS x y z rgb\n";
	PCFile << "SIZE 4 4 4 4\n";
	PCFile << "TYPE F F F F\n";
	PCFile << "COUNT 1 1 1 1\n";
	PCFile << "WIDTH " << nDepthWidth << std::endl;
	PCFile << "HEIGHT " << nDepthHeight << std::endl;
	PCFile << "VIEWPOINT 0 0 0 1 0 0 0\n";
	PCFile << "POINTS " << nDepthWidth * nDepthHeight << std::endl;
	PCFile << "DATA ascii\n";
	UINT32 rgb;
	RGBQUAD* pColor;
	PCFile << std::scientific;
	for (int i = 0; i <nDepthWidth * nDepthHeight; i++)
	{
		CameraSpacePoint cameraP = m_pCameraCoordinates[i];
		ColorSpacePoint colorP = m_pColorCoordinates[i];
		rgb = 0;
		if (invalidColorSpace(colorP))
			rgb = 0;
		else{
			int x = (int)(colorP.X + 0.5);
			int y = (int)(colorP.Y + 0.5);
			pColor = (pColorBuffer + x + y * nColorWidth);
			rgb = (static_cast<UINT32>(pColor->rgbRed) << 16 | static_cast<UINT32>(pColor->rgbGreen)<<8 | static_cast<UINT32>(pColor->rgbBlue));
		}
		PCFile << cameraP.X << "\t" << cameraP.Y << "\t" << cameraP.Z << "\t";
		PCFile << *reinterpret_cast<float*>(&rgb) << std::endl;
	}
	PCFile.close();
}
void CCoordinateMappingBasics::writePointCloudSharedMemory(const UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight,
	RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight,
	const BYTE* pBodyIndexBuffer, int nBodyIndexWidth, int nBodyIndexHeight)
{
	//updatet the color image video
	m_pDrawCoordinateMapping->Draw(reinterpret_cast<BYTE*>(pColorBuffer), cColorWidth*cColorHeight * sizeof(RGBQUAD));
	HRESULT hr;
	int pcType = *(reinterpret_cast<int*>(m_pBuf));
	float* pXYZ = reinterpret_cast<float*>(m_pBuf)+4;
	UINT32* pRGB = reinterpret_cast<UINT32*>(m_pBuf)+7;
	//now try just mapping the valid depth point with rgb information to the camera space
	hr = m_pCoordinateMapper->MapDepthFrameToCameraSpace(nDepthWidth*nDepthHeight, (UINT16*)pDepthBuffer, nDepthWidth*nDepthHeight, m_pCameraCoordinates);
	if (pcType == 0){
		//write the xyz point cloud data into shared memory
		for (int i = 0; i < nDepthWidth * nDepthHeight; i++){
			memcpy(pXYZ, m_pCameraCoordinates + i, sizeof(float)* 3);
			pXYZ += 4;//
		}
	}
	else{
		hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(nDepthWidth*nDepthHeight, (UINT16*)pDepthBuffer, nDepthWidth*nDepthHeight, m_pColorCoordinates);
		for (int i = 0; i < nDepthWidth * nDepthHeight; i++){
			memcpy(pXYZ, m_pCameraCoordinates + i, sizeof(float)* 3);
			ColorSpacePoint colorP = m_pColorCoordinates[i];
			if (invalidColorSpace(colorP))
				*pRGB = 0;
			else{
				int x = (int)(colorP.X + 0.5);
				int y = (int)(colorP.Y + 0.5);
				RGBQUAD* pColor = (pColorBuffer + x + y * nColorWidth);
				*pRGB = (static_cast<UINT32>(pColor->rgbRed) << 16 | static_cast<UINT32>(pColor->rgbGreen) << 8 | static_cast<UINT32>(pColor->rgbBlue));
			}
			pXYZ += 4;
			pRGB += 4;
		}
	}
}
void CCoordinateMappingBasics::MLQMappingSimilarity(vector<ColorSpacePoint>& originalP, vector<ColorSpacePoint>& mapping, vector<ColorSpacePoint>& deformaion)
{
	int sizeofMapping = mapping.size() / 2;
	if (sizeofMapping <= 0 || mapping.size()%2)
	{
		return;
	}
	//compute the corresponding position for every point
	for (int i = 0; i < originalP.size();i++)
	{
		ColorSpacePoint v = originalP[i];
		//compute the p* and q* for this point
		ColorSpacePoint p_star = { 0 }, q_star = { 0 };
		float sumWright = 0;
		for (int i = 0; i < mapping.size(); i+= 2)
		{
			float weightI = weightForVP(v, mapping[i]);
			p_star = addCSPoint(p_star, multiCSPoint(mapping[i],weightI));
			q_star = addCSPoint(q_star, multiCSPoint(mapping[i + 1], weightI));
			sumWright += weightI;
		}
		p_star = multiCSPoint(p_star, 1 / sumWright);
		q_star = multiCSPoint(q_star, 1 / sumWright);
		//compute the mus
		float mus = 0;
		for (int i = 0; i < mapping.size();i+=2)
		{
			float w = weightForVP(v, mapping[i]);
			mus += w*scatterofCSPoint(subCSPoint(mapping[i], p_star));
		}
		float matrixP[4], matrixVP[4], Matrixtemp[4], A[4] = { 0 };
		ColorSpacePoint qDelt = { 0 };
		for (int i = 0; i < mapping.size();i+= 2)
		{
			float w = weightForVP(v, mapping[i]);
			ColorSpacePoint pCaret = subCSPoint(mapping[i], p_star);
			ColorSpacePoint qCaret = subCSPoint(mapping[i + 1], q_star);
			ColorSpacePoint vpstar = subCSPoint(v, p_star);
			matrixP[0] = pCaret.X; matrixP[1] = pCaret.Y;
			matrixP[2] = pCaret.Y; matrixP[3] = -pCaret.X;
			matrixVP[0] = vpstar.X; matrixVP[1] = vpstar.Y;
			matrixVP[2] = vpstar.Y; matrixVP[3] = -vpstar.X;
			matrix2x2Multi(A, matrixP, matrixVP);
			matrix2x2Scale(A, w/mus);
			qDelt = addCSPoint(qDelt, point2dXmatrix(qCaret, A));
		}
		ColorSpacePoint cp = addCSPoint(qDelt, q_star);
		cp.X = (int)(cp.X + 0.5);
		cp.Y = (int)(cp.Y + 0.5);
		deformaion.push_back(cp);
	}
}
void CCoordinateMappingBasics::MLQMappingRigid(vector<ColorSpacePoint>& originalP, vector<ColorSpacePoint>& mapping, vector<ColorSpacePoint>& deformaion)
{
	int sizeofMapping = mapping.size() / 2;
	if (sizeofMapping <= 0 || mapping.size() % 2)
	{
		return;
	}
	//compute the corresponding position for every point
	for (int i = 0; i < originalP.size(); i++)
	{
		ColorSpacePoint v = originalP[i];
		//compute the p* and q* for this point
		ColorSpacePoint p_star = { 0 }, q_star = { 0 };
		float sumWright = 0;
		for (int i = 0; i < mapping.size(); i += 2)
		{
			float weightI = weightForVP(v, mapping[i]);
			p_star = addCSPoint(p_star, multiCSPoint(mapping[i], weightI));
			q_star = addCSPoint(q_star, multiCSPoint(mapping[i + 1], weightI));
			sumWright += weightI;
		}
		p_star = multiCSPoint(p_star, 1 / sumWright);
		q_star = multiCSPoint(q_star, 1 / sumWright);
		
		float matrixP[4], matrixVP[4], Matrixtemp[4], A[4] = { 0 };
		ColorSpacePoint frv = { 0 };
		ColorSpacePoint vpstar = subCSPoint(v, p_star);
		for (int i = 0; i < mapping.size(); i += 2)
		{
			float w = weightForVP(v, mapping[i]);
			ColorSpacePoint pCaret = subCSPoint(mapping[i], p_star);
			ColorSpacePoint qCaret = subCSPoint(mapping[i + 1], q_star);
			matrixP[0] = pCaret.X; matrixP[1] = pCaret.Y;
			matrixP[2] = pCaret.Y; matrixP[3] = -pCaret.X;
			matrixVP[0] = vpstar.X; matrixVP[1] = vpstar.Y;
			matrixVP[2] = vpstar.Y; matrixVP[3] = -vpstar.X;
			matrix2x2Multi(A, matrixP, matrixVP);
			matrix2x2Scale(A, w);
			frv = addCSPoint(frv, point2dXmatrix(qCaret, A));
		}
		frv = multiCSPoint(frv, 1/sqrt(scatterofCSPoint(frv)));
		float lengthPdelt = sqrt(scatterofCSPoint(vpstar));
		frv = multiCSPoint(frv, lengthPdelt);
		ColorSpacePoint cp = addCSPoint(frv, q_star);
		cp.X = (int)(cp.X + 0.5);
		cp.Y = (int)(cp.Y + 0.5);
		deformaion.push_back(cp);
	}
}
float CCoordinateMappingBasics::weightForVP(ColorSpacePoint v, ColorSpacePoint p)
{
	float alpha = 1;
	return 1.0f / (pow(v.X - p.X, 2 * alpha)+pow(v.Y - p.Y,2*alpha));
}
void CCoordinateMappingBasics::MLQMapping(vector<ColorSpacePoint>& originalP, vector<ColorSpacePoint>& mapping, vector<ColorSpacePoint>& deformaion, int type)
{
	//default Affine
	int sizeOfMapping = mapping.size()/2;
	if (sizeOfMapping <= 0 || mapping.size() % 2)
	{
		//MessageBox(NULL, L"size of mapping is wrong", L"Error", MB_OK);
		return;
	}
	for (int i = 0; i < originalP.size(); i++)
	{
		ColorSpacePoint v = originalP[i];
		//compute the p_star and q_star
		float p_star_x = 0; float p_star_y = 0;
		float q_star_x = 0; float q_star_y = 0;
		//weights are equal for every point
		float sumWp = 0, sumWq = 0;
		for (vector<ColorSpacePoint>::iterator it = mapping.begin(); it < mapping.end(); ++it){
			float wp = 1/(pow((it->X - v.X), 2) + pow(it->Y - v.Y, 2));
			p_star_x += wp* it->X; p_star_y += wp* it->Y;
			sumWp += wp;
			it++;
			q_star_x += wp* it->X; q_star_y += wp*it->Y;
		}
		p_star_x /= sumWp; p_star_y /= sumWp;
		q_star_x /= sumWp; q_star_y /= sumWp;
		//compute the matrix
		// | m0 m1|
		// | m2 m3| 
		float m0, m1, m2, m3;
		m0 = m1 = m2 = m3 = 0;
		for (vector<ColorSpacePoint>::iterator it = mapping.begin(); it < mapping.end(); it += 2){
			float wp = 1/(pow((it->X - v.X), 2) + pow(it->Y - v.Y, 2));
			float x = it->X - p_star_x;
			float y = it->Y - p_star_y;
			m0 += wp*x*x; m1 += wp*x*y; m2 += wp*x* y; m3 += wp*y*y;
		}
		//inverse of the matrix
		float scale = 1 / (m0 * m3 - m1*m2);
		float m00 = m3*scale;
		float m01 = -m1*scale;
		float m10 = -m2*scale;
		float m11 = m0 * scale;
		float vpstarx = v.X - p_star_x;
		float vpstary = v.Y - p_star_y;
		float xq = 0, yq =0;
		for (int j = 0; j < mapping.size(); j+= 2){
			float pjtx = mapping[j].X - p_star_x;
			float pjty = mapping[j].Y - p_star_y;
			float scale = (vpstarx * m00 + vpstary * m10)*pjtx + (vpstarx * m01 + vpstary*m11)*pjty;
			xq += scale * (mapping[j + 1].X - q_star_x);
			yq += scale * (mapping[j + 1].Y - q_star_y);
		}
		ColorSpacePoint res;
		res.X = xq + q_star_x;
		res.Y = yq + q_star_y;
		deformaion.push_back(res);
	}

}

template<class T> void CCoordinateMappingBasics::smoothDepthImage(T* pDepthBuffer, int nDepthWidth, int nDepthHeight, int windowSize)
{
	T* pDepthTemp = new T[nDepthWidth*nDepthHeight];
	memcpy(pDepthTemp, pDepthBuffer, sizeof(T)*nDepthWidth*nDepthHeight);
	int halfWS = windowSize / 2;
	std::vector<T> depthWindow;
	depthWindow.resize(windowSize*windowSize);
	//*pDepth < cMinDepthDistance || *pDepth > cMaxDepthDistance
	for (int j = halfWS; j < nDepthHeight - halfWS; j++)
	{
		for (int i = halfWS; i < nDepthWidth - halfWS; i++)
		{
			int vecIndex = 0; 
			T* pDepth = pDepthBuffer + j * nDepthWidth + i;
			T* pDepthT = pDepthTemp + j * nDepthWidth + i;
			for (int h = -halfWS; h <= halfWS; h++)
			{
				for (int w = -halfWS; w <= halfWS; ++w)
				{
					T depth = *(pDepthT + h * nDepthWidth + w);
					if (depth > 0){
						depthWindow[vecIndex++] = depth;
					}
				}
			}
			//sort the depth in the selected window
			std::sort(depthWindow.begin(), depthWindow.begin()+vecIndex);
			*pDepth = depthWindow[vecIndex / 2];
		}
	}
}

void CCoordinateMappingBasics::filterCameraSpacePoints(CameraSpacePoint* pCameraSpacePoints, int width, int height, int windowSize)
{
	int pointsOfDepth = width * height;
	float* pDepth = new float[pointsOfDepth];
	//filter the new point cloud, some point should be removed
	m_pCoordinateMapper->MapCameraPointsToDepthSpace(width*height, pCameraSpacePoints, width*height, m_pDepthCoordiantes);
	//memset(m_pDepthBuffer, 0, sizeof(UINT16)* pointsOfDepth);
	for (int i = 0; i < pointsOfDepth; ++i)
		pDepth[i] = 0.0;
	//generate the new depth map and filter the new occluded points.
	for (int i = 0; i < pointsOfDepth; ++i){
		int x = m_pDepthCoordiantes[i].X + 0.5;
		int y = m_pDepthCoordiantes[i].Y + 0.5;
		int index = x + y* width;
		//generate the depth map
		if (index >= 0 && index < pointsOfDepth){
			if (int(pDepth[index]) == 0)
				pDepth[index] = m_pCameraCoordinates[i].Z;
			else if (pDepth[index] > m_pCameraCoordinates[i].Z)
				pDepth[index] = m_pCameraCoordinates[i].Z;
		}
	}
	smoothDepthImage(pDepth, width, height, windowSize);
	int count = 0;
	for (int i = 0; i < pointsOfDepth; ++i){
		int x = m_pDepthCoordiantes[i].X + 0.5;
		int y = m_pDepthCoordiantes[i].Y + 0.5;
		int index = x + y* width;
		if (index >= 0 && index < pointsOfDepth){
			if (pDepth[index] *1.01 < m_pCameraCoordinates[i].Z){
				//remove this points
				m_pCameraCoordinates[i].X = 0;
				m_pCameraCoordinates[i].Z = 0;
				count++;
			}
		}
	}
	/*if (count)
	{
		std::wstring content = std::to_wstring(count);
		MessageBox(NULL, content.c_str(), content.c_str(), MB_OK);
	}*/
	delete[] pDepth;
}

void CCoordinateMappingBasics::correctedGazeofStaticImage()
{
	if (m_hWnd)
	{
		double fps = 0.0;

		LARGE_INTEGER qpcNow = { 0 };
		if (m_fFreq)
		{
			if (QueryPerformanceCounter(&qpcNow))
			{
				if (m_nLastCounter)
				{
					m_nFramesSinceUpdate++;
					fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
				}
			}
		}

		WCHAR szStatusMessage[64];
		StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L"T:%0.2f %0.2f %0.2f R:%0.2f %0.2f %0.2f FPS = %0.2f    Time = %I64d", m_cameraTranslation.x(), m_cameraTranslation.y(), m_cameraTranslation.z(), m_rotationX, m_rotationY, m_rotationZ, fps, (0 - m_nStartTime));

		if (SetStatusMessage(szStatusMessage, 1000, false))
		{
			m_nLastCounter = qpcNow.QuadPart;
			m_nFramesSinceUpdate = 0;
		}
	}
	/*if (m_computeDeformation == false){
		return;
	}
	m_computeDeformation = false;*/
	//load background
	loadDepthColorImg("backGround", m_pBackgroundRGBX, cColorWidth, cColorHeight, m_pDepthBuffer, cDepthWidth, cDepthHeight);
	loadDepthColorImg("user", m_pColorBufferSrc, cColorWidth, cColorHeight, m_pDepthBufferSrc, cDepthWidth, cDepthHeight);
	removeBackground(m_pColorBufferSrc, m_pBackgroundRGBX, m_pDepthBufferSrc, m_pDepthBuffer,700,1150);
	createMapping();
	//doDeformation();
	//generateNewImage();
	doInterpolationMLSDeformation(m_pOutputRGBX, m_pColorBufferSrc);
}
void CCoordinateMappingBasics::createMapping()
{
	m_pCoordinateMapper->MapDepthFrameToColorSpace(cDepthWidth*cDepthHeight, m_pDepthBufferSrc, cDepthWidth*cDepthHeight, m_pColorCoordinates);

	//mapping from depth to camera space and generate the point clouds file to find the key points
	m_pCoordinateMapper->MapDepthFrameToCameraSpace(cDepthWidth*cDepthHeight, m_pDepthBufferSrc, cDepthWidth*cDepthHeight, m_pCameraCoordinates);

	for (int depthIndex = 0; depthIndex < (cDepthWidth*cDepthHeight); ++depthIndex){
		//m_pCameraCoordinates[depthIndex].Y -= 0.2;
		Eigen::Translation3d point;
		point.x() = m_pCameraCoordinates[depthIndex].X;
		point.y() = m_pCameraCoordinates[depthIndex].Y;
		point.z() = m_pCameraCoordinates[depthIndex].Z;
		Eigen::Affine3d temp = m_cameraTransformation * point;
		m_pCameraCoordinates[depthIndex].X = temp.translation().x();
		m_pCameraCoordinates[depthIndex].Y = temp.translation().y();
		m_pCameraCoordinates[depthIndex].Z = temp.translation().z();
	}
	//virtual camera center
	HRESULT hr = m_pCoordinateMapper->MapCameraPointsToColorSpace(cDepthWidth*cDepthHeight, m_pCameraCoordinates, cDepthWidth*cDepthHeight, m_pColorCoordinatesDeformation);

	//to check if the color space mapping is valid
	int noValid = 0;
	memset(m_pBothColorSpaceValid, 0, cDepthWidth * cDepthHeight * sizeof(bool)); //reset
	memset(m_pValidColorMapping, 0, cColorWidth*cColorHeight*sizeof(bool));
	memset(m_pColorCoordinatesColor, 0, cColorHeight*cColorWidth*sizeof(ColorSpacePoint));

	for (int i = 0; i < cDepthWidth* cDepthHeight; i++){
		if (invalidColorSpace(m_pColorCoordinates[i]) || invalidColorSpace(m_pColorCoordinatesDeformation[i]))
			m_pBothColorSpaceValid[i] = false;
		else{
			//update the mapping information
			//we could switch this to check the result, forward warping or back warping
			int dx = (int)(m_pColorCoordinatesDeformation[i].X + 0.5);
			int dy = (int)(m_pColorCoordinatesDeformation[i].Y + 0.5);
			int x = (int)(m_pColorCoordinates[i].X + 0.5);
			int y = (int)(m_pColorCoordinates[i].Y + 0.5);
			if (m_colorSpaceStatus[x + y * cColorWidth])
			{
				m_pColorCoordinatesColor[x + y * cColorWidth].X = dx;
				m_pColorCoordinatesColor[x + y * cColorWidth].Y = dy;
				m_pValidColorMapping[x + y *cColorWidth] = true;
				m_pBothColorSpaceValid[i] = true;
				m_colorSpaceStatus[x + y * cColorWidth] = 2;
				noValid++;
			}
		}
	}
}
void CCoordinateMappingBasics::doDeformation()
{
	//do the deformation 
	vector<ColorSpacePoint> originalP, mapping, deformaion;
	originalP.clear(); mapping.clear(); deformaion.clear();
	int heightStep = 20;
	int heightExtern = 5;
	ColorSpacePoint temp;
	//first turn 
	for (int jstart = 0; jstart < cColorHeight; jstart+= heightStep)
	{
		originalP.clear(); mapping.clear(); deformaion.clear();
		int hStart = (jstart - heightExtern) < 0 ? 0 : (jstart - heightExtern);
		int hEnd = (jstart + heightStep + heightExtern) > cColorHeight ? cColorHeight : (jstart + heightStep + heightExtern);

		for (int i = hStart; i < hEnd; i ++)
		{
			for (int j = 0; j < cColorWidth; j ++){
				int index = j + i * cColorWidth;
				temp.X = j; temp.Y = i;
				if (m_pValidColorMapping[index]){
					mapping.push_back(temp);
					mapping.push_back(m_pColorCoordinatesColor[index]);
				}
			}
		}
		hEnd = (jstart + heightStep) > cColorHeight ? cColorHeight : (jstart + heightStep);
		for (int i = jstart; i < hEnd; i++)
		{
			for (int j = 0; j < cColorWidth; j++){
				int index = j + i * cColorWidth;
				temp.X = j; temp.Y = i;
				if (m_colorSpaceStatus[index] ==1){
					originalP.push_back(temp);
				}
			}
		}
		if (mapping.size() < 10)
			continue;
		MLQMappingRigid(originalP, mapping, deformaion);
		for (int i = 0; i < deformaion.size(); i++){
			int dIndex = deformaion[i].X + deformaion[i].Y * cColorWidth;
			if (dIndex < 0 || dIndex >= cColorHeight * cColorWidth)
				continue;
			int index = originalP[i].X + originalP[i].Y * cColorWidth;
			m_pColorCoordinatesColor[index] = deformaion[i];
		}
	}
}
void CCoordinateMappingBasics::doInterpolationMLSDeformation(RGBQUAD* &pDeformedImage, RGBQUAD* &pSrcImage)
{
	if (!m_enableSimpleInterpolation)
		return;
	//do simple interpolation according to the target color image
	int dIndexUL, dIndexUR, dIndexDL, dIndexDR;
	int lineStart = 0;
	int MAXInterpolationStep = 10;
	for (int line = 0; line < cColorHeight - 1; ++line){
		for (int index = 0; index < cColorWidth - 1; ++index){
			dIndexUL = lineStart + index;
			if (!m_pValidColorMapping[dIndexUL])
				continue;
			bool finished = false;
			for (int j = 1; j < MAXInterpolationStep && finished == false; j++)
			for (int i = 1; i < MAXInterpolationStep && finished == false; i++){
				dIndexUL = dIndexUL + 0; dIndexUR = dIndexUL + j;
				dIndexDL = dIndexUL + i*cColorWidth; dIndexDR = dIndexUL + i*cColorWidth + j;
				if (dIndexDR > cColorWidth* cColorHeight || dIndexUR - lineStart > cColorWidth - 1)
					finished = true;
				ColorSpacePoint cUL = { index, line };
				ColorSpacePoint cUR = { index + j, line };
				ColorSpacePoint cDL = { index, line + i };
				ColorSpacePoint cDR = { index + j, line + i };
				if (m_pValidColorMapping[dIndexUR] && m_pValidColorMapping[dIndexDL] && m_pValidColorMapping[dIndexDR]){
					colorDeformationBackMapping(pDeformedImage, pSrcImage, cColorWidth, cColorHeight, cUL, cUR, cDL, cDR, \
						m_pColorCoordinatesColor[dIndexUL], m_pColorCoordinatesColor[dIndexUR], m_pColorCoordinatesColor[dIndexDL], m_pColorCoordinatesColor[dIndexDR]);
					finished = true;
				}
			}
		}
		lineStart += cColorWidth;
	}
	//do MLS interpolation 
	if (!m_enableMLSDeformation)
		return;
	m_enableMLSDeformation = false;
	m_enableSimpleInterpolation = false;
	vector<ColorSpacePoint> originalP, mapping, deformaion;
	originalP.clear(); mapping.clear(); deformaion.clear();
	int widthStep = 10, heightStep = 5;
	ColorSpacePoint temp;
	//every small window to deform the image
	for (int h = 0; h < cColorHeight; h += heightStep)
	{
		for (int w = 0; w < cColorWidth; w++)
		{
			//int w = 0;
			int orginalIndex = w + h * cColorWidth;
			if (m_pValidColorMapping[orginalIndex])
				continue;
			originalP.clear(); mapping.clear(); deformaion.clear();
			ColorSpacePoint temp;
			for (int i = w; i <= w + widthStep && i < cColorWidth; i++){
				for (int j = h; j <= h + heightStep && j < cColorHeight; j++){
					int index = i + j * cColorWidth;
					if (index >= cColorWidth* cColorHeight)
						continue;
					temp.X = i; temp.Y = j;
					if (m_pValidColorMapping[index]){
						mapping.push_back(temp);
						mapping.push_back(m_pColorCoordinatesColor[index]);
						break;
					}
					else
					{
						originalP.push_back(temp);
					}
				}
				for (int j = h; j >= h - heightStep && j >= 0; j--){
					int index = i + j * cColorWidth;
					if (index >= cColorWidth* cColorHeight)
						continue;
					temp.X = i; temp.Y = j;
					if (m_pValidColorMapping[index]){
						mapping.push_back(temp);
						mapping.push_back(m_pColorCoordinatesColor[index]);
						break;
					}
					else
					{
						originalP.push_back(temp);
					}
				}
			}
			if (mapping.size() < 4){
				//h += heightStep - 1;
				continue;
			}
			MLQMappingRigid(originalP, mapping, deformaion);
			for (int i = 0; i < deformaion.size(); i++){
				int dIndex = deformaion[i].X + deformaion[i].Y * cColorWidth;
				if (dIndex < 0 || dIndex >= cColorHeight * cColorWidth)
					continue;
				int index = originalP[i].X + originalP[i].Y * cColorWidth;
				m_pColorCoordinatesColor[index] = deformaion[i];
				m_pValidColorMapping[index] = true;
				pDeformedImage[index] = pSrcImage[dIndex];
			}
		}
	}
}
void CCoordinateMappingBasics::generateNewImage()
{
	memcpy(m_originalRGBX, m_pColorBufferSrc, cColorWidth*cColorHeight * sizeof(RGBQUAD));
	for (int i = 0; i < cDepthWidth*cDepthHeight; i++)
	{
		if (m_pBothColorSpaceValid[i]){
			ColorSpacePoint cp = m_pColorCoordinates[i];
			int x = (int)(cp.X + 0.5);
			int y = (int)(cp.Y + 0.5);
			RGBQUAD* c = m_originalRGBX + x + y * cColorWidth;
			c->rgbBlue = 0;
			c->rgbGreen = 255;
			c->rgbRed = 2;
		}
	}
	m_pDrawCoordinateMapping->Draw(reinterpret_cast<BYTE*>(m_pColorBufferSrc), cColorWidth*cColorHeight * sizeof(RGBQUAD));

	memset(m_pOutputRGBX, 0, cColorWidth*cColorHeight*sizeof(RGBQUAD));
	for (int i = 0; i < cColorWidth * cColorHeight; i++)
	{
		if (m_colorSpaceStatus[i] == 0)
			continue;
		int index = m_pColorCoordinatesColor[i].X + m_pColorCoordinatesColor[i].Y * cColorWidth;
		if (index < 0 || index >= cColorHeight * cColorWidth)
			continue;
		//m_pOutputRGBX[index] = pColorBuffer[i];
		m_pOutputRGBX[index] = m_pColorBufferSrc[i];
	}
	m_pDrawDeformation->Draw(reinterpret_cast<BYTE*>(m_pOutputRGBX), cColorWidth*cColorHeight * sizeof(RGBQUAD));
}
void CCoordinateMappingBasics::removeBackground(RGBQUAD* &pUserColor, RGBQUAD* pBGColor, UINT16* &pUserDepth, UINT16* pBGDepth,int left, int right)
{
	//clear the same pixel as the background
	//UINT32 * pUColor = reinterpret_cast<UINT32*>(pUserColor);
	//UINT32 * pBColor = reinterpret_cast<UINT32*>(pBGColor);
	memset(m_colorSpaceStatus, 0, sizeof(int)*cColorWidth*cColorHeight);
	const int difT = 15;
	for (int h = 0; h < cColorHeight; ++h)
	for (int w = 0; w < left; ++w)
	{
		int i = w + h * cColorWidth;
		pUserColor[i].rgbRed = 0;
		pUserColor[i].rgbGreen = 0;
		pUserColor[i].rgbBlue = 0;

	}
	for (int h = 0; h < cColorHeight; ++h)
	for (int w = cColorWidth-1; w > right; --w)
	{
		int i = w + h * cColorWidth;
		pUserColor[i].rgbRed = 0;
		pUserColor[i].rgbGreen = 0;
		pUserColor[i].rgbBlue = 0;

	}
	for (int h = 0; h < cColorHeight; ++h)
	for (int w = left; w < right; ++w)
	{
		int i = w + h * cColorWidth;
		if (abs(pUserColor[i].rgbRed - pBGColor[i].rgbRed) < difT && abs(pUserColor[i].rgbGreen - pBGColor[i].rgbGreen) < difT && abs(pUserColor[i].rgbBlue - pBGColor[i].rgbBlue) < difT){
			pUserColor[i].rgbRed = 0;
			pUserColor[i].rgbGreen = 0;
			pUserColor[i].rgbBlue = 0;
		}
		else{
			m_colorSpaceStatus[i] = 1;
		}
	}
}
void CCoordinateMappingBasics::loadDepthColorImg(char* frameName, RGBQUAD* &pColorBuffer, int nColorWidth, int nColorHeight, UINT16* &pDepthBuffer, int nDepthWidth, int nDepthHeight)
{
	std::string cFileName = std::string(frameName) + "Color";
	std::string dFileName = std::string(frameName) + "Depth";
	ifstream iFile;
	iFile.open(cFileName.c_str(), std::ios::binary);
	if (iFile.good()){
		iFile.read(reinterpret_cast<char*>(pColorBuffer), sizeof(RGBQUAD)*nColorWidth* nColorHeight);
	}
	iFile.close();
	iFile.open(dFileName.c_str(), std::ios::binary);
	if (iFile.good()){
		iFile.read(reinterpret_cast<char*>(pDepthBuffer), sizeof(UINT16)*nDepthWidth*nDepthHeight);
	}
	iFile.close();
}
void CCoordinateMappingBasics::saveDepthColorImg(char* frameName, RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight, UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight)
{
	std::string cFileName = std::string(frameName) + "Color";
	std::string dFileName = std::string(frameName) + "Depth";
	ofstream cFile;
	cFile.open(cFileName.c_str(), std::ios::binary);
	if (cFile.good())
	{
		cFile.write(reinterpret_cast<char*> (pColorBuffer), sizeof(RGBQUAD)*nColorWidth * nColorHeight);
	}
	else{
		MessageBox(NULL, L"Error when opening color image", NULL, MB_OK);
	}
	cFile.close();
	ofstream dFile;
	dFile.open(dFileName.c_str(), std::ios::binary);
	if (dFile.good()){
		dFile.write(reinterpret_cast<char*>(pDepthBuffer), sizeof(UINT16)*nDepthWidth*nDepthHeight);
	}
	else{
		MessageBox(NULL, L"Error When opening depth image", NULL, MB_OK);
	}
	dFile.close();
}
/// <summary>
/// Handle new depth and color data
/// <param name="nTime">timestamp of frame</param>
/// <param name="pDepthBuffer">pointer to depth frame data</param>
/// <param name="nDepthWidth">width (in pixels) of input depth image data</param>
/// <param name="nDepthHeight">height (in pixels) of input depth image data</param>
/// <param name="pColorBuffer">pointer to color frame data</param>
/// <param name="nColorWidth">width (in pixels) of input color image data</param>
/// <param name="nColorHeight">height (in pixels) of input color image data</param>
/// <param name="pBodyIndexBuffer">pointer to body index frame data</param>
/// <param name="nBodyIndexWidth">width (in pixels) of input body index data</param>
/// <param name="nBodyIndexHeight">height (in pixels) of input body index data</param>
/// </summary>
void CCoordinateMappingBasics::ProcessFrame(INT64 nTime, 
                                            const UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight, 
                                            const RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight,
                                            const BYTE* pBodyIndexBuffer, int nBodyIndexWidth, int nBodyIndexHeight)
{
    if (m_hWnd)
    {
        if (!m_nStartTime)
        {
            m_nStartTime = nTime;
        }

        double fps = 0.0;

        LARGE_INTEGER qpcNow = {0};
        if (m_fFreq)
        {
            if (QueryPerformanceCounter(&qpcNow))
            {
                if (m_nLastCounter)
                {
                    m_nFramesSinceUpdate++;
                    fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
                }
            }
        }

        WCHAR szStatusMessage[64];
        StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L"T:%0.2f %0.2f %0.2f R:%0.2f %0.2f %0.2f FPS = %0.2f    Time = %I64d",m_cameraTranslation.x(),m_cameraTranslation.y(),m_cameraTranslation.z(),m_rotationX,m_rotationY,m_rotationZ, fps, (nTime - m_nStartTime));

        if (SetStatusMessage(szStatusMessage, 1000, false))
        {
            m_nLastCounter = qpcNow.QuadPart;
            m_nFramesSinceUpdate = 0;
        }
    }
	
    // Make sure we've received valid data
    if (m_pCoordinateMapper && m_pColorCoordinates && m_pOutputRGBX && 
        pDepthBuffer && (nDepthWidth == cDepthWidth) && (nDepthHeight == cDepthHeight) && 
        pColorBuffer && (nColorWidth == cColorWidth) && (nColorHeight == cColorHeight) &&
        pBodyIndexBuffer && (nBodyIndexWidth == cDepthWidth) && (nBodyIndexHeight == cDepthHeight))
    {
		//first step: processing the depth information to fill the hole
		memcpy(m_pDepthBuffer, pDepthBuffer, cDepthWidth*cDepthHeight*sizeof(UINT16));
		for (int y = 1; y < cDepthHeight - 1; ++y){
			for (int x = 1; x < cDepthWidth - 1; ++x){
				const UINT16* pDepth = pDepthBuffer + y * cDepthWidth + x;
				if (*pDepth < cMinDepthDistance || *pDepth > cMaxDepthDistance){
					//find the average of the eight adjacencys
					UINT16 sum = 0;
					int count = 0;
					for (int j = -1; j <= 1; j++){
						for (int i = -1; i <= 1; i++){
							UINT16 temp = *(pDepth + j*cDepthWidth + i);
							if (temp < cMinDepthDistance || temp > cMaxDepthDistance)
								continue;
							sum += temp;
							count++;
						}
					}
					*(m_pDepthBuffer + y * cDepthWidth + x) = (count == 0) ? 0 : (sum / count);
				}
			}
		}
		//update drawing
		m_pDrawCoordinateMapping->Draw(reinterpret_cast<BYTE*>(m_originalRGBX), cColorWidth*cColorHeight * sizeof(RGBQUAD));
		m_pDrawDeformation->Draw(reinterpret_cast<BYTE*>(m_pOutputRGBX), cColorWidth*cColorHeight * sizeof(RGBQUAD));
		//memcpy(m_pOutputRGBX, pColorBuffer, cColorWidth*cColorHeight * sizeof(RGBQUAD));
		//m_pDrawCoordinateMapping->Draw(reinterpret_cast<BYTE*>(m_pOutputRGBX), cColorWidth*cColorHeight * sizeof(RGBQUAD));
		if (!m_computeDeformation)
			return;
		m_computeDeformation = false;
		//original image and depth information
		HRESULT hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(nDepthWidth * nDepthHeight, (UINT16*)m_pDepthBuffer, nDepthWidth * nDepthHeight, m_pColorCoordinates);
		//memcpy(m_pColorCoordinates, m_pColorCoordinatesTemp, cDepthWidth*cDepthHeight*sizeof(ColorSpacePoint));
		//fillTheHoleInColorspace(m_pColorCoordinates, m_pColorCoordinatesTemp);
		
		//}
		//mapping from depth to camera space and generate the point clouds file to find the key points
		hr = m_pCoordinateMapper->MapDepthFrameToCameraSpace(nDepthWidth*nDepthHeight, (UINT16*)pDepthBuffer, nDepthWidth * nDepthHeight, m_pCameraCoordinates);
		if (SUCCEEDED(hr)){
			std::cout << "mapping from depth to camera space Successful" << std::endl;
			//MessageBox(NULL, L"mapping from depth to camera space", L"Successful", MB_OK);
			//apply a transformation to the piont clouds, this transfromatin should be the inversion of the transformation from current camera center to the virtual camera center
			// the simplest situation, the virtual camera centor is 20cm belower
			for (int depthIndex = 0; depthIndex < (nDepthWidth * nDepthHeight); ++depthIndex){
				//m_pCameraCoordinates[depthIndex].Y -= 0.2;
				Eigen::Translation3d point;
				point.x() = m_pCameraCoordinates[depthIndex].X;
				point.y() = m_pCameraCoordinates[depthIndex].Y;
				point.z() = m_pCameraCoordinates[depthIndex].Z;
				Eigen::Affine3d temp = m_cameraTransformation * point;
				m_pCameraCoordinates[depthIndex].X = temp.translation().x();
				m_pCameraCoordinates[depthIndex].Y = temp.translation().y();
				m_pCameraCoordinates[depthIndex].Z = temp.translation().z();
			}
		}
		else{

			//MessageBox(NULL, L"mapping from depth to camera space", L"Failed", MB_OK);
		}
		//virtual camera center
		hr = m_pCoordinateMapper->MapCameraPointsToColorSpace(nDepthWidth * nDepthHeight, m_pCameraCoordinates, nDepthWidth * nDepthHeight, m_pColorCoordinatesDeformation);
		//memcpy(m_pColorCoordinatesDeformation, m_pColorCoordinatesTemp, cDepthWidth*cDepthHeight*sizeof(ColorSpacePoint));
		//fillTheHoleInColorspace(m_pColorCoordinatesDeformation, m_pColorCoordinatesTemp);
		//memcpy(m_pOutputRGBX,pColorBuffer, cColorWidth*cColorHeight * sizeof(RGBQUAD));
		
		//to check if the color space mapping is valid
		int noValid = 0;
		memset(m_pBothColorSpaceValid, 0, cDepthWidth * cDepthHeight * sizeof(bool)); //reset
		memset(m_pValidColorMapping, 0, cColorWidth*cColorHeight*sizeof(bool));
		memset(m_pColorCoordinatesColor, 0, cColorHeight*cColorWidth*sizeof(ColorSpacePoint));

		for (int i = 0; i < cDepthWidth* cDepthHeight; i++){
			if (invalidColorSpace(m_pColorCoordinates[i]) || invalidColorSpace(m_pColorCoordinatesDeformation[i]))
				m_pBothColorSpaceValid[i] = false;
			else{
				m_pBothColorSpaceValid[i] = true;
				noValid++;
				//update the mapping information
				//we could switch this to check the result, forward warping or back warping
				int x = (int)(m_pColorCoordinatesDeformation[i].X + 0.5);
				int y = (int)(m_pColorCoordinatesDeformation[i].Y + 0.5);
				int dx = (int)(m_pColorCoordinates[i].X + 0.5);
				int dy = (int)(m_pColorCoordinates[i].Y + 0.5);
				m_pColorCoordinatesColor[x + y * cColorWidth].X = dx;
				m_pColorCoordinatesColor[x + y * cColorWidth].Y = dy;
				m_pValidColorMapping[x + y *cColorWidth] = true;
			}
		}
		//do the deformation 
		vector<ColorSpacePoint> originalP, mapping, deformaion;
		originalP.clear(); mapping.clear(); deformaion.clear();
		int widthStep = 40, heightStep = 40;
		ColorSpacePoint temp;
		//first turn 
		for (int istart = 0; istart < widthStep; istart++)
		for (int jstart = 0; jstart < heightStep; jstart++)
		{
			originalP.clear(); mapping.clear(); deformaion.clear();
			for (int i = 0; i < cColorHeight; i += heightStep)
			{
				for (int j = 0; j < cColorWidth; j += widthStep){
					int index = j + i * cColorWidth;
					temp.X = j; temp.Y = i;
					if (m_pValidColorMapping[index]){
						mapping.push_back(temp);
						mapping.push_back(m_pColorCoordinatesColor[index]);
					}
					else
					{
						originalP.push_back(temp);
					}
				}
			}
			//temp.X = 1, temp.Y = 1;
			//mapping.push_back(temp); mapping.push_back(temp);
			//temp.X = 3, temp.Y = 3;
			//mapping.push_back(temp); mapping.push_back(temp);
			//temp.X = 2, temp.Y = 2;
			//mapping.push_back(temp); mapping.push_back(temp);
			//temp.X = 3, temp.Y = 3;
			//mapping.push_back(temp); mapping.push_back(temp);
			//temp.X = 0, temp.Y = 1;
			//originalP.push_back(temp); 
			//temp.X = 2, temp.Y = 1;
			//originalP.push_back(temp);
			//temp.X = 2, temp.Y = 3;
			//originalP.push_back(temp);
			MLQMappingRigid(originalP, mapping, deformaion);
			for (int i = 0; i < deformaion.size(); i++){
				int dIndex = deformaion[i].X + deformaion[i].Y * cColorWidth;
				if (dIndex < 0 || dIndex >= cColorHeight * cColorWidth)
					continue;
				int index = originalP[i].X + originalP[i].Y * cColorWidth;
				m_pColorCoordinatesColor[index] = deformaion[i];
				m_pValidColorMapping[index] = true;
			}
		}
		////second turn: every small window to deform the image
		//for (int w = 0; w < cColorWidth / widthStep; w++)
		//for (int h = 0; h < cColorHeight / heightStep; h++)
		//{
		//	originalP.clear(); mapping.clear(); deformaion.clear();
		//	ColorSpacePoint temp;
		//	for (int i = w* widthStep; i <= (w + 1)*widthStep && i < cColorWidth; i++){
		//		for (int j = h*heightStep; j <= (h + 1)*heightStep && j < cColorHeight; j++){
		//			int index = i + j * cColorWidth;
		//			if (index >= cColorWidth* cColorHeight)
		//				continue;
		//			temp.X = i; temp.Y = j;
		//			if (m_pValidColorMapping[index]){
		//				mapping.push_back(temp);
		//				mapping.push_back(m_pColorCoordinatesColor[index]);
		//			}
		//			else
		//			{
		//				originalP.push_back(temp);
		//			}
		//		}
		//	}
		//	MLQMappingRigid(originalP, mapping, deformaion);
		//	for (int i = 0; i < deformaion.size(); i++){
		//		int dIndex = deformaion[i].X + deformaion[i].Y * cColorWidth;
		//		if (dIndex < 0 || dIndex >= cColorHeight * cColorWidth)
		//			continue;
		//		int index = originalP[i].X + originalP[i].Y * cColorWidth;
		//		m_pColorCoordinatesColor[index] = deformaion[i];
		//		m_pValidColorMapping[index] = true;
		//	}
		//}
		//show the map of the valid mapping points in original and deformational image
		memcpy(m_originalRGBX, pColorBuffer, cColorWidth*cColorHeight * sizeof(RGBQUAD));
		for (int i = 0; i < cDepthWidth*cDepthHeight; i++)
		{
			if (m_pBothColorSpaceValid[i]){
				ColorSpacePoint cp = m_pColorCoordinates[i];
				int x = (int) (cp.X + 0.5);
				int y = (int)(cp.Y + 0.5);
				RGBQUAD* c = m_originalRGBX + x + y * cColorWidth;
				c->rgbBlue = 0;
				c->rgbGreen = 255;
				c->rgbRed = 2; 
			}
		}
		m_pDrawCoordinateMapping->Draw(reinterpret_cast<BYTE*>(m_originalRGBX), cColorWidth*cColorHeight * sizeof(RGBQUAD));
		//deformational mapping
		/*memset(m_pOutputRGBX, 0, cColorWidth*cColorHeight*sizeof(RGBQUAD));
		for (int i = 0; i < cDepthWidth*cDepthHeight; i++)
		{
			if (m_pBothColorSpaceValid[i]){
				ColorSpacePoint cp = m_pColorCoordinatesDeformation[i];
				int x = (int)(cp.X + 0.5);
				int y = (int)(cp.Y + 0.5);
				RGBQUAD* c = m_pOutputRGBX + x + y * cColorWidth;
				c->rgbBlue = 0;
				c->rgbGreen = 255;
				c->rgbRed = 2;
			}
		}*/
		memset(m_pOutputRGBX, 0, cColorWidth*cColorHeight*sizeof(RGBQUAD));
		for (int i = 0; i < cColorWidth * cColorHeight; i++)
		{
			int index = m_pColorCoordinatesColor[i].X + m_pColorCoordinatesColor[i].Y * cColorWidth;
			if (index < 0 || index >= cColorHeight * cColorWidth)
				continue;
			//m_pOutputRGBX[index] = pColorBuffer[i];
			m_pOutputRGBX[i] = pColorBuffer[index];
		}
		m_pDrawDeformation->Draw(reinterpret_cast<BYTE*>(m_pOutputRGBX), cColorWidth*cColorHeight * sizeof(RGBQUAD));
		//memset(m_pOutputRGBX, 0, cColorWidth*cColorHeight*sizeof(RGBQUAD));
  //      if (SUCCEEDED(hr))
  //      {
		//	//now do the simplest deformation of the 2d image basing on retangle mapping
		//	int dIndexUL, dIndexUR, dIndexDL, dIndexDR;
		//	int lineStart = 0;
		//	for (int line = 0; line < nDepthHeight - 1; ++line){
		//		for (int index = 0; index < nDepthWidth - 1; ++index){
		//			dIndexUL = lineStart + index;
		//			if (!m_pBothColorSpaceValid[dIndexUL])
		//				continue;
		//			for (int i = 1; i<10; i++){
		//				dIndexUL = dIndexUL + 0; dIndexUR = dIndexUL + i;
		//				dIndexDL = dIndexUL + i*nDepthWidth; dIndexDR = dIndexUL + i*nDepthWidth + i;
		//				if (dIndexDR > nDepthWidth * nDepthHeight || dIndexUR - lineStart > nDepthWidth)
		//					break;
		//				if (m_pBothColorSpaceValid[dIndexUR] && m_pBothColorSpaceValid[dIndexDL] && m_pBothColorSpaceValid[dIndexDR])
		//					colorDeformationMapping(m_pOutputRGBX, pColorBuffer, cColorWidth, cColorHeight, m_pColorCoordinatesDeformation[dIndexUL], m_pColorCoordinatesDeformation[dIndexUR], m_pColorCoordinatesDeformation[dIndexDL], m_pColorCoordinatesDeformation[dIndexDR], \
		//					m_pColorCoordinates[dIndexUL], m_pColorCoordinates[dIndexUR], m_pColorCoordinates[dIndexDL], m_pColorCoordinates[dIndexDR]);
		//			}
		//		}
		//		lineStart += nDepthWidth;
		//	}
		//	//Draw the color data with the depth point
		//	m_pDrawDeformation->Draw(reinterpret_cast<BYTE*>(m_pOutputRGBX), cColorWidth*cColorHeight * sizeof(RGBQUAD));
		//	if (m_bSaveScreenshot)
		//	{
		//		WCHAR szScreenshotPath[MAX_PATH];

		//		// Retrieve the path to My Photos
		//		GetScreenshotFileName(szScreenshotPath, _countof(szScreenshotPath));

		//		// Write out the bitmap to disk
		//		HRESULT hr = SaveBitmapToFile(reinterpret_cast<BYTE*>(m_pOutputRGBX), cColorWidth, cColorHeight, sizeof(RGBQUAD)* 8, szScreenshotPath);

		//		// toggle off so we don't save a screenshot again next frame
		//		m_bSaveScreenshot = false;
		//	}
  //      }
    }
}

void CCoordinateMappingBasics::ProcessFrameByFeaturePoints(INT64 nTime,
	const UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight,
	const RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight,
	const BYTE* pBodyIndexBuffer, int nBodyIndexWidth, int nBodyIndexHeight)
{
	if (m_hWnd)
	{
		if (!m_nStartTime)
		{
			m_nStartTime = nTime;
		}

		double fps = 0.0;

		LARGE_INTEGER qpcNow = { 0 };
		if (m_fFreq)
		{
			if (QueryPerformanceCounter(&qpcNow))
			{
				if (m_nLastCounter)
				{
					m_nFramesSinceUpdate++;
					fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
				}
			}
		}

		WCHAR szStatusMessage[64];
		StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L"T:%0.2f %0.2f %0.2f R:%0.2f %0.2f %0.2f FPS = %0.2f    Time = %I64d", m_cameraTranslation.x(), m_cameraTranslation.y(), m_cameraTranslation.z(), m_rotationX, m_rotationY, m_rotationZ, fps, (nTime - m_nStartTime));

		if (SetStatusMessage(szStatusMessage, 1000, false))
		{
			m_nLastCounter = qpcNow.QuadPart;
			m_nFramesSinceUpdate = 0;
		}
	}

	// Make sure we've received valid data
	if (m_pCoordinateMapper && m_pColorCoordinates && m_pOutputRGBX &&
		pDepthBuffer && (nDepthWidth == cDepthWidth) && (nDepthHeight == cDepthHeight) &&
		pColorBuffer && (nColorWidth == cColorWidth) && (nColorHeight == cColorHeight) &&
		pBodyIndexBuffer && (nBodyIndexWidth == cDepthWidth) && (nBodyIndexHeight == cDepthHeight))
	{
		//update drawing
		m_pDrawCoordinateMapping->Draw(reinterpret_cast<BYTE*>(m_originalRGBX), cColorWidth*cColorHeight * sizeof(RGBQUAD));
		m_pDrawDeformation->Draw(reinterpret_cast<BYTE*>(m_pOutputRGBX), cColorWidth*cColorHeight * sizeof(RGBQUAD));
		//memcpy(m_pOutputRGBX, pColorBuffer, cColorWidth*cColorHeight * sizeof(RGBQUAD));
		//m_pDrawCoordinateMapping->Draw(reinterpret_cast<BYTE*>(m_pOutputRGBX), cColorWidth*cColorHeight * sizeof(RGBQUAD));
		if (!m_computeDeformation)
			return;
		m_computeDeformation = false;
		//read feature points from the shared memory
		int noFeaturePoints = *reinterpret_cast<int*>((byte*)m_pBuf + m_sizeOfPointClouds);
		do 
		{
			::Sleep(100);
		} while (noFeaturePoints ==0);
		float* pFeaturePoints = reinterpret_cast<float*>((byte*)m_pBuf + m_sizeOfPointClouds + sizeof(int));
		memcpy(m_pCameraCoordinates, pFeaturePoints, noFeaturePoints * 3 * sizeof(float));
		//project to 2d image
		//virtual camera center
		HRESULT hr = m_pCoordinateMapper->MapCameraPointsToColorSpace(noFeaturePoints, m_pCameraCoordinates, noFeaturePoints, m_pColorCoordinates);
		std::cout << "read feature points Successful" << std::endl;

		for (int depthIndex = 0; depthIndex < noFeaturePoints; ++depthIndex){
			//m_pCameraCoordinates[depthIndex].Y -= 0.2;
			Eigen::Translation3d point;
			point.x() = m_pCameraCoordinates[depthIndex].X;
			point.y() = m_pCameraCoordinates[depthIndex].Y;
			point.z() = m_pCameraCoordinates[depthIndex].Z;
			Eigen::Affine3d temp = m_cameraTransformation * point;
			m_pCameraCoordinates[depthIndex].X = temp.translation().x();
			m_pCameraCoordinates[depthIndex].Y = temp.translation().y();
			m_pCameraCoordinates[depthIndex].Z = temp.translation().z();
		}
		//virtual camera center
		hr = m_pCoordinateMapper->MapCameraPointsToColorSpace(noFeaturePoints, m_pCameraCoordinates, noFeaturePoints, m_pColorCoordinatesDeformation);


		//to check if the color space mapping is valid
		int noValid = 0;
		memset(m_pBothColorSpaceValid, 0, cDepthWidth * cDepthHeight * sizeof(bool)); //reset
		memset(m_pValidColorMapping, 0, cColorWidth*cColorHeight*sizeof(bool));
		memset(m_pColorCoordinatesColor, 0, cColorHeight*cColorWidth*sizeof(ColorSpacePoint));

		//format the color coordinates
		for (int i = 0; i < noFeaturePoints; i++){
			m_pColorCoordinatesDeformation[i].X = (int)(m_pColorCoordinatesDeformation[i].X + 0.5);
			m_pColorCoordinatesDeformation[i].Y = (int)(m_pColorCoordinatesDeformation[i].Y + 0.5);
			m_pColorCoordinates[i].X = (int)(m_pColorCoordinates[i].X + 0.5);
			m_pColorCoordinates[i].Y = (int)(m_pColorCoordinates[i].Y + 0.5);
		}
		for (int i = 0; i < noFeaturePoints; i++){
			if (invalidColorSpace(m_pColorCoordinates[i]) || invalidColorSpace(m_pColorCoordinatesDeformation[i]))
				m_pBothColorSpaceValid[i] = false;
			else{
				m_pBothColorSpaceValid[i] = true;
				noValid++;
				//update the mapping information
				//we could switch this to check the result, forward warping or back warping
				int x = (int)(m_pColorCoordinatesDeformation[i].X + 0.5);
				int y = (int)(m_pColorCoordinatesDeformation[i].Y + 0.5);
				int dx = (int)(m_pColorCoordinates[i].X + 0.5);
				int dy = (int)(m_pColorCoordinates[i].Y + 0.5);
				m_pColorCoordinatesColor[x + y * cColorWidth].X = dx;
				m_pColorCoordinatesColor[x + y * cColorWidth].Y = dy;
				m_pValidColorMapping[x + y *cColorWidth] = true;
			}
		}
		//do the deformation 
		vector<ColorSpacePoint> originalP, mapping, deformaion;
		originalP.clear(); mapping.clear(); deformaion.clear();
		int widthStep = 1, heightStep = 1;
		ColorSpacePoint temp;
		//first turn 
		for (int istart = 0; istart < widthStep; istart++)
		for (int jstart = 0; jstart < heightStep; jstart++)
		{
			originalP.clear(); mapping.clear(); deformaion.clear();
			for (int i = 0; i < cColorHeight; i += heightStep)
			{
				for (int j = 0; j < cColorWidth; j += widthStep){
					int index = j + i * cColorWidth;
					temp.X = j; temp.Y = i;
					if (m_pValidColorMapping[index]){
						mapping.push_back(temp);
						mapping.push_back(m_pColorCoordinatesColor[index]);
					}
					else
					{
						originalP.push_back(temp);
					}
				}
			}
			//temp.X = 1, temp.Y = 1;
			//mapping.push_back(temp); mapping.push_back(temp);
			//temp.X = 3, temp.Y = 3;
			//mapping.push_back(temp); mapping.push_back(temp);
			//temp.X = 2, temp.Y = 2;
			//mapping.push_back(temp); mapping.push_back(temp);
			//temp.X = 3, temp.Y = 3;
			//mapping.push_back(temp); mapping.push_back(temp);
			//temp.X = 0, temp.Y = 1;
			//originalP.push_back(temp); 
			//temp.X = 2, temp.Y = 1;
			//originalP.push_back(temp);
			//temp.X = 2, temp.Y = 3;
			//originalP.push_back(temp);
			MLQMappingRigid(originalP, mapping, deformaion);
			for (int i = 0; i < deformaion.size(); i++){
				int dIndex = deformaion[i].X + deformaion[i].Y * cColorWidth;
				if (dIndex < 0 || dIndex >= cColorHeight * cColorWidth)
					continue;
				int index = originalP[i].X + originalP[i].Y * cColorWidth;
				m_pColorCoordinatesColor[index] = deformaion[i];
				m_pValidColorMapping[index] = true;
			}
		}
		////second turn: every small window to deform the image
		//for (int w = 0; w < cColorWidth / widthStep; w++)
		//for (int h = 0; h < cColorHeight / heightStep; h++)
		//{
		//	originalP.clear(); mapping.clear(); deformaion.clear();
		//	ColorSpacePoint temp;
		//	for (int i = w* widthStep; i <= (w + 1)*widthStep && i < cColorWidth; i++){
		//		for (int j = h*heightStep; j <= (h + 1)*heightStep && j < cColorHeight; j++){
		//			int index = i + j * cColorWidth;
		//			if (index >= cColorWidth* cColorHeight)
		//				continue;
		//			temp.X = i; temp.Y = j;
		//			if (m_pValidColorMapping[index]){
		//				mapping.push_back(temp);
		//				mapping.push_back(m_pColorCoordinatesColor[index]);
		//			}
		//			else
		//			{
		//				originalP.push_back(temp);
		//			}
		//		}
		//	}
		//	MLQMappingRigid(originalP, mapping, deformaion);
		//	for (int i = 0; i < deformaion.size(); i++){
		//		int dIndex = deformaion[i].X + deformaion[i].Y * cColorWidth;
		//		if (dIndex < 0 || dIndex >= cColorHeight * cColorWidth)
		//			continue;
		//		int index = originalP[i].X + originalP[i].Y * cColorWidth;
		//		m_pColorCoordinatesColor[index] = deformaion[i];
		//		m_pValidColorMapping[index] = true;
		//	}
		//}
		//show the map of the valid mapping points in original and deformational image
		memcpy(m_originalRGBX, pColorBuffer, cColorWidth*cColorHeight * sizeof(RGBQUAD));
		for (int i = 0; i < cDepthWidth*cDepthHeight; i++)
		{
			if (m_pBothColorSpaceValid[i]){
				ColorSpacePoint cp = m_pColorCoordinates[i];
				int x = (int)(cp.X + 0.5);
				int y = (int)(cp.Y + 0.5);
				RGBQUAD* c = m_originalRGBX + x + y * cColorWidth;
				c->rgbBlue = 0;
				c->rgbGreen = 0;
				c->rgbRed = 255;
			}
		}
		m_pDrawCoordinateMapping->Draw(reinterpret_cast<BYTE*>(m_originalRGBX), cColorWidth*cColorHeight * sizeof(RGBQUAD));
		//deformational mapping
		/*memset(m_pOutputRGBX, 0, cColorWidth*cColorHeight*sizeof(RGBQUAD));
		for (int i = 0; i < cDepthWidth*cDepthHeight; i++)
		{
		if (m_pBothColorSpaceValid[i]){
		ColorSpacePoint cp = m_pColorCoordinatesDeformation[i];
		int x = (int)(cp.X + 0.5);
		int y = (int)(cp.Y + 0.5);
		RGBQUAD* c = m_pOutputRGBX + x + y * cColorWidth;
		c->rgbBlue = 0;
		c->rgbGreen = 255;
		c->rgbRed = 2;
		}
		}*/
		memset(m_pOutputRGBX, 0, cColorWidth*cColorHeight*sizeof(RGBQUAD));
		for (int i = 0; i < cColorWidth * cColorHeight; i++)
		{
			int index = m_pColorCoordinatesColor[i].X + m_pColorCoordinatesColor[i].Y * cColorWidth;
			if (index < 0 || index >= cColorHeight * cColorWidth)
				continue;
			//m_pOutputRGBX[index] = pColorBuffer[i];
			m_pOutputRGBX[i] = pColorBuffer[index];
		}
		m_pDrawDeformation->Draw(reinterpret_cast<BYTE*>(m_pOutputRGBX), cColorWidth*cColorHeight * sizeof(RGBQUAD));
		//memset(m_pOutputRGBX, 0, cColorWidth*cColorHeight*sizeof(RGBQUAD));
		//      if (SUCCEEDED(hr))
		//      {
		//	//now do the simplest deformation of the 2d image basing on retangle mapping
		//	int dIndexUL, dIndexUR, dIndexDL, dIndexDR;
		//	int lineStart = 0;
		//	for (int line = 0; line < nDepthHeight - 1; ++line){
		//		for (int index = 0; index < nDepthWidth - 1; ++index){
		//			dIndexUL = lineStart + index;
		//			if (!m_pBothColorSpaceValid[dIndexUL])
		//				continue;
		//			for (int i = 1; i<10; i++){
		//				dIndexUL = dIndexUL + 0; dIndexUR = dIndexUL + i;
		//				dIndexDL = dIndexUL + i*nDepthWidth; dIndexDR = dIndexUL + i*nDepthWidth + i;
		//				if (dIndexDR > nDepthWidth * nDepthHeight || dIndexUR - lineStart > nDepthWidth)
		//					break;
		//				if (m_pBothColorSpaceValid[dIndexUR] && m_pBothColorSpaceValid[dIndexDL] && m_pBothColorSpaceValid[dIndexDR])
		//					colorDeformationMapping(m_pOutputRGBX, pColorBuffer, cColorWidth, cColorHeight, m_pColorCoordinatesDeformation[dIndexUL], m_pColorCoordinatesDeformation[dIndexUR], m_pColorCoordinatesDeformation[dIndexDL], m_pColorCoordinatesDeformation[dIndexDR], \
				//					m_pColorCoordinates[dIndexUL], m_pColorCoordinates[dIndexUR], m_pColorCoordinates[dIndexDL], m_pColorCoordinates[dIndexDR]);
		//			}
		//		}
		//		lineStart += nDepthWidth;
		//	}
		//	//Draw the color data with the depth point
		//	m_pDrawDeformation->Draw(reinterpret_cast<BYTE*>(m_pOutputRGBX), cColorWidth*cColorHeight * sizeof(RGBQUAD));
		//	if (m_bSaveScreenshot)
		//	{
		//		WCHAR szScreenshotPath[MAX_PATH];

		//		// Retrieve the path to My Photos
		//		GetScreenshotFileName(szScreenshotPath, _countof(szScreenshotPath));

		//		// Write out the bitmap to disk
		//		HRESULT hr = SaveBitmapToFile(reinterpret_cast<BYTE*>(m_pOutputRGBX), cColorWidth, cColorHeight, sizeof(RGBQUAD)* 8, szScreenshotPath);

		//		// toggle off so we don't save a screenshot again next frame
		//		m_bSaveScreenshot = false;
		//	}
		//      }
	}
}

void CCoordinateMappingBasics::ProcessFrameInterpolation(INT64 nTime,
	const UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight,
	const RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight,
	const BYTE* pBodyIndexBuffer, int nBodyIndexWidth, int nBodyIndexHeight)
{
	if (m_hWnd)
	{
		if (!m_nStartTime)
		{
			m_nStartTime = nTime;
		}

		double fps = 0.0;

		LARGE_INTEGER qpcNow = { 0 };
		if (m_fFreq)
		{
			if (QueryPerformanceCounter(&qpcNow))
			{
				if (m_nLastCounter)
				{
					m_nFramesSinceUpdate++;
					fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
				}
			}
		}

		WCHAR szStatusMessage[64];
		StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L"T:%0.2f %0.2f %0.2f R:%0.2f %0.2f %0.2f FPS = %0.2f    Time = %I64d", m_cameraTranslation.x(), m_cameraTranslation.y(), m_cameraTranslation.z(), m_rotationX, m_rotationY, m_rotationZ, fps, (nTime - m_nStartTime));

		if (SetStatusMessage(szStatusMessage, 1000, false))
		{
			m_nLastCounter = qpcNow.QuadPart;
			m_nFramesSinceUpdate = 0;
		}
	}
	if (!m_enableSimpleInterpolation)
		return;
	int pointsOfDepth = nDepthHeight * nDepthWidth;
	int pointsOfColor = nColorWidth * nColorHeight;
	// Make sure we've received valid data
	if (m_pCoordinateMapper && m_pColorCoordinates && m_pOutputRGBX &&
		pDepthBuffer && (nDepthWidth == cDepthWidth) && (nDepthHeight == cDepthHeight) &&
		pColorBuffer && (nColorWidth == cColorWidth) && (nColorHeight == cColorHeight) &&
		pBodyIndexBuffer && (nBodyIndexWidth == cDepthWidth) && (nBodyIndexHeight == cDepthHeight))
	{
		//first step: processing the depth information to fill the hole
		memcpy(m_pDepthBuffer, pDepthBuffer, cDepthWidth*cDepthHeight*sizeof(UINT16));
		smoothDepthImage(m_pDepthBuffer, nDepthWidth, nDepthHeight,5);
		//update drawing
		/*m_pDrawCoordinateMapping->Draw(reinterpret_cast<BYTE*>(m_originalRGBX), cColorWidth*cColorHeight * sizeof(RGBQUAD));
		m_pDrawDeformation->Draw(reinterpret_cast<BYTE*>(m_pOutputRGBX), cColorWidth*cColorHeight * sizeof(RGBQUAD));*/

		//original image and depth information
		HRESULT hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(nDepthWidth * nDepthHeight, (UINT16*)m_pDepthBuffer, nDepthWidth * nDepthHeight, m_pColorCoordinates);
		//mapping from depth to camera space and generate the point clouds file to find the key points
		hr = m_pCoordinateMapper->MapDepthFrameToCameraSpace(nDepthWidth*nDepthHeight, (UINT16*)pDepthBuffer, nDepthWidth * nDepthHeight, m_pCameraCoordinates);
		if (SUCCEEDED(hr)){
			std::cout << "mapping from depth to camera space Successful" << std::endl;
			//MessageBox(NULL, L"mapping from depth to camera space", L"Successful", MB_OK);
			//apply a transformation to the piont clouds, this transfromatin should be the inversion of the transformation from current camera center to the virtual camera center
			// the simplest situation, the virtual camera centor is 20cm belower
			for (int depthIndex = 0; depthIndex < (nDepthWidth * nDepthHeight); ++depthIndex){
				//m_pCameraCoordinates[depthIndex].Y -= 0.2;
				Eigen::Translation3d point;
				point.x() = m_pCameraCoordinates[depthIndex].X;
				point.y() = m_pCameraCoordinates[depthIndex].Y;
				point.z() = m_pCameraCoordinates[depthIndex].Z;
				Eigen::Affine3d temp = m_cameraTransformation * point;
				m_pCameraCoordinates[depthIndex].X = temp.translation().x();
				m_pCameraCoordinates[depthIndex].Y = temp.translation().y();
				m_pCameraCoordinates[depthIndex].Z = temp.translation().z();
			}
		}
		filterCameraSpacePoints(m_pCameraCoordinates, nDepthWidth, nDepthHeight);
		//virtual camera center
		hr = m_pCoordinateMapper->MapCameraPointsToColorSpace(nDepthWidth * nDepthHeight, m_pCameraCoordinates, nDepthWidth * nDepthHeight, m_pColorCoordinatesDeformation);

		//to check if the color space mapping is valid
		int noValid = 0;
		memset(m_pBothColorSpaceValid, 0, cDepthWidth * cDepthHeight * sizeof(bool)); //reset
		memset(m_pValidColorMapping, 0, cColorWidth*cColorHeight*sizeof(bool));
		memset(m_pColorCoordinatesColor, 0, cColorWidth*cColorHeight*sizeof(ColorSpacePoint));

		for (int i = 0; i < cDepthWidth* cDepthHeight; i++){
			if (invalidColorSpace(m_pColorCoordinates[i]) || invalidColorSpace(m_pColorCoordinatesDeformation[i]))
				m_pBothColorSpaceValid[i] = false;
			else{
				m_pBothColorSpaceValid[i] = true;
				noValid++;
				//update the mapping information
				//we could switch this to check the result, forward warping or back warping
				int x = (int)(m_pColorCoordinatesDeformation[i].X + 0.5);
				int y = (int)(m_pColorCoordinatesDeformation[i].Y + 0.5);
				int dx = (int)(m_pColorCoordinates[i].X + 0.5);
				int dy = (int)(m_pColorCoordinates[i].Y + 0.5);
				/*if (abs(x-dx)>1 || abs(y - dy) > 1)
				{
					MessageBox(NULL, L"mapping not equal 1", NULL, MB_OK);
				}*/
				m_pColorCoordinatesColor[x + y * cColorWidth].X = dx;
				m_pColorCoordinatesColor[x + y * cColorWidth].Y = dy;
 				m_pValidColorMapping[x + y *cColorWidth] = true;
			}
		}
		//deformational mapping
		memcpy(m_originalRGBX, pColorBuffer, cColorWidth*cColorHeight * sizeof(RGBQUAD));
		for (int i = 0; i < cDepthWidth*cDepthHeight; i++)
		{
			if (m_pBothColorSpaceValid[i]){
				ColorSpacePoint cp = m_pColorCoordinates[i];
				int x = (int)(cp.X + 0.5);
				int y = (int)(cp.Y + 0.5);
				RGBQUAD* c = m_originalRGBX + x + y * cColorWidth;
				c->rgbBlue = 0;
				c->rgbGreen = 255;
				c->rgbRed = 255;
			}
		}

		memset(m_pOutputRGBXInterpolation, 0, cColorWidth*cColorHeight*sizeof(RGBQUAD));

		////DO interpolation according to the original depth maping
		////now do the simplest deformation of the 2d image basing on rectangle mapping
		//int dIndexUL, dIndexUR, dIndexDL, dIndexDR;
		//int lineStart = 0;
		//for (int line = 0; line < nDepthHeight - 1; ++line){
		//	for (int index = 0; index < nDepthWidth - 1; ++index){
		//		dIndexUL = lineStart + index;
		//		if (!m_pBothColorSpaceValid[dIndexUL])
		//			continue;
		//		bool finished = false;
		//		for (int j = 1; j < 20 && finished==false; j++)
		//		for (int i = 1; i<20 && finished == false; i++){
		//			dIndexUL = dIndexUL + 0; dIndexUR = dIndexUL + j;
		//			dIndexDL = dIndexUL + i*nDepthWidth; dIndexDR = dIndexUL + i*nDepthWidth + j;
		//			if (dIndexDR > nDepthWidth * nDepthHeight || dIndexUR - lineStart > nDepthWidth)
		//				continue;;
		//			if (m_pBothColorSpaceValid[dIndexUR] && m_pBothColorSpaceValid[dIndexDL] && m_pBothColorSpaceValid[dIndexDR]){
		//				colorDeformationBackMapping(m_pOutputRGBXInterpolation, pColorBuffer, cColorWidth, cColorHeight, m_pColorCoordinatesDeformation[dIndexUL], m_pColorCoordinatesDeformation[dIndexUR], m_pColorCoordinatesDeformation[dIndexDL], m_pColorCoordinatesDeformation[dIndexDR], \
		//					m_pColorCoordinates[dIndexUL], m_pColorCoordinates[dIndexUR], m_pColorCoordinates[dIndexDL], m_pColorCoordinates[dIndexDR]);
		//				finished = true;
		//			}
		//		}
		//	}
		//	lineStart += nDepthWidth;
		//}

		//do simple interpolation according to the target color image
		int dIndexUL, dIndexUR, dIndexDL, dIndexDR;
		int lineStart = 0;
		int MAXInterpolationStep = 10;
		for (int line = 0; line < nColorHeight - 1; ++line){
			for (int index = 0; index < nColorWidth - 1; ++index){
				dIndexUL = lineStart + index;
				if (!m_pValidColorMapping[dIndexUL])
					continue;
				bool finished = false;
				for (int j = 1; j < MAXInterpolationStep && finished == false; j++)
				for (int i = 1; i < MAXInterpolationStep && finished == false; i++){
					dIndexUL = dIndexUL + 0; dIndexUR = dIndexUL + j;
					dIndexDL = dIndexUL + i*nColorWidth; dIndexDR = dIndexUL + i*nColorWidth + j;
					if (dIndexDR > nColorWidth* nColorHeight || dIndexUR - lineStart > nColorWidth-1)
						finished = true;
					ColorSpacePoint cUL = {index,line};
					ColorSpacePoint cUR = { index+j, line };
					ColorSpacePoint cDL = { index, line+i };
					ColorSpacePoint cDR = { index+j, line+i };
					if (m_pValidColorMapping[dIndexUR] && m_pValidColorMapping[dIndexDL] && m_pValidColorMapping[dIndexDR]){
						/*if (abs(cDR.X - m_pColorCoordinatesColor[dIndexDR].X) > 2)
						{
							MessageBox(NULL, L"mapping not equal 2", NULL, MB_OK);
						}*/
						colorDeformationBackMapping(m_pOutputRGBXInterpolation, pColorBuffer, cColorWidth, cColorHeight, cUL, cUR, cDL,cDR, \
							m_pColorCoordinatesColor[dIndexUL], m_pColorCoordinatesColor[dIndexUR], m_pColorCoordinatesColor[dIndexDL], m_pColorCoordinatesColor[dIndexDR]);
						finished = true;
					}
				}
			}
			lineStart += nColorWidth;
		}
		//do MLS interpolation
		//do the deformation 
		if (!m_enableMLSDeformation)
			return;
		m_enableMLSDeformation = false;
		m_enableSimpleInterpolation = false;
		vector<ColorSpacePoint> originalP, mapping, deformaion;
		originalP.clear(); mapping.clear(); deformaion.clear();
		int widthStep = 10, heightStep = 10;
		ColorSpacePoint temp;
		//every small window to deform the image
		for (int w = 0; w < cColorWidth ; w++)
		for (int h = 0; h < cColorHeight; h+= heightStep)
		{
			//int w = 0;
			int orginalIndex = w + h * cColorWidth;
			if (m_pValidColorMapping[orginalIndex])
				continue;
			originalP.clear(); mapping.clear(); deformaion.clear();
			ColorSpacePoint temp;
			for (int i = w; i <= w + widthStep && i < cColorWidth; i++){
				for (int j = h; j <= h + heightStep && j < cColorHeight; j++){
					int index = i + j * cColorWidth;
					if (index >= cColorWidth* cColorHeight)
						continue;
					temp.X = i; temp.Y = j;
					if (m_pValidColorMapping[index]){
						mapping.push_back(temp);
						mapping.push_back(m_pColorCoordinatesColor[index]);
						break;
					}
					else
					{
						originalP.push_back(temp);
					}
				}
				for (int j = h; j >= h - heightStep && j >=0 ; j--){
					int index = i + j * cColorWidth;
					if (index >= cColorWidth* cColorHeight)
						continue;
					temp.X = i; temp.Y = j;
					if (m_pValidColorMapping[index]){
						mapping.push_back(temp);
						mapping.push_back(m_pColorCoordinatesColor[index]);
						break;
					}
					else
					{
						originalP.push_back(temp);
					}
				}
			}
			if (mapping.size() < 4){
				h += heightStep - 1;
				continue;
			}
			MLQMappingRigid(originalP, mapping, deformaion);
			for (int i = 0; i < deformaion.size(); i++){
				int dIndex = deformaion[i].X + deformaion[i].Y * cColorWidth;
				if (dIndex < 0 || dIndex >= cColorHeight * cColorWidth)
					continue;
				int index = originalP[i].X + originalP[i].Y * cColorWidth;
				m_pColorCoordinatesColor[index] = deformaion[i];
				m_pValidColorMapping[index] = true;
				m_pOutputRGBXInterpolation[index] = pColorBuffer[dIndex];
			}
		}
	}
}

/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
/// <param name="showTimeMsec">time in milliseconds to ignore future status messages</param>
/// <param name="bForce">force status update</param>
bool CCoordinateMappingBasics::SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce)
{
    DWORD now = GetTickCount();

    if (m_hWnd && (bForce || (m_nNextStatusTime <= now)))
    {
        SetDlgItemText(m_hWnd, IDC_STATUS, szMessage);
        m_nNextStatusTime = now + nShowTimeMsec;

        return true;
    }

    return false;
}

/// <summary>
/// Get the name of the file where screenshot will be stored.
/// </summary>
/// <param name="lpszFilePath">string buffer that will receive screenshot file name.</param>
/// <param name="nFilePathSize">number of characters in lpszFilePath string buffer.</param>
/// <returns>
/// S_OK on success, otherwise failure code.
/// </returns>
HRESULT CCoordinateMappingBasics::GetScreenshotFileName(_Out_writes_z_(nFilePathSize) LPWSTR lpszFilePath, UINT nFilePathSize)
{
    WCHAR* pszKnownPath = NULL;
    HRESULT hr = SHGetKnownFolderPath(FOLDERID_Pictures, 0, NULL, &pszKnownPath);

    if (SUCCEEDED(hr))
    {
        // Get the time
        WCHAR szTimeString[MAX_PATH];
        GetTimeFormatEx(NULL, 0, NULL, L"hh'-'mm'-'ss", szTimeString, _countof(szTimeString));

        // File name will be KinectScreenshotDepth-HH-MM-SS.bmp
        StringCchPrintfW(lpszFilePath, nFilePathSize, L"%s\\KinectScreenshot-CoordinateMapping-%s.bmp", pszKnownPath, szTimeString);
    }

    if (pszKnownPath)
    {
        CoTaskMemFree(pszKnownPath);
    }

    return hr;
}

/// <summary>
/// Save passed in image data to disk as a bitmap
/// </summary>
/// <param name="pBitmapBits">image data to save</param>
/// <param name="lWidth">width (in pixels) of input image data</param>
/// <param name="lHeight">height (in pixels) of input image data</param>
/// <param name="wBitsPerPixel">bits per pixel of image data</param>
/// <param name="lpszFilePath">full file path to output bitmap to</param>
/// <returns>indicates success or failure</returns>
HRESULT CCoordinateMappingBasics::SaveBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCWSTR lpszFilePath)
{
    DWORD dwByteCount = lWidth * lHeight * (wBitsPerPixel / 8);

    BITMAPINFOHEADER bmpInfoHeader = {0};

    bmpInfoHeader.biSize        = sizeof(BITMAPINFOHEADER);  // Size of the header
    bmpInfoHeader.biBitCount    = wBitsPerPixel;             // Bit count
    bmpInfoHeader.biCompression = BI_RGB;                    // Standard RGB, no compression
    bmpInfoHeader.biWidth       = lWidth;                    // Width in pixels
    bmpInfoHeader.biHeight      = -lHeight;                  // Height in pixels, negative indicates it's stored right-side-up
    bmpInfoHeader.biPlanes      = 1;                         // Default
    bmpInfoHeader.biSizeImage   = dwByteCount;               // Image size in bytes

    BITMAPFILEHEADER bfh = {0};

    bfh.bfType    = 0x4D42;                                           // 'M''B', indicates bitmap
    bfh.bfOffBits = bmpInfoHeader.biSize + sizeof(BITMAPFILEHEADER);  // Offset to the start of pixel data
    bfh.bfSize    = bfh.bfOffBits + bmpInfoHeader.biSizeImage;        // Size of image + headers

    // Create the file on disk to write to
    HANDLE hFile = CreateFileW(lpszFilePath, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);

    // Return if error opening file
    if (NULL == hFile) 
    {
        return E_ACCESSDENIED;
    }

    DWORD dwBytesWritten = 0;
    
    // Write the bitmap file header
    if (!WriteFile(hFile, &bfh, sizeof(bfh), &dwBytesWritten, NULL))
    {
        CloseHandle(hFile);
        return E_FAIL;
    }
    
    // Write the bitmap info header
    if (!WriteFile(hFile, &bmpInfoHeader, sizeof(bmpInfoHeader), &dwBytesWritten, NULL))
    {
        CloseHandle(hFile);
        return E_FAIL;
    }
    
    // Write the RGB Data
    if (!WriteFile(hFile, pBitmapBits, bmpInfoHeader.biSizeImage, &dwBytesWritten, NULL))
    {
        CloseHandle(hFile);
        return E_FAIL;
    }    

    // Close the file
    CloseHandle(hFile);
    return S_OK;
}

/// <summary>
/// Load an image from a resource into a buffer
/// </summary>
/// <param name="resourceName">name of image resource to load</param>
/// <param name="resourceType">type of resource to load</param>
/// <param name="nOutputWidth">width (in pixels) of scaled output bitmap</param>
/// <param name="nOutputHeight">height (in pixels) of scaled output bitmap</param>
/// <param name="pOutputBuffer">buffer that will hold the loaded image</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT CCoordinateMappingBasics::LoadResourceImage(PCWSTR resourceName, PCWSTR resourceType, UINT nOutputWidth, UINT nOutputHeight, RGBQUAD* pOutputBuffer)
{
    IWICImagingFactory* pIWICFactory = NULL;
    IWICBitmapDecoder* pDecoder = NULL;
    IWICBitmapFrameDecode* pSource = NULL;
    IWICStream* pStream = NULL;
    IWICFormatConverter* pConverter = NULL;
    IWICBitmapScaler* pScaler = NULL;

    HRSRC imageResHandle = NULL;
    HGLOBAL imageResDataHandle = NULL;
    void *pImageFile = NULL;
    DWORD imageFileSize = 0;

    HRESULT hrCoInit = CoInitialize(NULL);
    HRESULT hr = hrCoInit;

    if (SUCCEEDED(hr))
    {
        hr = CoCreateInstance(CLSID_WICImagingFactory, NULL, CLSCTX_INPROC_SERVER, IID_IWICImagingFactory, (LPVOID*)&pIWICFactory);
    }

    if (SUCCEEDED(hr))
    {
        // Locate the resource
        imageResHandle = FindResourceW(HINST_THISCOMPONENT, resourceName, resourceType);
        hr = imageResHandle ? S_OK : E_FAIL;
    }

    if (SUCCEEDED(hr))
    {
        // Load the resource
        imageResDataHandle = LoadResource(HINST_THISCOMPONENT, imageResHandle);
        hr = imageResDataHandle ? S_OK : E_FAIL;
    }

    if (SUCCEEDED(hr))
    {
        // Lock it to get a system memory pointer.
        pImageFile = LockResource(imageResDataHandle);
        hr = pImageFile ? S_OK : E_FAIL;
    }

    if (SUCCEEDED(hr))
    {
        // Calculate the size.
        imageFileSize = SizeofResource(HINST_THISCOMPONENT, imageResHandle);
        hr = imageFileSize ? S_OK : E_FAIL;
    }

    if (SUCCEEDED(hr))
    {
        // Create a WIC stream to map onto the memory.
        hr = pIWICFactory->CreateStream(&pStream);
    }

    if (SUCCEEDED(hr))
    {
        // Initialize the stream with the memory pointer and size.
        hr = pStream->InitializeFromMemory(
            reinterpret_cast<BYTE*>(pImageFile),
            imageFileSize);
    }

    if (SUCCEEDED(hr))
    {
        // Create a decoder for the stream.
        hr = pIWICFactory->CreateDecoderFromStream(
            pStream,
            NULL,
            WICDecodeMetadataCacheOnLoad,
            &pDecoder);
    }

    if (SUCCEEDED(hr))
    {
        // Create the initial frame.
        hr = pDecoder->GetFrame(0, &pSource);
    }

    if (SUCCEEDED(hr))
    {
        // Convert the image format to 32bppPBGRA
        // (DXGI_FORMAT_B8G8R8A8_UNORM + D2D1_ALPHA_MODE_PREMULTIPLIED).
        hr = pIWICFactory->CreateFormatConverter(&pConverter);
    }

    if (SUCCEEDED(hr))
    {
        hr = pIWICFactory->CreateBitmapScaler(&pScaler);
    }

    if (SUCCEEDED(hr))
    {
        hr = pScaler->Initialize(
            pSource,
            nOutputWidth,
            nOutputHeight,
            WICBitmapInterpolationModeCubic
            );
    }

    if (SUCCEEDED(hr))
    {
        hr = pConverter->Initialize(
            pScaler,
            GUID_WICPixelFormat32bppPBGRA,
            WICBitmapDitherTypeNone,
            NULL,
            0.f,
            WICBitmapPaletteTypeMedianCut);
    }

    UINT width = 0;
    UINT height = 0;
    if (SUCCEEDED(hr))
    {
        hr = pConverter->GetSize(&width, &height);
    }

    // make sure the image scaled correctly so the output buffer is big enough
    if (SUCCEEDED(hr))
    {
        if ((width != nOutputWidth) || (height != nOutputHeight))
        {
            hr = E_FAIL;
        }
    }

    if (SUCCEEDED(hr))
    {
        hr = pConverter->CopyPixels(NULL, width * sizeof(RGBQUAD), nOutputWidth * nOutputHeight * sizeof(RGBQUAD), reinterpret_cast<BYTE*>(pOutputBuffer));
    }

    SafeRelease(pScaler);
    SafeRelease(pConverter);
    SafeRelease(pSource);
    SafeRelease(pDecoder);
    SafeRelease(pStream);
    SafeRelease(pIWICFactory);

    if (SUCCEEDED(hrCoInit))
    {
        CoUninitialize();
    }

    return hr;
}

