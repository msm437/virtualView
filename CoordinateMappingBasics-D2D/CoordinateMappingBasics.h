//------------------------------------------------------------------------------
// <copyright file="CoordinateMappingBasics.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once
#include <Eigen/core>
#include <Eigen/Geometry>
#include "resource.h"
#include "ImageRenderer.h"
#include <vector>

using namespace std;
using  std::vector;

class CCoordinateMappingBasics
{
    static const int        cDepthWidth  = 512;
    static const int        cDepthHeight = 424;
    static const int        cColorWidth  = 1920;
    static const int        cColorHeight = 1080;
	typedef struct
	{
		int x;
		int y;
	} Position2D;
public:
    /// <summary>
    /// Constructor
    /// </summary>
    CCoordinateMappingBasics();

    /// <summary>
    /// Destructor
    /// </summary>
    ~CCoordinateMappingBasics();

    /// <summary>
    /// Handles window messages, passes most to the class instance to handle
    /// </summary>
    /// <param name="hWnd">window message is for</param>
    /// <param name="uMsg">message</param>
    /// <param name="wParam">message data</param>
    /// <param name="lParam">additional message data</param>
    /// <returns>result of message processing</returns>
    static LRESULT CALLBACK MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

    /// <summary>
    /// Handle windows messages for a class instance
    /// </summary>
    /// <param name="hWnd">window message is for</param>
    /// <param name="uMsg">message</param>
    /// <param name="wParam">message data</param>
    /// <param name="lParam">additional message data</param>
    /// <returns>result of message processing</returns>
    LRESULT CALLBACK        DlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

    /// <summary>
    /// Creates the main window and begins processing
    /// </summary>
    /// <param name="hInstance"></param>
    /// <param name="nCmdShow"></param>
    int                     Run(HINSTANCE hInstance, int nCmdShow);
	inline void					colorDeformationMapping(RGBQUAD* pDstColorBuffer, const RGBQUAD* pSrcColorBuffer, int width, int height, ColorSpacePoint dUL, ColorSpacePoint dUR, ColorSpacePoint dDL, ColorSpacePoint dDR, \
		ColorSpacePoint sUL, ColorSpacePoint sUR, ColorSpacePoint sDL, ColorSpacePoint sDR);
	inline void					colorDeformationBackMapping(RGBQUAD* pDstColorBuffer, const RGBQUAD* pSrcColorBuffer, int width, int height, ColorSpacePoint dUL, ColorSpacePoint dUR, ColorSpacePoint dDL, ColorSpacePoint dDR, \
		ColorSpacePoint sUL, ColorSpacePoint sUR, ColorSpacePoint sDL, ColorSpacePoint sDR);
	//using the mapping relationship between two image, compute the position for every original point
	void	MLQMapping(vector<ColorSpacePoint>& originalP, vector<ColorSpacePoint>& mapping, vector<ColorSpacePoint>& deformaion, int type = 0);
	void	MLQMappingSimilarity(vector<ColorSpacePoint>& originalP, vector<ColorSpacePoint>& mapping, vector<ColorSpacePoint>& deformaion);
	void	MLQMappingRigid(vector<ColorSpacePoint>& originalP, vector<ColorSpacePoint>& mapping, vector<ColorSpacePoint>& deformaion);
protected:
	inline float  weightForVP(ColorSpacePoint v, ColorSpacePoint p);
	inline ColorSpacePoint addCSPoint(ColorSpacePoint a, ColorSpacePoint b){ ColorSpacePoint sum; sum.X = a.X + b.X; sum.Y = a.Y + b.Y; return sum; }
	inline ColorSpacePoint multiCSPoint(ColorSpacePoint a, float scale){ ColorSpacePoint res; res.X = a.X * scale; res.Y = a.Y * scale; return res; }
	inline ColorSpacePoint subCSPoint(ColorSpacePoint a, ColorSpacePoint b){ ColorSpacePoint res; res.X = a.X - b.X; res.Y = a.Y - b.Y; return res; }
	inline float scatterofCSPoint(ColorSpacePoint a){ return a.X * a.X + a.Y * a.Y; }
	inline void matrix2x2Multi(float* res, float* m, float* n){ res[0] = m[0] * n[0] + m[1] * n[2]; res[1] = m[0] * n[1] + m[1] * n[3]; res[2] = m[2] * n[0] + m[3] * n[2]; res[3] = m[2] * n[1] + m[3] * n[3]; }
	inline void matrix2x2Add(float* res, float* m) { res[0] += m[0]; res[1] += m[1]; res[2] += m[2]; res[3] += m[3]; }
	inline void matrix2x2Scale(float* res, float s) { res[0] *= s; res[1] *= s; res[2] *= s; res[3] *= s; }
	inline ColorSpacePoint point2dXmatrix(ColorSpacePoint p, float* m){ ColorSpacePoint res; res.X = p.X * m[0] + p.Y * m[2]; res.Y = p.X * m[1] + p.Y *m[3]; return res; }

private:
    HWND                    m_hWnd;
    INT64                   m_nStartTime;
    INT64                   m_nLastCounter;
    double                  m_fFreq;
    DWORD                   m_nNextStatusTime;
    DWORD                   m_nFramesSinceUpdate;
    bool                    m_bSaveScreenshot;

    // Current Kinect
    IKinectSensor*          m_pKinectSensor;
    ICoordinateMapper*      m_pCoordinateMapper;
	ColorSpacePoint*		m_pColorCoordinatesTemp;
    ColorSpacePoint*        m_pColorCoordinates;
	ColorSpacePoint*		m_pColorCoordinatesDeformation;
	ColorSpacePoint*		m_pColorCoordinatesColor;
	DepthSpacePoint*		m_pDepthCoordiantes;
	CameraSpacePoint*		m_pCameraCoordinates;
	CameraSpacePoint*		m_pColorCameraCoordinates;

    // Frame reader
    IMultiSourceFrameReader*m_pMultiSourceFrameReader;

    // Direct2D
    ImageRenderer*          m_pDrawCoordinateMapping;
	ImageRenderer*			m_pDrawDeformation;
    ID2D1Factory*           m_pD2DFactory;
	ID2D1Factory*           m_pD2DFactoryDeformation;
	UINT16*					m_pDepthBuffer;
	RGBQUAD*				m_originalRGBX;
    RGBQUAD*                m_pOutputRGBX;
	RGBQUAD*                m_pOutputRGBXInterpolation;
    RGBQUAD*                m_pBackgroundRGBX; 
    RGBQUAD*                m_pColorRGBX;
	bool*					m_pBothColorSpaceValid; //color space in both situations are valid
	bool*					m_pValidColorMapping;
	int*					m_colorSpaceStatus;

	UINT16 *				m_pDepthBufferSrc;
	RGBQUAD*				m_pColorBufferSrc;
	BYTE*					m_pBodyIndexBufferSrc;
    /// <summary>
    /// Main processing function
    /// </summary>
    void                    Update();

    /// <summary>
    /// Initializes the default Kinect sensor
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                 InitializeDefaultSensor();
	//save one frame information
	void saveDepthColorImg(char* frameName, RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight, UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight);
	void loadDepthColorImg(char* frameName, RGBQUAD* &pColorBuffer, int nColorWidth, int nColorHeight, UINT16* &pDepthBuffer, int nDepthWidth, int nDephtHeight);
	void removeBackground(RGBQUAD* &pUserColor, RGBQUAD* pBGColor, UINT16* &pUserDepth, UINT16* pBGDepth, int left = 0, int right = 1920);
	void createMapping();
	void doDeformation();
	void doInterpolationMLSDeformation(RGBQUAD* &pDeformedImage, RGBQUAD* &pSrcImage);
	void generateNewImage();
	void correctedGazeofStaticImage();

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
	void					saveBackground(const UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight,
											const RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight);
    void                    ProcessFrame(INT64 nTime, 
                                         const UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight, 
                                         const RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight,
                                         const BYTE* pBodyIndexBuffer, int nBodyIndexWidth, int nBodyIndexHeight);
	void                    ProcessFrameInterpolation(INT64 nTime,
		const UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight,
		const RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight,
		const BYTE* pBodyIndexBuffer, int nBodyIndexWidth, int nBodyIndexHeight);
	void                    ProcessFrameByFeaturePoints(INT64 nTime,
										const UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight,
										const RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight,
										const BYTE* pBodyIndexBuffer, int nBodyIndexWidth, int nBodyIndexHeight);
	void					SavePointCloud(const UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight,
											RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight);
	void					saveColorPointCloud(const UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight,
												RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight,
												const BYTE* pBodyIndexBuffer, int nBodyIndexWidth, int nBodyIndexHeight);
	void					writePointCloudSharedMemory(const UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight,
												RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight,
												const BYTE* pBodyIndexBuffer, int nBodyIndexWidth, int nBodyIndexHeight);
    /// <summary>
    /// Set the status bar message
    /// </summary>
    /// <param name="szMessage">message to display</param>
    /// <param name="nShowTimeMsec">time in milliseconds for which to ignore future status messages</param>
    /// <param name="bForce">force status update</param>
    bool                    SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce);

    /// <summary>
    /// Get the name of the file where screenshot will be stored.
    /// </summary>
    /// <param name="lpszFilePath">string buffer that will receive screenshot file name.</param>
    /// <param name="nFilePathSize">number of characters in lpszFilePath string buffer.</param>
    /// <returns>
    /// S_OK on success, otherwise failure code.
    /// </returns>
    HRESULT                 GetScreenshotFileName(_Out_writes_z_(nFilePathSize) LPWSTR lpszFilePath, UINT nFilePathSize);

    /// <summary>
    /// Save passed in image data to disk as a bitmap
    /// </summary>
    /// <param name="pBitmapBits">image data to save</param>
    /// <param name="lWidth">width (in pixels) of input image data</param>
    /// <param name="lHeight">height (in pixels) of input image data</param>
    /// <param name="wBitsPerPixel">bits per pixel of image data</param>
    /// <param name="lpszFilePath">full file path to output bitmap to</param>
    /// <returns>indicates success or failure</returns>

    HRESULT                 SaveBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCWSTR lpszFilePath);
    /// <summary>
    /// Load an image from a resource into a buffer
    /// </summary>
    /// <param name="resourceName">name of image resource to load</param>
    /// <param name="resourceType">type of resource to load</param>
    /// <param name="nOutputWidth">width (in pixels) of scaled output bitmap</param>
    /// <param name="nOutputHeight">height (in pixels) of scaled output bitmap</param>
    /// <param name="pOutputBuffer">buffer that will hold the loaded image</param>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                 LoadResourceImage(PCWSTR resourceName, PCWSTR resourceType, UINT nOutputWidth, UINT nOutputHeight, RGBQUAD* pOutputBuffer);
	UINT16 cMaxDepthDistance, cMinDepthDistance;
	//check if the position is invalid 
	inline bool invalidColorSpace(ColorSpacePoint c);
	void fillTheHoleInColorspace(ColorSpacePoint* dst, ColorSpacePoint* src);
	template<class T> void smoothDepthImage(T* pDepthBuffer, int nDepthWidth, int nDepthHeight, int windowSize = 3);
	void filterCameraSpacePoints(CameraSpacePoint* pCameraSpacePoints, int width, int height, int windowSize = 3);
	//inline bool invalidDepth
	Eigen::Affine3d m_cameraTransformation;
	double m_rotationX;
	double m_rotationY;
	double m_rotationZ;
	Eigen::Translation3d m_cameraTranslation;

	bool m_updateNewframe;
	bool m_computeDeformation;
	bool m_enableSimpleInterpolation;
	bool m_enableMLSDeformation;
	bool m_saveBackground;
	bool m_saveHumanbody;
	//pointer for the shared memory
	HANDLE	m_hMapFile;
	void*	m_pBuf;
	size_t  m_bufSize;
	size_t  m_sizeOfFeaturePoints;
	size_t  m_sizeOfPointClouds;
};

