/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "holonomic_navigator_demoMain.h"
#include "EscapeGap.h"
#include <sbpl/headers.h>  
#include <vector>
#include <wx/msgdlg.h>
#include "CAboutBox.h"

#include "MapGenerator.h"
#include "PathPlanner.h"

// Librerias para utilizar variable random
#include <ctime>
#include <cstdlib>

// Librerias para sbpl
#include <cmath>
#include <cstring>
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>


//(*InternalHeaders(holonomic_navigator_demoFrame)
#include <wx/artprov.h>
#include <wx/bitmap.h>
#include <wx/tglbtn.h>
#include <wx/settings.h>
#include <wx/font.h>
#include <wx/intl.h>
#include <wx/image.h>
#include <wx/string.h>
//*)

#include <mrpt/gui/WxUtils.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include "imgs/main_icon.xpm"
#include "../wx-common/mrpt_logo.xpm"

#include "../ReactiveNavigationDemo/DEFAULT_GRIDMAP_DATA.h"

using namespace std;

#define numOfRegions 20
#define planificationFrecuency 50

#define Derecha 0
#define Izquierda 1

double distanceTraveled = 0;
double timeElapsed = 0;
double minimumDistanceToObstacle = 100;

struct timeval t_ini, t_fin;

// Archivos de debugg
ofstream data("data.txt", ios::app);
ofstream optimalRoad("optimalRoad.txt", ios::app);

//************ Planificador SBPL *********
   enum PlannerType
{
    INVALID_PLANNER_TYPE = -1,
    PLANNER_TYPE_ADSTAR,
    PLANNER_TYPE_ARASTAR,
    PLANNER_TYPE_PPCP,
    PLANNER_TYPE_RSTAR,
    PLANNER_TYPE_VI,
    PLANNER_TYPE_ANASTAR,

    NUM_PLANNER_TYPES
};

// A custom Art provider for customizing the icons:
class MyArtProvider : public wxArtProvider
{
protected:
    virtual wxBitmap CreateBitmap(const wxArtID& id,
                                  const wxArtClient& client,
                                  const wxSize& size);
};

// CreateBitmap function
wxBitmap MyArtProvider::CreateBitmap(const wxArtID& id,
                                     const wxArtClient& client,
                                     const wxSize& size)
{
    if (id == wxART_MAKE_ART_ID(MAIN_ICON))   return wxBitmap(main_icon_xpm);
    if (id == wxART_MAKE_ART_ID(IMG_MRPT_LOGO))  return wxBitmap(mrpt_logo_xpm);

    // Any wxWidgets icons not implemented here
    // will be provided by the default art provider
    return wxNullBitmap;
}

#include <mrpt/nav.h>
#include <mrpt/gui.h>
#include <mrpt/opengl.h>
#include <mrpt/system/filesystem.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace mrpt::nav;
using namespace mrpt::poses;
using namespace std;

//********************* Macros *****************************

#define timeSimul 100
#define step 100

//*********************************************************

//****************** Variables globales *********************

static float Sensor_Radius= 1.5;
static bool startDebugg = false;

#define initRobotPoseX -20.35
#define initRobotPoseY -9.8

#define initTargetPoseX 22.0
#define initTargetPoseY -6.0

#define resetRobotX -20.1
#define resetRobotY -8.1

#define resetTargetX 17.85
#define resetTargetY -5.42

//*************** PRUEBA MapGenerator ********************

double X;
double Y;
    
bool regionStatePlan[20];
bool regiones[20] = {false, false, false, false, false, false, false, true, true, true, true, true, true, false, false, false, false, false, false, false};

MapGenerator mapObj;    // Creacion de objeto de la clase encargada de formar el mapa a planificar,
                        // teniendo en cuenta el entorno con sus obstaculos

CPathPlanner planifObj; // Creacion de objeto de la clase encargada de la realizacion de la planificacion
                        // del camino del robot

parametros params;

//*********************************************************

//*********************************************************

//(*IdInit(holonomic_navigator_demoFrame)
const long holonomic_navigator_demoFrame::ID_BUTTON1 = wxNewId();
const long holonomic_navigator_demoFrame::ID_BUTTON2 = wxNewId();
const long holonomic_navigator_demoFrame::ID_BUTTON3 = wxNewId();
const long holonomic_navigator_demoFrame::ID_BUTTON8 = wxNewId(); // NEW!
const long holonomic_navigator_demoFrame::ID_BUTTON6 = wxNewId();
const long holonomic_navigator_demoFrame::ID_BUTTON7 = wxNewId();
const long holonomic_navigator_demoFrame::ID_BUTTON4 = wxNewId();
const long holonomic_navigator_demoFrame::ID_BUTTON5 = wxNewId();
const long holonomic_navigator_demoFrame::ID_RADIOBOX1 = wxNewId();
const long holonomic_navigator_demoFrame::ID_TEXTCTRL1 = wxNewId();
const long holonomic_navigator_demoFrame::ID_PANEL1 = wxNewId();
const long holonomic_navigator_demoFrame::ID_PANEL2 = wxNewId();
const long holonomic_navigator_demoFrame::ID_NOTEBOOK1 = wxNewId();
const long holonomic_navigator_demoFrame::ID_STATICTEXT2 = wxNewId();
const long holonomic_navigator_demoFrame::ID_STATICTEXT1 = wxNewId();
const long holonomic_navigator_demoFrame::ID_XY_GLCANVAS = wxNewId();
const long holonomic_navigator_demoFrame::ID_CUSTOM1 = wxNewId();
const long holonomic_navigator_demoFrame::ID_TEXTCTRL2 = wxNewId();
const long holonomic_navigator_demoFrame::ID_MENUITEM4 = wxNewId();
const long holonomic_navigator_demoFrame::idMenuQuit = wxNewId();
const long holonomic_navigator_demoFrame::ID_MENUITEM1 = wxNewId();
const long holonomic_navigator_demoFrame::ID_MENUITEM2 = wxNewId();
const long holonomic_navigator_demoFrame::ID_MENUITEM3 = wxNewId();
const long holonomic_navigator_demoFrame::idMenuAbout = wxNewId();
const long holonomic_navigator_demoFrame::ID_STATUSBAR1 = wxNewId();
const long holonomic_navigator_demoFrame::ID_TIMER1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(holonomic_navigator_demoFrame,wxFrame)
    //(*EventTable(holonomic_navigator_demoFrame)
    //*)
END_EVENT_TABLE()

holonomic_navigator_demoFrame::holonomic_navigator_demoFrame(wxWindow* parent,wxWindowID id) :
	m_holonomicMethod( NULL ),
	m_gridMap(),
	m_targetPoint(initTargetPoseX, initTargetPoseY),
	m_robotPose(initRobotPoseX, initRobotPoseY, 0),
	m_curCursorPos(0,0),
	m_cursorPickState(cpsNone)
{
    // Load my custom icons:
#if wxCHECK_VERSION(2, 8, 0)
    wxArtProvider::Push(new MyArtProvider);
#else
    wxArtProvider::PushProvider(new MyArtProvider);
#endif

    //(*Initialize(holonomic_navigator_demoFrame)
    wxFlexGridSizer* FlexGridSizer4;
    wxMenuItem* MenuItem2;
    wxFlexGridSizer* FlexGridSizer3;
    wxMenuItem* MenuItem1;
    wxFlexGridSizer* FlexGridSizer5;
    wxFlexGridSizer* FlexGridSizer2;
    wxBoxSizer* BoxSizer2;
    wxMenu* Menu1;
    wxFlexGridSizer* FlexGridSizer7;
    wxBoxSizer* BoxSizer1;
    wxMenuBar* MenuBar1;
    wxFlexGridSizer* FlexGridSizer1;
    wxMenu* Menu2;

    Create(parent, id, _("Holonomic Navigation Demo - Part of MRPT"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("id"));
    FlexGridSizer1 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer1->AddGrowableCol(0);
    FlexGridSizer1->AddGrowableRow(1);
    FlexGridSizer2 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer2->AddGrowableCol(1);
    FlexGridSizer4 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer4->AddGrowableCol(0);
    FlexGridSizer4->AddGrowableRow(0);
    FlexGridSizer4->AddGrowableRow(1);
    BoxSizer1 = new wxBoxSizer(wxHORIZONTAL);
    btnLoadMap = new wxCustomButton(this,ID_BUTTON1,_("Load map..."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FOLDER")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON1"));
    btnLoadMap->SetBitmapDisabled(btnLoadMap->CreateBitmapDisabled(btnLoadMap->GetBitmapLabel()));
    btnLoadMap->SetBitmapMargin(wxSize(2,4));
    BoxSizer1->Add(btnLoadMap, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 1);
    btnHelp = new wxCustomButton(this,ID_BUTTON2,_("About..."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_QUESTION")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON2"));
    btnHelp->SetBitmapDisabled(btnHelp->CreateBitmapDisabled(btnHelp->GetBitmapLabel()));
    btnHelp->SetBitmapMargin(wxSize(5,4));
    BoxSizer1->Add(btnHelp, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 1);
    btnQuit = new wxCustomButton(this,ID_BUTTON3,_("Exit"),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_QUIT")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON3"));
    btnQuit->SetBitmapDisabled(btnQuit->CreateBitmapDisabled(btnQuit->GetBitmapLabel()));
    BoxSizer1->Add(btnQuit, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 1);
    FlexGridSizer4->Add(BoxSizer1, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    BoxSizer2 = new wxBoxSizer(wxHORIZONTAL);
    // NEW!
    btnReset= new wxCustomButton(this,ID_BUTTON8,_("Reset"),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_CROSS_MARK")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON8"));
    btnReset->SetBitmapDisabled(btnReset->CreateBitmapDisabled(btnReset->GetBitmapLabel()));
    btnReset->SetBitmapMargin(wxSize(2,4));
    BoxSizer1->Add(btnReset, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 1);
    //*****
    btnPlaceRobot = new wxCustomButton(this,ID_BUTTON6,_("Place robot..."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FIND")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON6"));
    btnPlaceRobot->SetBitmapDisabled(btnPlaceRobot->CreateBitmapDisabled(btnPlaceRobot->GetBitmapLabel()));
    btnPlaceRobot->SetBitmapMargin(wxSize(2,4));
    BoxSizer2->Add(btnPlaceRobot, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 2);
    btnPlaceTarget = new wxCustomButton(this,ID_BUTTON7,_("Set target..."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FIND")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON7"));
    btnPlaceTarget->SetBitmapDisabled(btnPlaceTarget->CreateBitmapDisabled(btnPlaceTarget->GetBitmapLabel()));
    btnPlaceTarget->SetBitmapMargin(wxSize(2,4));
    BoxSizer2->Add(btnPlaceTarget, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 2);
    btnStart = new wxCustomButton(this,ID_BUTTON4,_("START"),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_GO_FORWARD")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON4"));
    btnStart->SetBitmapDisabled(btnStart->CreateBitmapDisabled(btnStart->GetBitmapLabel()));
    btnStart->SetBitmapMargin(wxSize(2,4));
    BoxSizer2->Add(btnStart, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 2);
    btnStop = new wxCustomButton(this,ID_BUTTON5,_("STOP"),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_CROSS_MARK")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON5"));
    btnStop->SetBitmapDisabled(btnStop->CreateBitmapDisabled(btnStop->GetBitmapLabel()));
    btnStop->SetBitmapMargin(wxSize(2,4));
    BoxSizer2->Add(btnStop, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 2);
    FlexGridSizer4->Add(BoxSizer2, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer2->Add(FlexGridSizer4, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    Notebook1 = new wxNotebook(this, ID_NOTEBOOK1, wxDefaultPosition, wxDefaultSize, 0, _T("ID_NOTEBOOK1"));
    Panel1 = new wxPanel(Notebook1, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
    FlexGridSizer7 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer7->AddGrowableCol(1);
    FlexGridSizer7->AddGrowableRow(0);
    wxString __wxRadioBoxChoices_1[3] =
    {
    	_("VFF (Virtual Force Field)"),
    	_("ND (Nearness  Diagram)"),
        _("EG (Escape Gap)")
    };
    rbHoloMethod = new wxRadioBox(Panel1, ID_RADIOBOX1, _(" Holonomic method "), wxDefaultPosition, wxSize(-1,-1), 3, __wxRadioBoxChoices_1, 1, wxRA_HORIZONTAL, wxDefaultValidator, _T("ID_RADIOBOX1"));
    rbHoloMethod->SetSelection(2);
    FlexGridSizer7->Add(rbHoloMethod, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edHoloParams = new wxTextCtrl(Panel1, ID_TEXTCTRL1, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER|wxTE_PROCESS_TAB|wxTE_MULTILINE|wxVSCROLL|wxHSCROLL|wxALWAYS_SHOW_SB, wxDefaultValidator, _T("ID_TEXTCTRL1"));
    edHoloParams->SetMinSize(wxSize(-1,100));
    wxFont edHoloParamsFont(8,wxTELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    edHoloParams->SetFont(edHoloParamsFont);
    FlexGridSizer7->Add(edHoloParams, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 2);
    Panel1->SetSizer(FlexGridSizer7);
    FlexGridSizer7->Fit(Panel1);
    FlexGridSizer7->SetSizeHints(Panel1);
    Panel2 = new wxPanel(Notebook1, ID_PANEL2, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL2"));
    Notebook1->AddPage(Panel1, _("Configuration"), true);
    Notebook1->AddPage(Panel2, _("Stats"), false);
    FlexGridSizer2->Add(Notebook1, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 2);
    FlexGridSizer1->Add(FlexGridSizer2, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer3 = new wxFlexGridSizer(2, 2, 0, 0);
    FlexGridSizer3->AddGrowableCol(0);
    FlexGridSizer3->AddGrowableCol(1);
    FlexGridSizer3->AddGrowableRow(1);
    StaticText2 = new wxStaticText(this, ID_STATICTEXT2, _("[3D view of the simulated world]"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
    FlexGridSizer3->Add(StaticText2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText1 = new wxStaticText(this, ID_STATICTEXT1, _("[Local view]"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
    FlexGridSizer3->Add(StaticText1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    m_plot3D = new CMyGLCanvas(this,ID_XY_GLCANVAS,wxDefaultPosition,wxSize(450,350),wxTAB_TRAVERSAL,_T("ID_XY_GLCANVAS"));
    FlexGridSizer3->Add(m_plot3D, 1, wxALL|wxEXPAND|wxFIXED_MINSIZE|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 1);
    FlexGridSizer5 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer5->AddGrowableCol(0);
    FlexGridSizer5->AddGrowableRow(0);
    m_plotScan = new CMyGLCanvas(this,ID_CUSTOM1,wxDefaultPosition,wxSize(150,150),wxTAB_TRAVERSAL,_T("ID_CUSTOM1"));
    FlexGridSizer5->Add(m_plotScan, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 1);
    edInfoLocalView = new wxTextCtrl(this, ID_TEXTCTRL2, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE|wxTE_READONLY|wxTE_DONTWRAP|wxALWAYS_SHOW_SB, wxDefaultValidator, _T("ID_TEXTCTRL2"));
    edInfoLocalView->SetMinSize(wxSize(-1,100));
    edInfoLocalView->SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BTNFACE));
    wxFont edInfoLocalViewFont(8,wxTELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    edInfoLocalView->SetFont(edInfoLocalViewFont);
    FlexGridSizer5->Add(edInfoLocalView, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 2);
    FlexGridSizer3->Add(FlexGridSizer5, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer1->Add(FlexGridSizer3, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    SetSizer(FlexGridSizer1);
    MenuBar1 = new wxMenuBar();
    Menu1 = new wxMenu();
    MenuItem3 = new wxMenuItem(Menu1, ID_MENUITEM4, _("Load map..."), wxEmptyString, wxITEM_NORMAL);
    Menu1->Append(MenuItem3);
    MenuItem1 = new wxMenuItem(Menu1, idMenuQuit, _("Quit\tAlt-F4"), _("Quit the application"), wxITEM_NORMAL);
    Menu1->Append(MenuItem1);
    MenuBar1->Append(Menu1, _("&File"));
    Menu3 = new wxMenu();
    mnuViewMaxRange = new wxMenuItem(Menu3, ID_MENUITEM1, _("Maximum sensor range"), wxEmptyString, wxITEM_CHECK);
    Menu3->Append(mnuViewMaxRange);
    mnuViewRobotPath = new wxMenuItem(Menu3, ID_MENUITEM2, _("Robot path"), wxEmptyString, wxITEM_CHECK);
    Menu3->Append(mnuViewRobotPath);
    Menu3->AppendSeparator();
    MenuItem5 = new wxMenuItem(Menu3, ID_MENUITEM3, _("Clear robot path"), wxEmptyString, wxITEM_NORMAL);
    Menu3->Append(MenuItem5);
    MenuBar1->Append(Menu3, _("&View"));
    Menu2 = new wxMenu();
    MenuItem2 = new wxMenuItem(Menu2, idMenuAbout, _("About\tF1"), _("Show info about this application"), wxITEM_NORMAL);
    Menu2->Append(MenuItem2);
    MenuBar1->Append(Menu2, _("Help"));
    SetMenuBar(MenuBar1);
    StatusBar1 = new wxStatusBar(this, ID_STATUSBAR1, 0, _T("ID_STATUSBAR1"));
    int __wxStatusBarWidths_1[3] = { -2, -2, -3 };
    int __wxStatusBarStyles_1[3] = { wxSB_NORMAL, wxSB_NORMAL, wxSB_NORMAL };
    StatusBar1->SetFieldsCount(3,__wxStatusBarWidths_1);
    StatusBar1->SetStatusStyles(3,__wxStatusBarStyles_1);
    SetStatusBar(StatusBar1);
    timRunSimul.SetOwner(this, ID_TIMER1);
    timRunSimul.Start(timeSimul, false);
    FlexGridSizer1->Fit(this);
    FlexGridSizer1->SetSizeHints(this);
    Center();

    Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnbtnLoadMapClick);
    Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnAbout);
    Connect(ID_BUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnQuit);
    Connect(ID_BUTTON8,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::Reset); //NEW!!!
    Connect(ID_BUTTON6,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnbtnPlaceRobotClick);
    Connect(ID_BUTTON7,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnbtnPlaceTargetClick);
    Connect(ID_BUTTON4,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnbtnStartClick);
    Connect(ID_BUTTON5,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnbtnStopClick);
    Connect(ID_MENUITEM4,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnbtnLoadMapClick);
    Connect(idMenuQuit,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnQuit);
    Connect(ID_MENUITEM1,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnMenuItemChangeVisibleStuff);
    Connect(ID_MENUITEM2,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnMenuItemChangeVisibleStuff);
    Connect(ID_MENUITEM3,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnMenuItemClearRobotPath);
    Connect(idMenuAbout,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnAbout);
    Connect(ID_TIMER1,wxEVT_TIMER,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OntimRunSimulTrigger);
    //*)

	m_plot3D->Connect(wxEVT_MOTION,(wxObjectEventFunction)&holonomic_navigator_demoFrame::Onplot3DMouseMove,0,this);
	m_plot3D->Connect(wxEVT_LEFT_DOWN,(wxObjectEventFunction)&holonomic_navigator_demoFrame::Onplot3DMouseClick,0,this);

	mnuViewMaxRange->Check(true);
	mnuViewRobotPath->Check(true);

	btnStart->Enable(true);
	btnStop->Enable(false);

	WX_START_TRY

	// Initialize gridmap:
	// -------------------------------
	CMemoryStream  s( DEFAULT_GRIDMAP_DATA, sizeof(DEFAULT_GRIDMAP_DATA) );
	s >> m_gridMap;

	// Populate 3D views:
	// -------------------------------
	{
		mrpt::opengl::CGridPlaneXYPtr obj = mrpt::opengl::CGridPlaneXY::Create(-50,50, -50,50, 0, 1);
		obj->setColor_u8(TColor(30,30,30,50));
		m_plot3D->m_openGLScene->insert( obj );
	}

	gl_grid = mrpt::opengl::CSetOfObjects::Create();
	m_plot3D->m_openGLScene->insert(gl_grid);
	this->updateMap3DView();

	gl_robot = mrpt::opengl::CSetOfObjects::Create();
	{
		mrpt::opengl::CCylinderPtr obj = mrpt::opengl::CCylinder::Create(0.2,0.1,0.9);
		obj->setColor_u8( TColor::red );
		gl_robot->insert( obj );
	}
	m_plot3D->m_openGLScene->insert(gl_robot);

    //******** PUNTOS DEL LASER ************
    if (!draw) {
    	gl_scan3D = mrpt::opengl::CPlanarLaserScan::Create();
    	gl_scan3D->enableLine(false);
    	gl_scan3D->setPointsWidth(3.0);
    	gl_robot->insert(gl_scan3D);
    }

    if (!draw) {
	   gl_robot_sensor_range = mrpt::opengl::CDisk::Create(0,0);
	   gl_robot_sensor_range->setColor_u8(TColor(0,0,255,90));
	   gl_robot_sensor_range->setLocation(0,0,0.01);
	   gl_robot->insert(gl_robot_sensor_range);
    }

    //******** LINEA DEL CAMINO SEGUIDO ********
	gl_robot_path = mrpt::opengl::CSetOfLines::Create();
	gl_robot_path->setLineWidth(7);
	gl_robot_path->setColor_u8(TColor(52,152,219,255));
	m_plot3D->m_openGLScene->insert(gl_robot_path);

    //******* LINEA CAMINO OPTIMO *************
    gl_planificator_path = mrpt::opengl::CSetOfLines::Create();
    gl_planificator_path->setLineWidth(5);
    gl_planificator_path->setColor_u8(TColor(255,128,0,255));
    m_plot3D->m_openGLScene->insert(gl_planificator_path);

    //********** LAS FLECHAS DEL TARGET ****************
	gl_target = mrpt::opengl::CSetOfObjects::Create();
	{
		mrpt::opengl::CArrowPtr obj;
		obj = mrpt::opengl::CArrow::Create( 1,0,0,  0.2,0,0,  0.4,0.05, 0.15 ); obj->setColor_u8( TColor(0,0,255) ); gl_target->insert(obj);
		obj = mrpt::opengl::CArrow::Create(-1,0,0, -0.2,0,0,  0.4,0.05, 0.15 ); obj->setColor_u8( TColor(0,0,255) ); gl_target->insert(obj);
		obj = mrpt::opengl::CArrow::Create( 0,1,0,  0,0.2,0,  0.4,0.05, 0.15 ); obj->setColor_u8( TColor(0,0,255) ); gl_target->insert(obj);
		obj = mrpt::opengl::CArrow::Create(0,-1,0,  0,-0.2,0, 0.4,0.05, 0.15 ); obj->setColor_u8( TColor(0,0,255) ); gl_target->insert(obj);
		m_plot3D->m_openGLScene->insert(gl_target);
	}

	{	// Sign of "picking a navigation target":
		m_gl_placing_nav_target = opengl::CSetOfObjects::Create();

		mrpt::opengl::CArrowPtr obj;
		obj = mrpt::opengl::CArrow::Create( 1,0,0,  0.2,0,0,  0.4,0.05, 0.15 ); obj->setColor_u8( TColor(0,0,255) ); m_gl_placing_nav_target->insert(obj);
		obj = mrpt::opengl::CArrow::Create(-1,0,0, -0.2,0,0,  0.4,0.05, 0.15 ); obj->setColor_u8( TColor(0,0,255) ); m_gl_placing_nav_target->insert(obj);
		obj = mrpt::opengl::CArrow::Create( 0,1,0,  0,0.2,0,  0.4,0.05, 0.15 ); obj->setColor_u8( TColor(0,0,255) ); m_gl_placing_nav_target->insert(obj);
		obj = mrpt::opengl::CArrow::Create(0,-1,0,  0,-0.2,0, 0.4,0.05, 0.15 ); obj->setColor_u8( TColor(0,0,255) ); m_gl_placing_nav_target->insert(obj);
		m_gl_placing_nav_target->setVisibility(false); // Start invisible
		m_plot3D->m_openGLScene->insert(m_gl_placing_nav_target);
	}

	{	// Sign of "replacing the robot":
		m_gl_placing_robot = opengl::CSetOfObjects::Create();
		mrpt::opengl::CCylinderPtr obj = mrpt::opengl::CCylinder::Create(0.2,0.1,0.9);
		obj->setColor_u8(TColor(255,0,0,120));
		m_gl_placing_robot->insert(obj);
		m_gl_placing_robot->setVisibility(false); // Start invisible
		m_plot3D->m_openGLScene->insert(m_gl_placing_robot);
	}

	m_plot3D->m_openGLScene->insert(mrpt::opengl::stock_objects::CornerXYZ(1));

	// Set camera:
	m_plot3D->cameraPointingX=0;
	m_plot3D->cameraPointingY=0;
	m_plot3D->cameraPointingZ=0;
	m_plot3D->cameraZoomDistance = 40;
	m_plot3D->cameraElevationDeg = 70;
	m_plot3D->cameraAzimuthDeg = -100;
	m_plot3D->cameraIsProjective = true;

	// 2D view ==============
	{
		mrpt::opengl::CGridPlaneXYPtr obj = mrpt::opengl::CGridPlaneXY::Create(-1,1.001, -1,1.001, 0, 1);
		obj->setColor_u8(TColor(30,30,30,50));
		m_plotScan->m_openGLScene->insert(obj);
	}

    gl_scan2D = mrpt::opengl::CPlanarLaserScan::Create();
    gl_scan2D->enableLine(false);
    gl_scan2D->enableSurface(false);
    gl_scan2D->setPointsWidth(3.0);
    m_plotScan->m_openGLScene->insert(gl_scan2D);

    //*********** LINEA DE DIRECCION ************	
    gl_line_direction = mrpt::opengl::CSimpleLine::Create();
	gl_line_direction->setLineWidth(4);
	gl_line_direction->setColor_u8(TColor(0,0,0));
	m_plotScan->m_openGLScene->insert(gl_line_direction);

	gl_rel_target = mrpt::opengl::CPointCloud::Create();
	gl_rel_target->setPointSize(7);
	gl_rel_target->setColor_u8(TColor(0,0,255));
	gl_rel_target->insertPoint(0,0,0);
	m_plotScan->m_openGLScene->insert(gl_rel_target);

    gl_subgoal = mrpt::opengl::CPointCloud::Create();
    gl_subgoal->setPointSize(7);
    gl_subgoal->setColor_u8(TColor(0,0,255));
    gl_subgoal->insertPoint(0,0,0);
    m_plot3D->m_openGLScene->insert(gl_subgoal);

	m_plotScan->m_openGLScene->insert(mrpt::opengl::stock_objects::CornerXYSimple(0.1,2));

	gl_nd_gaps= mrpt::opengl::CSetOfLines::Create();
	gl_nd_gaps->setLineWidth(2);
	gl_nd_gaps->setColor_u8(TColor(255,0,0));
	m_plotScan->m_openGLScene->insert(gl_nd_gaps);

    // Llamada al constructor de la clase CHolonomicEG. En caso de querer dibujar
    // las regiones alrededor del robot, entre otras cosas para la obtencion de datos
    // visuales, el constructor llamado seria uno distinto
    if (draw) {
        m_EscapeGap = new CHolonomicEG(numOfRegions, gl_robot, m_plotScan, &params); // DIBUJAR
    } else {
        m_EscapeGap = new CHolonomicEG(numOfRegions, m_plotScan, &params); 
    }

    //**************** PRUEBA **********************

        // Se abre el archivo del mapa
        mapObj.openFile(Trunc); // Normal (no borra), Trunc (borra)

        if (debuggLines) cout<<"Linea 512"<<endl;
        mapObj.initData();
            
        mapObj.readData();
        if (debuggLines) cout<<"Linea 516"<<endl;

        mapObj.mapDimension();
        if (debuggLines) cout<<"Linea 519"<<endl;

        mapObj.readMap();
        mapObj.initVariables(m_Accuracy, m_robotPose.x, m_robotPose.y, m_targetPoint.x, m_targetPoint.y);
        mapObj.writeData();
       
        mapObj.closeFile();

    //*************** FIN DE PRUEBA ***************

	m_plotScan->clearColorR = 0.9f;
	m_plotScan->clearColorG = 0.9f;
	m_plotScan->clearColorB = 0.9f;

	// Set camera:
	m_plotScan->cameraPointingX=0;
	m_plotScan->cameraPointingY=0;
	m_plotScan->cameraPointingZ=0;
	m_plotScan->cameraZoomDistance = 2.2;
	m_plotScan->cameraElevationDeg = 90;
	m_plotScan->cameraAzimuthDeg = -90;
	m_plotScan->cameraIsProjective = false;

	// Update positions of stuff:
	this->updateViewsDynamicObjects();


	// Retrieve default parameters for holonomic methods:
	// ------------------------------------------------------
	{
		mrpt::utils::CConfigFileMemory cfg;

		m_simul_options.saveToConfigFile(cfg,"SIMULATOR");

		mrpt::nav::CHolonomicVFF holo_VFF;
		holo_VFF.options.saveToConfigFile(cfg,"VFF_CONFIG");

		mrpt::nav::CHolonomicND holo_ND;
		holo_ND.options.saveToConfigFile(cfg,"ND_CONFIG");

		this->edHoloParams->SetValue( _U( cfg.getContent().c_str() ) );
	}

	WX_END_TRY


	this->Maximize();
}

holonomic_navigator_demoFrame::~holonomic_navigator_demoFrame()
{
    //(*Destroy(holonomic_navigator_demoFrame)
    //*)
}

void holonomic_navigator_demoFrame::OnQuit(wxCommandEvent& event)
{
    Close();
}

void holonomic_navigator_demoFrame::OnAbout(wxCommandEvent& event)
{
	CAboutBox dlg(this);
	dlg.ShowModal();
}

void holonomic_navigator_demoFrame::updateMap3DView()
{
	gl_grid->clear();
	m_gridMap.getAs3DObject(gl_grid);
}

void holonomic_navigator_demoFrame::OnbtnPlaceRobotClick(wxCommandEvent& event)
{
	if (m_cursorPickState!=cpsPlaceRobot)
	{
		m_cursorPickState = cpsPlaceRobot;
		m_plot3D->SetCursor( *wxCROSS_CURSOR );
	}
	else
	{	// Cancel:
		m_cursorPickState = cpsNone;
		m_plot3D->SetCursor( *wxSTANDARD_CURSOR );
		m_gl_placing_robot->setVisibility(false);
	}
	btnPlaceRobot->SetValue( m_cursorPickState == cpsPlaceRobot );
	btnPlaceRobot->Refresh();

    updateViewsDynamicObjects();
}

void holonomic_navigator_demoFrame::OnbtnPlaceTargetClick(wxCommandEvent& event)
{
	if (m_cursorPickState!=cpsPickTarget)
	{
		m_cursorPickState = cpsPickTarget;
		m_plot3D->SetCursor( *wxCROSS_CURSOR );
	}
	else
	{	// Cancel:
		m_cursorPickState = cpsNone;
		m_plot3D->SetCursor( *wxSTANDARD_CURSOR );
		m_gl_placing_nav_target->setVisibility(false);
	}
	btnPlaceTarget->SetValue( m_cursorPickState == cpsPickTarget );
	btnPlaceTarget->Refresh();
}

void holonomic_navigator_demoFrame::OnbtnStartClick(wxCommandEvent& event)
{
	reinitSimulator();

	btnStart->Enable(false); btnStart->Refresh();
	btnStop->Enable(true);   btnStop->Refresh();
	edHoloParams->Enable(false);
	rbHoloMethod->Enable(false);

    //************
    gettimeofday(&t_ini, NULL);
    //************
}

void holonomic_navigator_demoFrame::OnbtnStopClick(wxCommandEvent& event)
{
	btnStart->Enable(true); btnStart->Refresh();
	btnStop->Enable(false);   btnStop->Refresh();
	edHoloParams->Enable(true);
	rbHoloMethod->Enable(true);
    startDebugg = true;

    //************
    gettimeofday(&t_fin, NULL);
    cout<<"Impresion de tiempo transcurrido"<<endl;
    timeval_diff(&t_ini, &t_fin);
    //************
}

// Run simulator (when "running"):
void holonomic_navigator_demoFrame::OntimRunSimulTrigger(wxTimerEvent& event)
{
	try
	{
		if (btnStop->IsEnabled())
		{
			simulateOneStep(step*1e-3);
		}
		updateViewsDynamicObjects();
	}
	catch (std::exception &e)
	{
		wxMessageBox( wxString(e.what(),wxConvUTF8), wxT("Exception"), wxOK, this);
		// Stop:
		wxCommandEvent ev;
		OnbtnStopClick(ev);
	}
}

// Create navigator object & load params from GUI:
void holonomic_navigator_demoFrame::reinitSimulator()
{
	WX_START_TRY

	// Delete old & build new navigator:
	mrpt::utils::delete_safe(m_holonomicMethod);
	switch (rbHoloMethod->GetSelection())
	{
	case 0: m_holonomicMethod = new mrpt::nav::CHolonomicVFF(); break;
	case 1: m_holonomicMethod = new mrpt::nav::CHolonomicND(); break;
    case 2: m_holonomicMethod = new mrpt::nav::CHolonomicND(); break;
	default:
		throw std::runtime_error("Invalid holonomic method selected!");
	};

	// Load params:
	{
		CConfigFileMemory cfg;
		cfg.setContent( std::string(edHoloParams->GetValue().mb_str() ) );
		m_holonomicMethod->initialize(cfg);

		m_simul_options.loadFromConfigFile(cfg,"SIMULATOR");
	}

	// Update GUI stuff:
	if (!draw) gl_robot_sensor_range->setDiskRadius(m_simul_options.MAX_SENSOR_RADIUS*1.01,m_simul_options.MAX_SENSOR_RADIUS*0.99);

	WX_END_TRY
}


bool changeObjective = false;

void holonomic_navigator_demoFrame::Reset(wxCommandEvent& event)
{ startDebugg = startDebugg ? false : true; }

void holonomic_navigator_demoFrame::simulateOneStep(double time_step)
{
    static bool onlyOneTime = false;

    //*********** For data ****************
    static int mainCycles = 0; // Debugging
    mainCycles++; // Debugging
    //*************************************

    //****** Inicializar PathPlanner.h ******
    cout<<"********To Test PathPlanner.h***********"<<endl;
    int XstartAbsolute;
    int YstartAbsolute;
    int Xcell;
    int Ycell;
    std::vector<int> dataEnvironment;
    //************************************

    if (debuggMapFail) cout<<"Line 731"<<endl;
	// Simulate 360deg range scan: 
	CObservation2DRangeScan simulatedScan;

	simulatedScan.aperture = M_2PI;
	simulatedScan.rightToLeft = true;
	simulatedScan.maxRange = Sensor_Radius;
	simulatedScan.sensorPose = CPose2D(0,0,0);

	m_gridMap.laserScanSimulator( simulatedScan, m_robotPose,0.5, m_simul_options.SENSOR_NUM_RANGES, m_simul_options.SENSOR_RANGE_NOISE_STD );

	if (!draw) gl_scan3D->setScan( simulatedScan );  // Draw real scan in 3D view 

	// Normalize:
	for (size_t j=0;j<simulatedScan.scan.size();j++) simulatedScan.scan[j] /= simulatedScan.maxRange;

	gl_scan2D->setScan( simulatedScan ); // Draw scaled scan in right-hand view

	// Navigate:
	mrpt::math::TPoint2D finalRelTargetPose = mrpt::math::TPoint2D( mrpt::poses::CPoint2D(m_targetPoint) - mrpt::poses::CPose2D(m_robotPose) );
	finalRelTargetPose*= 1.0/simulatedScan.maxRange; // Normalized relative target:

	double desiredDirection,desiredSpeed;
	mrpt::nav::CHolonomicLogFileRecordPtr  out_log;

    //************************* PARTE DELIBERATIVA ********************************************
    static int cycles = 51;
    static bool playPlanification = true; // Esta variable indica si se debe realizar
                                          // la planificacion en el ciclo actual
    static bool replanification = false;
    static bool makeReplanification = true;

    cout<<"Empezamos"<<endl;
    
    if (debuggMapFail) cout<<"Line 767"<<endl;

    if (playPlanification || roadPlanificator) {

        if (roadPlanificator) {
            mapObj.openFile(Normal);
            if (debuggMapFail) cout<<"Line 773"<<endl;

            mapObj.obstaclesDetection(m_robotPose.x, m_robotPose.y, simulatedScan.scan, simulatedScan.maxRange);

            // Paso de variables entre la clase generador del mapa y la clase que se 
            // encarga de la planificación del camino del robot
            mapObj.getStartAbsolute(XstartAbsolute, YstartAbsolute);
            cout<<"startAbsolute = ("<<XstartAbsolute<<", "<<YstartAbsolute<<")"<<endl;
            planifObj.setStartAbsolute(XstartAbsolute, YstartAbsolute);
            planifObj.setMapAccuracy(m_Accuracy);
            mapObj.getDataEnvironment(dataEnvironment);
            cout<<"Data after obstaclesDetection con size: "<<dataEnvironment.size()<<endl;
            planifObj.setDataEnvironment(dataEnvironment);
            mapObj.getCellsInformation(Xcell, Ycell);
            cout<<"cellsInformation: ("<<Xcell<<", "<<Ycell<<")"<<endl;
            planifObj.setCellsInformation(Xcell, Ycell);

            if (debuggMapFail) cout<<"Line 777"<<endl;
            mapObj.closeFile();
            if (debuggMapFail) cout<<"Line 779"<<endl;
            mapObj.openFile(Trunc);
            if (debuggMapFail) cout<<"Line 781"<<endl;
            mapObj.writeData();
            if (debuggMapFail) cout<<"Line 783"<<endl;
            mapObj.closeFile();
        }

        if (debuggMapFail) cout<<"Line 786"<<endl;

        cycles++;

        if (makeReplanification && planificatorActive) {
            replanification = true;
            cout<<"------incond-----makeReplanification = "<<makeReplanification<<endl;
            makeReplanification = false;
            cycles = 0;

            planifObj.resetGoals();

            if (searchSolution) {

                cout<<"Replanificando..."<<endl;
                planifObj.executePlanification();
                cout<<"******Calculo de time de planificacion******"<<endl;
            }
            planifObj.openSol();
            // Limpiado de vectores contenedores del camino optimo
            XoptimalRoad.clear();
            YoptimalRoad.clear();
            planifObj.readSolution(XoptimalRoad, YoptimalRoad);
            planifObj.closeSol();
            cout<<"XoptimalRoad.size() despues replanification = "<<XoptimalRoad.size()<<endl;
        }
    }  
        
    if (debuggMapFail) cout<<"Line 897"<<endl;
    //************************** END PARTE DELIBERATIVA *********************************


    // Contador de obstáculos visibles
    int obstacleCounter=0;
    for (size_t p=0; p<simulatedScan.scan.size(); p++) {
        if(simulatedScan.scan[p]<0.98) obstacleCounter++;
    }

    // Variables que se encargan de modificar el comportamiento del robot
    static mrpt::math::TPoint2D realRelTargetPose = finalRelTargetPose; // Inicialmente no existe un
    static mrpt::math::TPoint2D relTargetPose = realRelTargetPose;      // subgoal, unicamente el destino final
    static bool targetSeen = true;      // Se activa cuando el robot avista su destino actual
    static bool nearSubGoal = false;    // Se activa cuando el robot se encuentra cerca del actual
                                        // subgoal. Cada ciclo el valor es recalculado
    static bool nearSubGoalActive = false; // Al contrario que nearSubgoal, puede estar activo
                                           // durante varios ciclos hasta ser desactivado
    static bool finalTargetSeen = false;   // Se avista cuando el robot avista su destino final
    static bool restartSimulation = false; // Al estar activo, se realiza un reset de la simulacion
    static bool changeDirection = false;   // Al activarse indica al robot que debe decidir una nueva direccion
    static bool firstSolution = true;   // Realiza ciertas operaciones solo en el primer ciclo
    // Angulo del punto de destino respecto del robot
    static double targetAngle = atan2(m_targetPoint.y - m_robotPose.y, m_targetPoint.x - m_robotPose.x);
    
    // Al no haber subgoals cuando el planificador se desactiva, no es necesario indicar
    // cuando se esta cerca de un subgoal, porque no existen
    if (!planificatorActive) nearSubGoal = false;

    // Cuando se indica que hay un cambio de objetivo al topar con un obstaculo entre
    // el robot y el destino, el booleando targetSeen, que indica que el punto de destino
    // esta avistado, se desactiva
    if (changeObjective) {
        targetSeen = false;  
        changeObjective = false;
    }

    //**************** PRINT DATA OPTIMAL ROAD **************
    if (debuggMapFail) cout<<"Line 920"<<endl;

    // Comprobacion del camino optimo actual, calculo de la distancia que tiene el camino
    // planificado y dibujo del camino planificado
    if (planificatorActive) {

        double optimalDistance = 0.0;
        if (firstWayToCheckPath)
            makeReplanification = planifObj.checkOptimusRoad(XoptimalRoad, YoptimalRoad);
        else makeReplanification = planifObj.checkOptimusRoad_alternative(XoptimalRoad, YoptimalRoad);

        cout<<"--------makeReplanification = "<<makeReplanification<<endl;

        gl_planificator_path->clear();
        for (int i = 0; i < XoptimalRoad.size(); i++) {
            const TPoint3D  plan_pt(XoptimalRoad[i], YoptimalRoad[i], 0.01);

            if (gl_planificator_path->empty())
                gl_planificator_path->appendLine(plan_pt,plan_pt);
            else gl_planificator_path->appendLineStrip(plan_pt);

            if (i > 0){
                optimalDistance += sqrt(pow(XoptimalRoad[i]-XoptimalRoad[i-1], 2) + pow(YoptimalRoad[i] - YoptimalRoad[i-1], 2));
            }
        }

        cout<<"*********************** OPTIMAL DISTANCE ********************"<<endl;
        cout<<"optimalDistance = "<<optimalDistance<<endl;
    }   
    //*******************************************************************

    //***************** SIGUIENDO LOS PUNTOS DE SUBGOAL *******************

    static double Xsol, Ysol; // Coordenadas pertenecientes al subgoal actual

    if (debuggDemoMain) cout<<"***************** FOLLOWING THE PLANIFICATION POINTS *******************"<<endl; // DEBUGG

    if (debuggMapFail) cout<<"Line 954"<<endl;

    if (planificatorActive) {

        //Se comprueba si el robot puede avanzar hacia el siguiente subgoal
        nearSubGoal = planifObj.goToNextGoal(m_robotPose.x, m_robotPose.y, Xsol, Ysol, XoptimalRoad, YoptimalRoad);
        
        // Se volvera a calcular la dirección que debe seguir el robot en el caso de que
        // se ejecute el algoritmo por primera vez, se llegue al subgoal actual o 
        // se indique que debe haber una nueva replanificacion
        if (nearSubGoal || firstSolution || replanification) {
            if (debuggPlanificator) cout<<"Cambio de direccion por nearSubGoal"<<endl;
            if (debuggLines) cout<<"Cambio de direccion!!!!"<<endl;
            
            changeDirection = true;
            if (nearSubGoal&&!firstSolution) {
                cout<<"NEARSUBGOALACTIVE BEFORE = "<<nearSubGoalActive<<endl;
                nearSubGoalActive = true;
                cout<<"NEARSUBGOALACTIVE TO TRUE"<<endl;
            } else {
                cout<<"NEARSUBGOALACTIVE SAME VALUE = "<<nearSubGoalActive<<endl;
            }

            firstSolution = false;
        } else {
            if (debuggPlanificator) cout<<"No estamos en targetSeen"<<endl;
        }

        if (debuggDemoMain) cout<<"Cycles reiniciados a "<<cycles<<endl; // DEBUGG
    }

    // Si el planificator esta activo, se calcularan las coordenadas relativas respecto al
    // robot delsubgoal actual, en caso contrario se conseguiran las coordenadas relativas 
    // respecto al robot del punto de destino
    if (planificatorActive) {
        cout<<"Planificador activo"<<endl;
        realRelTargetPose.x = Xsol-m_robotPose.x;
        realRelTargetPose.y = Ysol-m_robotPose.y;
    } else {
        cout<<"Planificador desactivado"<<endl;
        realRelTargetPose.x = m_targetPoint.x - m_robotPose.x;
        realRelTargetPose.y = m_targetPoint.y - m_robotPose.y;
    }

    if (rbHoloMethod->GetSelection() == 0) {
        relTargetPose = realRelTargetPose;
    }

    finalRelTargetPose.x = m_targetPoint.x - m_robotPose.x;
    finalRelTargetPose.y = m_targetPoint.y - m_robotPose.y;
    
    if (debuggDemoMain) {
        cout<<"Solution: ("<<Xsol<<" ,"<<Ysol<<")"<<endl; // DEBUGG
        cout<<"realRelTargetPose sin obstaculos: ("<<realRelTargetPose.x<<" ,"<<realRelTargetPose.y<<")"<<endl; // DEBUGG
        cout<<"relTargetPose: ("<<relTargetPose.x<<" ,"<<relTargetPose.y<<")"<<endl; // DEBUGG
        cout<<"Ciclos: "<<cycles<<endl; // DEBUGG
    }

    if (debuggDemoMain) {
        cout<<"Valor: "<<sqrt(pow(finalRelTargetPose.x - realRelTargetPose.x, 2) + pow(finalRelTargetPose.y - realRelTargetPose.y, 2))<<endl; // DEBUGG
        cout<<"Valor realRelTargetPose dist: "<<sqrt(pow(realRelTargetPose.x, 2) + pow(realRelTargetPose.y, 2))<<endl;
        cout<<"¡¡¡¡¡¡Estamos cerca del TARGET!!!!!!"<<endl; // DEBUGG
        cout<<"¡¡¡¡¡¡LEJOS DE TARGET!!!!!"<<endl; // DEBUGG
    }

    //************* LLEGANDO A OBJETIVOS INTERMEDIOS *************

    if (debuggStepByStep) {
        cout<<"Ciclo a: "<<cycles<<endl;
        cout<<"realRelTargetPose: "<<realRelTargetPose<<endl;
        cout<<"finalRelTargetPose: "<<finalRelTargetPose<<endl;
    }

    //**********************************************************************

    if (debuggLines) cout<<"Line 1149"<<endl;

	this->m_holonomicMethod->navigate(
		relTargetPose,                    // La posicion relativa del target en x,y [IN]
		simulatedScan.scan,               // Los puntos obtenidos por los obstaculos [IN]
		m_simul_options.ROBOT_MAX_SPEED,  // Velocidad maxima del robot [IN]
		desiredDirection,                 // Direccion deseada en angulo [-PI,PI] [OUT]
		desiredSpeed,                     // Velocidad en pseumetros/segundos --> pseumoetros²=metros² + (rad * r)² [OUT]
		out_log );                        // Sitio donde se guarda la informacion [IN/OUT]

    // Posicion del ciclo anterior del robot
    m_previousRobotPose = m_robotPose;
    // Movimiento del robot
	m_robotPose.x += cos(desiredDirection) * desiredSpeed * time_step;
	m_robotPose.y += sin(desiredDirection) * desiredSpeed * time_step;

    if (debuggLines) cout<<"Line 1167"<<endl;

    // Distancia recorrida por el robot
    distanceTraveled += sqrt(pow(m_robotPose.x - m_previousRobotPose.x, 2) + pow(m_robotPose.y - m_previousRobotPose.y, 2));

    // Tiempo transcurrido
    timeElapsed += timeSimul/1000.0;

    // Calculo de la minima distancia del robot a un obstaculo
    for (int i = 0; i < simulatedScan.scan.size(); i++) {
        if (simulatedScan.scan[i]*simulatedScan.maxRange < minimumDistanceToObstacle)  {
            minimumDistanceToObstacle = simulatedScan.scan[i]*simulatedScan.maxRange;
        }
    }

    if (debuggLines) cout<<"Line 1179"<<endl;

    // Impresion de datos necesarios para la realizacion de la memoria
    if (printDataInFile) {
        if (sqrt(pow(finalRelTargetPose.x, 2) + pow(finalRelTargetPose.y, 2)) > 0.01) {
            cout<<"m_robotPose.x = "<<m_robotPose.x<<" m_robotPose.y = "<<m_robotPose.y<<endl;
            cout<<"m_previousRobotPose.x = "<<m_previousRobotPose.x<<" m_previousRobotPose.y = "<<m_previousRobotPose.y<<endl;
            cout<<"distanceTraveled = "<<distanceTraveled<<endl;
            data<<"*************cycles: "<<mainCycles<<"****************"<<endl;
            data<<"m_robotPose.x = "<<m_robotPose.x<<" m_robotPose.y = "<<m_robotPose.y<<endl;
            data<<"m_previousRobotPose.x = "<<m_previousRobotPose.x<<" m_previousRobotPose.y = "<<m_previousRobotPose.y<<endl;
            data<<"distanceTraveled = "<<distanceTraveled<<endl;

            cout<<"Remaining distance: "<<sqrt(pow(finalRelTargetPose.x, 2) + pow(finalRelTargetPose.y, 2))<<endl;
            data<<"Remaining distance: "<<sqrt(pow(finalRelTargetPose.x, 2) + pow(finalRelTargetPose.y, 2))<<endl;

            cout<<"Tiempo transcurrido: "<<timeElapsed<<endl;
            data<<"Tiempo transcurrido: "<<timeElapsed<<endl;

            cout<<"Minima distancia a obstaculo: "<<minimumDistanceToObstacle<<endl;
            data<<"Minima distancia a obstaculo: "<<minimumDistanceToObstacle<<endl;
        }
    }

    //***************************************************

	// Update path graph:
	const TPoint3D  cur_pt(m_robotPose.x,m_robotPose.y,0.01);

    if (debuggLines) cout<<"Line 1208"<<endl;

	if (gl_robot_path->empty())
	     gl_robot_path->appendLine(cur_pt,cur_pt);
	else gl_robot_path->appendLineStrip(cur_pt);

    if (debuggLines) cout<<"Line 1214"<<endl;

    // Posicionamiento del punto del destino
	gl_rel_target->setLocation(realRelTargetPose.x, realRelTargetPose.y,0);

    //********* gl_subgoal **************
    
    // Dibujo del punto donde se encuentra el subgoal
    gl_subgoal->setLocation(Xsol, Ysol, 0);

	// Clear stuff which will be updated if used below:
	edInfoLocalView->Clear();
	gl_nd_gaps->clear();

    if (debuggLines) cout<<"Line 1222"<<endl;

	// Update 2D view graphs:
	if (out_log && IS_CLASS(out_log, CLogFileRecord_ND))
	{
		CLogFileRecord_NDPtr log = CLogFileRecord_NDPtr(out_log);
		const size_t nGaps = log->gaps_ini.size();
 		const string sSitu = mrpt::utils::TEnumType<CHolonomicND::TSituations>::value2name(log->situation);

        string sLog = mrpt::format("ND situation : %s \n",sSitu.c_str());

        if (debuggLines) cout<<"Line 1234"<<endl;

        // Se pasará al algoritmo ND en los siguientes casos:
        //- Se haya seleccionado expresamente
        //- El objetivo actual haya sido avistado
        //- El objetivo final haya sido avistado
        //- Se produzca una replanificacion
        //- Estemos cerca de un subgoal
        if (rbHoloMethod->GetSelection() == 1||targetSeen||finalTargetSeen||replanification||nearSubGoal)
        {
            if (startDebugg) {
                btnStart->Enable(true); btnStart->Refresh();
                btnStop->Enable(false);   btnStop->Refresh();
                edHoloParams->Enable(true);
                rbHoloMethod->Enable(true);
            }

            if (debuggLines) cout<<"Line 1253"<<endl;
            //*********************************************

            if (debuggPlanificator) cout<<"Entrado en MOTION TO GOAL: "<<endl<<"GetSelection = "<<rbHoloMethod->GetSelection()<<endl;
            if (debuggPlanificator) cout<<"--------targetSeen: "<<targetSeen<<"  finalTargetSeen: "<<finalTargetSeen<<" replanification: "<<replanification<<" nearSubGoal: "<<nearSubGoal<<endl;

            //*********** Comprobamos si el target esta en una region ocupada **************************
            double alfa=atan2(realRelTargetPose.y,realRelTargetPose.x);
            double alfa_deg=((alfa+M_PI)*simulatedScan.scan.size())/M_2PI; // Calculamos el angulo en el que se encuentra el target 
                                                                           // respecto al robot y lo pasamos a grados
            double division_reg=180.0/numOfRegions;
            int regTarget;

            if ((alfa_deg<division_reg/2)||(alfa_deg>=(180-division_reg/2))) {
                regTarget=0;
            }else {
                regTarget=(int)trunc((alfa_deg/division_reg)+0.5); // Añadimos 0.5 debido al desfase de ángulos que hay
            }

            if (debuggStepByStep) cout<<"El angulo en el que se encuentra el target: "<<alfa_deg<<" regTarget"<<regTarget<<endl;

            // Se comprueba si alguno de los puntos pertenecientes a los obstaculos detectados
            // se encuentra dentro del rango en el que se encuentra el destino actual. En este
            // caso, se cambiara de nuevo al algoritmo iND, rodeando un nuevo obstaculo
            for (size_t i = 0; i < simulatedScan.scan.size(); i++) {
                if (simulatedScan.scan[i]<0.98) {
                    double currentObstaclePointAngle = i*180/181.0;
                    
                    if (regTarget == 0 && (currentObstaclePointAngle < division_reg/8.0 || currentObstaclePointAngle >= (180-division_reg/8.0))) {
                        cout<<"YA NO MTG "<<endl;
                        cout<<"//////////nearSubGoal = "<<nearSubGoal<<" changeDirection = "<<changeDirection<<"replanification = "<<replanification<<"///////////"<<endl;
                        changeObjective = true;
                        replanification = false;
                        
                        if (planificatorActive && changeDirection) {
                            cout<<"nearSubGoal = "<<nearSubGoal<<" changeDirection = "<<changeDirection<<endl;
                            changeDirection = false;
                            m_EscapeGap->setDirection(chooseDirection(m_robotPose, Xsol, Ysol, m_previousRobotPose, !replanification && nearSubGoalActive));
                            nearSubGoalActive = false;
                        }
                        else if (!planificatorActive) m_EscapeGap->setDirection(chooseDirection(m_robotPose, m_targetPoint.x, m_targetPoint.y, m_previousRobotPose, true));
                        //***************************
                        if (debuggMotionToGoal) cout<<"currentObstaclePointAngle en region 0: "<<currentObstaclePointAngle<<endl;
                    } else {
                        double targetRegionUpperAngle = regTarget*division_reg + division_reg/8.0 + (regTarget*180/simulatedScan.scan.size()/(numOfRegions));
                        double targetRegionLowerAngle = regTarget*division_reg - division_reg/8.0 + (regTarget*180/simulatedScan.scan.size()/(numOfRegions));
                        double distanceToTarget = sqrt(pow(realRelTargetPose.x, 2) + pow(realRelTargetPose.y, 2));
                         
                        if (currentObstaclePointAngle < targetRegionUpperAngle && currentObstaclePointAngle >= targetRegionLowerAngle && distanceToTarget > Sensor_Radius){
                            cout<<"YA NO MTG POR DISTANCIA -> desiredDirection = "<<desiredDirection<<endl;
                            cout<<"currentObstaclePointAngle = "<<currentObstaclePointAngle<<"targetRegionLowerAngle = "<<targetRegionLowerAngle<<"targetRegionUpperAngle = "<<targetRegionUpperAngle<<endl;
                            cout<<"//////////nearSubGoal = "<<nearSubGoal<<" changeDirection = "<<changeDirection<<" replanification = "<<replanification<<" nearSubGoalActive = "<<nearSubGoalActive<<"///////////"<<endl;
                            changeObjective = true;
                            replanification = false;
                            
                           if (debuggPlanificator) cout<<"robot pose = ("<<m_robotPose.x<<", "<<m_robotPose.y<<") subtarget = ("<<Xsol<<", "<<Ysol<<")"<<endl;
                            if (planificatorActive && changeDirection) {
                                cout<<"nearSubGoal = "<<nearSubGoal<<" changeDirection = "<<changeDirection<<endl;
                                changeDirection = false;
                                m_EscapeGap->setDirection(chooseDirection(m_robotPose, Xsol, Ysol, m_previousRobotPose, !replanification && nearSubGoalActive));
                                nearSubGoalActive = false;
                            }
                           else if (!planificatorActive) m_EscapeGap->setDirection(chooseDirection(m_robotPose, m_targetPoint.x, m_targetPoint.y, m_previousRobotPose, true));
                            //***********************
                            if (debuggMotionToGoal) cout<<"currentObstaclePointAngle: "<<(double)currentObstaclePointAngle<<" targetRegionLowerAngle: "<<targetRegionLowerAngle<<" targetRegionUpperAngle: "<<targetRegionUpperAngle<<endl;
                        } 
                    }
                }
            }

            // Resetear deteccion de obstaculos

            if (debuggDemoFunctions) printf("Dentro de navegador simple\n"); // DEBUGG

            if (restartSimulation) {
                m_EscapeGap->reset();

                playPlanification = true; // Activamos de nuevo el planificador

                restartSimulation=false;
            }
            
            relTargetPose=realRelTargetPose;
            gl_nd_gaps->appendLine(0,0,0, 0,0,0);

            for (size_t i=0;i<nGaps;i++)
            {
                const size_t N_STEPS = 20;
                for (size_t j=0;j<N_STEPS;j++)
                {
                     const double sec = log->gaps_ini[i] + j*(log->gaps_end[i]-log->gaps_ini[i])/static_cast<double>(N_STEPS-1);
                     const double ang = M_PI *( -1 + 2*sec/((float)simulatedScan.scan.size()) );
                     const double d = simulatedScan.scan[sec]-0.05;
                     gl_nd_gaps->appendLineStrip(d*cos(ang),d*sin(ang),0);
                }
                gl_nd_gaps->appendLineStrip(0,0,0);
            }
        }
        else if (obstacleCounter==0) {
            if (debuggMapFail) cout<<"Line 1214"<<endl;
            if (debuggDemoFunctions) printf("****No hay obstaculos****\n"); // DEBUGG

            if (targetSeen) {
                relTargetPose=realRelTargetPose;
            }
        }
        else {
            if (debuggMapFail) cout<<"Line 1223"<<endl;
            if (startDebugg) m_EscapeGap->writeDebugg = true;

            restartSimulation=true;

            if (debuggLines) cout<<"Line 1392"<<endl;

            // Se ejecuta el algoritmo iND
            sLog+=m_EscapeGap->update(realRelTargetPose,
                                      simulatedScan.scan,m_robotPose,simulatedScan.maxRange,&relTargetPose,&targetSeen,
                                      log->gaps_ini,log->gaps_end, regionStatePlan, desiredDirection, mainCycles, timeElapsed);

            //********************* Para documentacion **********

            cout<<"************Puntos del target*****************"<<endl;
            cout<<"m_targetPoint.x = "<<m_targetPoint.x<<" m_targetPoint.y = "<<m_targetPoint.y<<endl;
                
            if (debuggStepByStep) cout<<"Valor targetSeen: "<<targetSeen<<endl;

            if (debuggLines) cout<<"Line 1404"<<endl;

            cout<<"Valor targetSeen = "<<targetSeen<<endl; // Debugg hibrido

            static int debuggCycle = 0;
            if (startDebugg|| targetSeen) {
                btnStart->Enable(true); btnStart->Refresh();
                btnStop->Enable(false);   btnStop->Refresh();
                edHoloParams->Enable(true);
                rbHoloMethod->Enable(true);
            }

            debuggCycle++;
            if (debuggStepByStep) cout<<"Obstaculos angulos: "<<endl;
            if (debuggStepByStep) {
                for (unsigned int i = 0; i < simulatedScan.scan.size(); i++) {
                    if (i%20==0) cout<<endl;
                    if ((double)simulatedScan.scan[i]<0.98) {
                        printf("%u:%.2f\t",i,(double)(i*180/181.0));
                    }
                }
            }
            if (debuggStepByStep) cout<<endl<<"Ciclo de debugg: "<<debuggCycle<<endl;
            
            if (!debuggDemoMain && debuggStepByStep) {
                for (size_t i = 0; i<20; i++) {
                    cout<<"regionStatePlan["<<i<<"] = "<<regionStatePlan[i]<<endl;   
                }
            }

            if (targetSeen) {
                Sensor_Radius=1.5;
            } else {
                Sensor_Radius=1.5;
            }

        }
        sLog+=mrpt::format("obstacleCounter: %f\n", (double)obstacleCounter);
        sLog+=mrpt::format("sensor radius: %f\n", (double)Sensor_Radius);
        sLog+=mrpt::format("restartSimulation: %d",restartSimulation);
        sLog+=mrpt::format("paramsSize %d", params.ang.size());

        edInfoLocalView->SetValue(_U(sLog.c_str()));
	}

	// Movement direction:
	const double d = desiredSpeed/m_simul_options.ROBOT_MAX_SPEED;
	gl_line_direction->setLineCoords(0,0,0,cos(desiredDirection) * d, sin(desiredDirection) * d, 0 );
}

void holonomic_navigator_demoFrame::updateViewsDynamicObjects()
{
	gl_robot->setLocation( m_robotPose.x, m_robotPose.y, 0);

	// animate target:
	{
		const double TARGET_BOUNCE_MIN = 0.7;
		const double TARGET_BOUNCE_MAX = 1;

		const double TARGET_BOUNCE_PERIOD = 1.0;
		const double t = fmod( m_runtime.Tac(), TARGET_BOUNCE_PERIOD ) / TARGET_BOUNCE_PERIOD;

		// Parabolic path
		const double s = 4*t*(TARGET_BOUNCE_MAX - TARGET_BOUNCE_MIN)*(1-t) + TARGET_BOUNCE_MIN;

		gl_target->setLocation( m_targetPoint.x, m_targetPoint.y, 0);
		gl_target->setScale(s);
	}

	// Labels:
	StatusBar1->SetStatusText( _U( mrpt::format("Robot: (%.03f,%.03f)", m_robotPose.x, m_robotPose.y ).c_str() ), 0 );
	StatusBar1->SetStatusText( _U( mrpt::format("Target: (%.03f,%.03f)", m_targetPoint.x, m_targetPoint.y ).c_str()  ), 1 );

	// Show/hide:
	if (!draw) gl_robot_sensor_range->setVisibility( mnuViewMaxRange->IsChecked() );
	gl_robot_path->setVisibility( mnuViewRobotPath->IsChecked() );

	// Refresh:
	m_plot3D->Refresh();
	m_plotScan->Refresh();
}

void holonomic_navigator_demoFrame::Onplot3DMouseMove(wxMouseEvent& event)
{
	int X, Y;
	event.GetPosition(&X,&Y);

	// Intersection of 3D ray with ground plane ====================
	TLine3D ray;
	m_plot3D->m_openGLScene->getViewport("main")->get3DRayForPixelCoord( X,Y,ray);
	// Create a 3D plane, e.g. Z=0
	const TPlane ground_plane(TPoint3D(0,0,0),TPoint3D(1,0,0),TPoint3D(0,1,0));
	// Intersection of the line with the plane:
	TObject3D inters;
	intersect(ray,ground_plane, inters);
	// Interpret the intersection as a point, if there is an intersection:
	TPoint3D inters_pt;
	if (inters.getPoint(inters_pt))
	{
		m_curCursorPos.x = inters_pt.x;
		m_curCursorPos.y = inters_pt.y;

		if (m_cursorPickState==cpsPickTarget)
		{
			m_gl_placing_nav_target->setVisibility(true);
			m_gl_placing_nav_target->setLocation(m_curCursorPos.x,m_curCursorPos.y,0.05);
		}
		else
		if (m_cursorPickState==cpsPlaceRobot)
		{
			m_gl_placing_robot->setVisibility(true);
			m_gl_placing_robot->setLocation(m_curCursorPos.x,m_curCursorPos.y,0.05);
		}

		StatusBar1->SetStatusText(wxString::Format(wxT("X=%.03f Y=%.04f Z=0"),m_curCursorPos.x,m_curCursorPos.y), 2);
	}

	// Do normal process in that class:
	m_plot3D->OnMouseMove(event);
}

void holonomic_navigator_demoFrame::Onplot3DMouseClick(wxMouseEvent& event)
{
	switch (m_cursorPickState)
	{
	case cpsPickTarget:
		{
			m_targetPoint = m_curCursorPos;

			btnPlaceTarget->SetValue(false);
			btnPlaceTarget->Refresh();
			m_gl_placing_nav_target->setVisibility(false);
			break;
		}
	case cpsPlaceRobot:
		{
			m_robotPose.x = m_curCursorPos.x;
			m_robotPose.y = m_curCursorPos.y;

			btnPlaceRobot->SetValue(false);
			btnPlaceRobot->Refresh();
			m_gl_placing_robot->setVisibility(false);
			break;
		}
	default:
		break;
	}

	m_plot3D->SetCursor( *wxSTANDARD_CURSOR ); // End of cross cursor
	m_cursorPickState = cpsNone; // End of mode

	// Do normal process in that class:
	m_plot3D->OnMouseDown(event);
}

// ==== holonomic_navigator_demoFrame::TOptions ======
holonomic_navigator_demoFrame::TOptions::TOptions() :
	ROBOT_MAX_SPEED   ( 4.0 ),
	MAX_SENSOR_RADIUS ( Sensor_Radius ),
	SENSOR_NUM_RANGES ( 181),
	SENSOR_RANGE_NOISE_STD (0.02)
{
}

void holonomic_navigator_demoFrame::TOptions::loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section)
{
	MRPT_START

	// Load from config text:
	MRPT_LOAD_CONFIG_VAR(ROBOT_MAX_SPEED,double,  source,section );
	MRPT_LOAD_CONFIG_VAR(MAX_SENSOR_RADIUS,double,  source,section );
	MRPT_LOAD_CONFIG_VAR(SENSOR_NUM_RANGES, uint64_t,  source,section );
	MRPT_LOAD_CONFIG_VAR(SENSOR_RANGE_NOISE_STD,double,  source,section );

	MRPT_END
}

void holonomic_navigator_demoFrame::TOptions::saveToConfigFile(mrpt::utils::CConfigFileBase &cfg,const std::string &section) const
{
	MRPT_START
	const int WN = 40, WV = 20;

	cfg.write(section,"ROBOT_MAX_SPEED",ROBOT_MAX_SPEED,   WN,WV, "Maximum speed for the robot (m/s)");
	cfg.write(section,"MAX_SENSOR_RADIUS",MAX_SENSOR_RADIUS,   WN,WV, "Maximum range of the 360deg sensor (meters)");
	cfg.write(section,"SENSOR_NUM_RANGES",SENSOR_NUM_RANGES,   WN,WV, "Number of ranges in the 360deg sensor FOV");
	cfg.write(section,"SENSOR_RANGE_NOISE_STD",SENSOR_RANGE_NOISE_STD,   WN,WV, "Sensor noise (one sigma, in meters)");

	MRPT_END
}

void holonomic_navigator_demoFrame::OnMenuItemChangeVisibleStuff(wxCommandEvent& event)
{
	updateViewsDynamicObjects();
}

void holonomic_navigator_demoFrame::OnMenuItemClearRobotPath(wxCommandEvent& event)
{
	gl_robot_path->clear();
	updateViewsDynamicObjects();
}

void holonomic_navigator_demoFrame::OnbtnLoadMapClick(wxCommandEvent& event)
{
	WX_START_TRY

	wxFileDialog dlg(
		this,
		_("Select grid map to load"),
		_("."),
		_("grid.png"),
		wxT("Image files (*.png,*.jpg,*.gif)|*.png;*.jpg;*.gif|Binary gridmap files (*.gridmap,*.gridmap.gz)|*.gridmap;*.gridmap.gz|All files (*.*)|*.*"),
		wxFD_OPEN | wxFD_FILE_MUST_EXIST );

	if (dlg.ShowModal() != wxID_OK)
		return;

	const wxString sFil =  dlg.GetPath();
	const std::string fil = std::string(sFil.mb_str());

	const std::string fil_ext = mrpt::system::extractFileExtension(fil,true);

	if (mrpt::system::lowerCase(fil_ext)=="gridmap")
	{
		CFileGZInputStream f(fil);
		f >> m_gridMap;
	}
	else
	{
		// Try loading the image:
		CImage img;
		if (!img.loadFromFile(fil, 0 /* force grayscale */ ))
		{
			wxMessageBox(_("Error"),_("Can't load the image file (check its format)."));
		}
		else
		{
			// We also need the size of each pixel:
			double cx =-1;
			double cy =-1;
			double cell_size = 0.05;

			const wxString sCellSize = wxGetTextFromUser(_("Enter the size (in meters) of each pixel:"),_("Grid parameters"),_("0.05"), this);
			const wxString sCX = wxGetTextFromUser(_("Enter the central pixel (x-coordinate), or -1 = the image center:"),_("Grid parameters"),_("-1"), this);
			const wxString sCY = wxGetTextFromUser(_("Enter the central pixel (y-coordinate), or -1 = the image center:"),_("Grid parameters"),_("-1"), this);

			if (sCellSize.ToDouble(&cell_size) && sCX.ToDouble(&cx) && sCY.ToDouble(&cy) )
			{
				if (!m_gridMap.loadFromBitmap(img,cell_size,cx,cy))
					wxMessageBox(_("Error"),_("Can't load the image file into the gridmap..."));
			}
			else
				wxMessageBox(_("Error"),_("Error parsing the numbers you entered..."));
		}
	}

	updateMap3DView();
	m_plot3D->Refresh();

	WX_END_TRY
}

int holonomic_navigator_demoFrame::chooseDirection(const mrpt::math::TPose2D &rPose, double XsubTarget, double YsubTarget, const mrpt::math::TPose2D &previousRPose, bool forNearSubgoal) {

    cout<<"-------------------------------------------------------------------"<<endl;
    cout<<"****++++++++********++++chooseDirection_three****++++++++********++++"<<endl;
    cout<<"-------------------------------------------------------------------"<<endl;
    double X = XsubTarget - rPose.x;
    double Y = YsubTarget - rPose.y;
    double angle = atan2(Y, X);
    double XrelR = rPose.x - previousRPose.x;
    double YrelR = rPose.y - previousRPose.y;
    double desiredAngle = atan2(YrelR, XrelR);

    cout<<"XsubTarget = "<<XsubTarget<<" YsubTarget = "<<YsubTarget<<endl;
    cout<<"rPose.x = "<<rPose.x<<" rPose.y = "<<rPose.y<<endl;
    cout<<"previousRPose.x = "<<previousRPose.x<<" previousRPose.y = "<<previousRPose.y<<endl;
    cout<<"X = "<<X<<" Y = "<<Y<<" angle = "<<angle<<endl;
    cout<<"XrelR = "<<XrelR<<" YrelR = "<<YrelR<<" desiredAngle = "<<desiredAngle<<endl;
    cout<<"Diff Angle = "<<desiredAngle - angle<<endl;

    if (forNearSubgoal) {
        cout<<"Por subgoal cercano"<<endl;
        if (desiredAngle - angle <= 0.0000) {
            cout<<"Angulo menor que 0, entonces sentido horario"<<endl;
            return Izquierda;
        } else {
            cout<<"Angulo mayor que 0, entonces sentido antihorario"<<endl;
            return Derecha;
        }
    } else {
        cout<<"Por replanification"<<endl;
        if (desiredAngle - angle<= 0.0000) {
            cout<<"Angulo menor que 0, entonces sentido horario"<<endl;
            return Izquierda;
        } else {
            cout<<"Angulo mayor que 0, entonces sentido antihorario"<<endl;
            return Derecha;
        }
    }    
}

void holonomic_navigator_demoFrame::timeval_diff(struct timeval *a, struct timeval *b)
{
    double secs = 
    (double)(b->tv_sec + (double)b->tv_usec/1000000) -
    (double)(a->tv_sec + (double)a->tv_usec/1000000);
    cout<<"*******Tiempo********"<<endl;
    printf("%.16g milliseconds\n", secs * 1000.0);
}