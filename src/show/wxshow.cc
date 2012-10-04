/*
 * wxshow implementation
 *
 * Copyright (C) Jan Elseberg
 *
 * Released under the GPL version 3.
 *
 */

#include "show_common.cc"
#include "wx/wx.h"
#include "wx/sizer.h"
#include "wx/glcanvas.h"
#include "show/wxshow.h"
#include "show/selectionframe.h"


class SelectionImpl : public Selection {
  public:

		SelectionImpl( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Selection"), bool advanced_controls = false, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( -1,-1 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL ) : Selection(parent, id, title, pos, size, style, advanced_controls) {
  
      if (pointtype.hasReflectance()) m_choice11->Append(wxT("reflectance"));
      if (pointtype.hasTemperature()) m_choice11->Append(wxT("temperature"));
      if (pointtype.hasAmplitude()) m_choice11->Append(wxT("amplitude"));
      if (pointtype.hasDeviation()) m_choice11->Append(wxT("deviation"));
      if (pointtype.hasType()) m_choice11->Append(wxT("type"));

    };
		// Virtual event handlers, overide them in your derived class
    //
		virtual void OnDrawPoints( wxCommandEvent& event ) { 
      if (event.IsChecked()) {
        show_points = true;
      } else {
        show_points = false;
      }
      haveToUpdate = 1;
      event.Skip(); 
    }
		virtual void OnDrawCameras( wxCommandEvent& event ) 
    {
      if (event.IsChecked()) {
        show_cameras = true;
      } else {
        show_cameras = false;
      }
      haveToUpdate = 1;
      event.Skip(); 
    }
		virtual void OnDrawPaths( wxCommandEvent& event )
    {
      if (event.IsChecked()) {
        show_path = true;
      } else {
        show_path = false;
      }
      haveToUpdate = 1;
      event.Skip(); 
    }
		virtual void OnPointSize( wxSpinEvent& event ) 
    { 
      pointsize = event.GetPosition();
      haveToUpdate = 1;
      event.Skip(); 
    }
		virtual void OnFogChoice( wxCommandEvent& event ) 
    { 
      show_fog = event.GetSelection();
      haveToUpdate = 1;
      event.Skip(); 
    }
		virtual void OnFogDensity( wxSpinEvent& event ) { 
      wxSpinCtrlDbl *spinner = (wxSpinCtrlDbl*)event.GetEventObject();
      fogDensity = spinner->GetValue();
      haveToUpdate = 1;

      event.Skip(); 
    }
		virtual void OnColorValue( wxCommandEvent& event )
    {
      // we can't use the glui-show function for changing the color as a listbox is used here instead
      switch (event.GetSelection()) {
        case 0:
          cm->setCurrentType(PointType::USE_HEIGHT);
          break;
        case 1:
          if (pointtype.hasReflectance()) {
            cm->setCurrentType(PointType::USE_REFLECTANCE);
            break;
          }
        case 2:
          if (pointtype.hasTemperature()) {
            cm->setCurrentType(PointType::USE_TEMPERATURE);
            break;
          }
        case 3:
          if (pointtype.hasAmplitude()) {
            cm->setCurrentType(PointType::USE_AMPLITUDE);
            break;
          }
        case 4:
          if (pointtype.hasDeviation()) {
            cm->setCurrentType(PointType::USE_DEVIATION);
            break;
          }
        case 5:
          if (pointtype.hasType()) {
            cm->setCurrentType(PointType::USE_TYPE);
            break;
          }
        default:
          break;
      };
      haveToUpdate = 1;
      resetMinMax(0);
      event.Skip(); 
    }
		virtual void OnColorMap( wxCommandEvent& event ) 
    { 
      listboxColorMapVal = event.GetSelection();
      changeColorMap(0);
      haveToUpdate = 1;
      event.Skip(); 
    }
		virtual void OnColorType( wxCommandEvent& event ) { 
      colorScanVal = event.GetSelection();
      setScansColored(0);
      haveToUpdate = 1;
      event.Skip(); 
    }
		virtual void OnColorMinVal( wxSpinEvent& event ) { 
      //mincolor_value = event.GetPosition();
      wxSpinCtrlDbl *spinner = (wxSpinCtrlDbl*)event.GetEventObject();
      mincolor_value = spinner->GetValue();
      minmaxChanged(0);
      haveToUpdate = 1;
      event.Skip(); 
    }
		virtual void OnColorMaxVal( wxSpinEvent& event ) { 
      //maxcolor_value = event.GetPosition();
      wxSpinCtrlDbl *spinner = (wxSpinCtrlDbl*)event.GetEventObject();
      maxcolor_value = spinner->GetValue();
      minmaxChanged(0);
      haveToUpdate = 1;
      event.Skip(); 
    }
		virtual void OnColorResetMinMax( wxCommandEvent& event ) { 
      resetMinMax(0);
      haveToUpdate = 1;
      event.Skip(); 
    }
		virtual void OnInvert( wxCommandEvent& event ) { 
      invertView(0);
      haveToUpdate = 1;
      event.Skip(); 
    }
		virtual void OnAnimDelay( wxSpinEvent& event ) { 
      anim_delay = event.GetPosition();
      event.Skip(); 
    }
		virtual void OnAnimate( wxCommandEvent& event ) { 
      startAnimation(0);
      event.Skip(); 
    }
		virtual void OnCameraFile( wxCommandEvent& event ) { 
      wxString s = event.GetString();
      const wxCharBuffer buffer = wxString(event.GetString()).mb_str(wxConvISO8859_1);
      const char* cc = buffer.data();
      strcpy(path_file_name, cc);
      event.Skip(); 
    }
		virtual void OnCameraSavePath( wxCommandEvent& event ) { 
      savePath(0);
      event.Skip(); 
    }
		virtual void OnCameraLoadPath( wxCommandEvent& event ) { 
      loadPath(0);
      event.Skip(); 
    }
    virtual void OnCameraLoadRobotPath( wxCommandEvent& event ) { 
      for(unsigned int i = 0; i < MetaMatrix.size(); i++){
        //temp variable
        double *temp;
        //Now, lets go to the last of each frame file to
        //extract the transformation matrix obtained
        //after scan matching has been done.
        glMultMatrixd(MetaMatrix[i].back());

        //temp is final transformation matrix
        temp = MetaMatrix[i].back();

        Point campos(temp[12], temp[13] + 100, temp[14]);

        // calculate lookat point
        Point lookat(0, 0, 50);
        Point up(0, 50, 0);
        double tmat[16];
        for (int i =0;i<16;i++) tmat[i] = temp[i]; 
        lookat.transform(tmat);
        lookat.x = lookat.x ; 
        lookat.y = lookat.y + 100; 
        lookat.z = lookat.z  ; 

        up.transform(tmat);
        up.x = up.x ; 
        up.y = up.y + 100; 
        up.z = up.z  ; 

        cams.push_back(campos);
        lookats.push_back(lookat);
        ups.push_back(up);
      }
      //signal for the update of scene
      haveToUpdate = 1;
      event.Skip(); 
    }
		virtual void OnSaveAnimation( wxCommandEvent& event ) { 
      if (event.IsChecked()) {
        save_animation = true;
      } else {
        save_animation = false;
      }
      haveToUpdate = 1;
      event.Skip(); 
    }
		virtual void OnAnimatePath( wxCommandEvent& event ) { 
      pathAnimate(0);
      event.Skip(); 
    }
		virtual void OnPositionFile( wxCommandEvent& event ) { 
      wxString s = event.GetString();
      const wxCharBuffer buffer = wxString(event.GetString()).mb_str(wxConvISO8859_1);
      const char* cc = buffer.data();
      strcpy(pose_file_name, cc);
      event.Skip(); 
    }
		virtual void OnPositionSave( wxCommandEvent& event ) { 
      savePose(0);
      event.Skip(); 
    }
		virtual void OnPositionLoad( wxCommandEvent& event ) { 
      loadPose(0);
      haveToUpdate = 1;
      event.Skip(); 
    }
		virtual void OnFactor( wxSpinEvent& event ) { 
      factor = event.GetPosition();
      event.Skip(); 
    }
		virtual void OnSaveImage( wxCommandEvent& event ) { 
      saveImage(0);
      event.Skip(); 
    }
		virtual void OnSelectionFile( wxCommandEvent& event ) {
      wxString s = event.GetString();
      const wxCharBuffer buffer = wxString(event.GetString()).mb_str(wxConvISO8859_1);
      const char* cc = buffer.data();
      strcpy(selection_file_name, cc);
      event.Skip(); 
    }
		virtual void OnSelectionSave( wxCommandEvent& event ) { 
      saveSelection(0);
      event.Skip(); 
    }
		virtual void OnSelectionClear( wxCommandEvent& event ) { 
      clearSelection(0);
      event.Skip(); 
    }
		virtual void OnSelectionSU( wxCommandEvent& event ) { 
      if (event.IsChecked()) {
        selectOrunselect = true;
      } else {
        selectOrunselect = false;
      }
      event.Skip(); 
    }
		virtual void OnSelectionSV( wxCommandEvent& event ) { 
      if (event.IsChecked()) {
        select_voxels = true;
      } else {
        select_voxels = false;
      }
      event.Skip(); 
    }
		virtual void OnSelectionDepth( wxSpinEvent& event ) { 
      selection_depth = event.GetPosition();
      event.Skip(); 
    }
		virtual void OnSelectionBrushsize( wxSpinEvent& event ) { 
      brush_size = event.GetPosition();
      event.Skip(); 
    }
		virtual void OnFrameSpinner( wxSpinEvent& event ) { 
      current_frame = event.GetPosition();
      haveToUpdate = 1;
      event.Skip(); 
    }
		virtual void OnFramerateSpinner( wxSpinEvent& event ) { 
      idealfps = event.GetPosition();
      haveToUpdate = 1;
      event.Skip(); 
    }
		virtual void OnFarplaneSpinner( wxSpinEvent& event ) { 
      maxfardistance = event.GetPosition();
      haveToUpdate = 1;
      event.Skip(); 
    }
		virtual void OnNearplaneSpinner( wxSpinEvent& event ) { 
      neardistance = event.GetPosition();
      haveToUpdate = 1;
      event.Skip(); 
    }
		virtual void OnCycleLOD( wxCommandEvent& event ) { 
      ::cycleLOD();
      haveToUpdate = 1;
      event.Skip(); 
    }
		
    virtual void OnLODAdaption( wxSpinEvent& event ) { 
      adaption_rate = ((wxSpinCtrlDbl*)event.GetEventObject())->GetValue();
      haveToUpdate = 1;
      event.Skip(); 
    }

  public:
    void updateControls() {
      if (!MetaMatrix.empty()) {
        frame_spin->SetRange(0, MetaMatrix[0].size()-1);
        frame_spin->SetValue(current_frame);
      }
      m_spinCtrl61->SetValue(mincolor_value);
      m_spinCtrl6->SetValue(maxcolor_value);
    }

};

class ControlImpl : public Controls {

  public:
  
		ControlImpl( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Controls"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( -1,-1 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL )
      : Controls( parent, id, title, pos, size, style ) {}
		
    virtual void OnApexAngle( wxSpinEvent& event ) { 
      haveToUpdate = 2;
      cangle = ((wxSpinCtrlDbl*)event.GetEventObject())->GetValue();
      event.Skip(); 
    }
		virtual void OnParallelZoom( wxSpinEvent& event ) { 
      haveToUpdate = 2;
      pzoom = ((wxSpinCtrlDbl*)event.GetEventObject())->GetValue();
      event.Skip(); 
    }
		virtual void OnTopView( wxCommandEvent& event ) {
      topView(); // TODO update controls
      haveToUpdate = 2;
      event.Skip(); 
    }
		virtual void OnResetPosition( wxCommandEvent& event ) { 
      resetView(0);
      event.Skip(); 
    }
		virtual void OnChooseCamera( wxSpinEvent& event ) {
      cam_choice = event.GetPosition();
      haveToUpdate = 1;
      event.Skip(); 
    }
		virtual void OnAddCamera( wxCommandEvent& event ) { 
      callAddCamera(1);
      haveToUpdate = 1;
      event.Skip(); 
    }
		virtual void OnDeleteCamera( wxCommandEvent& event ) { 
      callDeleteCamera(0);
      haveToUpdate = 1;
      event.Skip(); 
    }
		virtual void OnMouseNav( wxCommandEvent& event ) { 
      /*
      if (event.IsChecked()) {
        cameraNavMouseMode = true;
      } else {
        cameraNavMouseMode = false;
      }*/
      // TODO implement secondary navigation procedure
      event.Skip(); 
    }
		virtual void OnAlwaysAllPoints( wxCommandEvent& event ) {
      changePointMode(0);
      haveToUpdate = 1;
      event.Skip(); 
    }
		virtual void OnAlwaysReducePoints( wxCommandEvent& event ) { 
      changePointMode(1);
      haveToUpdate = 1;
      event.Skip(); 
    }
  
  public:
    void updateControls() {
      apex_spinner->SetValue(cangle);
      parallel_spinner->SetValue(pzoom);

      if(showTopView) {
        apex_spinner->Disable();
        parallel_spinner->Enable();
      } else {
        parallel_spinner->Disable();
        apex_spinner->Enable();
      }

      switch(pointmode) {
        case -1:
          always_box->SetValue(false);
          alwaysred_box->SetValue(true);
          break;
        case 0:
          always_box->SetValue(false);
          alwaysred_box->SetValue(false);
          break;
        case 1:
          always_box->SetValue(true);
          alwaysred_box->SetValue(false);
          break;
      }
  
      
      camera_spinner->SetRange(1, cams.size());
      camera_spinner->SetValue(cam_choice);
    }
};


class wxShow: public wxApp
{
    virtual bool OnInit();

    SelectionImpl *selection;
    ControlImpl *controls;

    wxFrame *frame;
    BasicGLPane * glPane;
public:
    void OnClose(wxCloseEvent& event);
    void updateControls(){
      controls->updateControls();
      selection->updateControls();
    }

};
    

static wxShow *globalGUI = 0;
    
void wxShow::OnClose(wxCloseEvent& event) {
  exit(0);
}
 
IMPLEMENT_APP(wxShow)
 
 
bool wxShow::OnInit()
{

  globalGUI = this;
  // wxwidgets saves the argv as wxchar, which is 2-byte unicode, so we must convert it back
  char **new_argv = new char*[argc];
  for (int i = 0; i < argc; i++) {
    const wxCharBuffer buffer = wxString(wxApp::argv[i]).mb_str(wxConvISO8859_1);
    const char* cc = buffer.data();
    new_argv[i] = new char[ strlen(cc) +1 ];
    strcpy(new_argv[i], cc);
  }
  initShow(argc, new_argv);
  //glClearColor(0.0, 0.0, 0.0, 0.0);

  wxBoxSizer* sizer = new wxBoxSizer(wxHORIZONTAL);
  frame = new wxFrame((wxFrame *)NULL, -1,  wxT("Viewer"), wxPoint(START_X, START_Y), wxSize(START_WIDTH, START_HEIGHT));

  int args[] = {WX_GL_RGBA, WX_GL_DOUBLEBUFFER, WX_GL_DEPTH_SIZE, 16, 0};

  glPane = new BasicGLPane( (wxFrame*) frame, args);
  sizer->Add(glPane, 1, wxEXPAND);

  frame->SetSizer(sizer);
  frame->SetAutoLayout(true);


  selection = new SelectionImpl( (wxWindow*)NULL, wxID_ANY, wxT("Selection"), advanced_controls, wxPoint(START_X + START_WIDTH + 50, START_Y + 30) );
  controls = new ControlImpl( (wxWindow*)NULL, wxID_ANY, wxT("Controls"), wxPoint(START_X, START_Y + START_HEIGHT + 20 ) );
 
  selection->SetSize(wxSize(200,393) );
  selection->SetAutoLayout(true);
  selection ->Show();
  selection ->Layout();
  
  controls->SetAutoLayout(true);
  controls ->Show();
	
  frame->Show();
  frame->Connect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( wxShow::OnClose ) );

  return true;
} 
 
BEGIN_EVENT_TABLE(BasicGLPane, wxGLCanvas)
EVT_IDLE(BasicGLPane::idle_event)
EVT_MOTION(BasicGLPane::mouseMoved)
EVT_MOUSE_EVENTS(BasicGLPane::mouseEvent)
  //EVT_LEFT_DOWN(BasicGLPane::mouseDown)
//EVT_LEFT_UP(BasicGLPane::mouseReleased)
//EVT_RIGHT_DOWN(BasicGLPane::rightClick)
//EVT_LEAVE_WINDOW(BasicGLPane::mouseLeftWindow)
EVT_SIZE(BasicGLPane::resized)
EVT_KEY_DOWN(BasicGLPane::keyPressed)
//EVT_KEY_UP(BasicGLPane::keyReleased)
//EVT_MOUSEWHEEL(BasicGLPane::mouseWheelMoved)
EVT_PAINT(BasicGLPane::render)
END_EVENT_TABLE()
 
 
// some useful events to use
void BasicGLPane::mouseEvent(wxMouseEvent& event) 
{
  SetFocus();
  int x = event.GetX();
  int y = event.GetY();
 
  if (event.Dragging()) {
  } else if (event.IsButton()) {
    int button, state;
    if (event.ButtonDown()) {
      state = GLUT_DOWN;
    } else {
      state = GLUT_UP;
    }

    if (event.Button(1)) {
      button = GLUT_LEFT_BUTTON;
    } else if (event.Button(2)) {
      button = GLUT_MIDDLE_BUTTON;
    } else if (event.Button(3)) {
      button = GLUT_RIGHT_BUTTON;
    } else {
      button = 0;
    }
    CallBackMouseFunc(button, state, x, y);
  }
}

void BasicGLPane::idle() {
  if(glutGetWindow() != window_id)
    glutSetWindow(window_id);
	 
  /*
  static unsigned long start = GetCurrentTimeInMilliSec();
  // return as nothing has to be updated
  if (haveToUpdate == 0) {
    if ((GetCurrentTimeInMilliSec()-start) > 100) {
      paint(true);
      start = GetCurrentTimeInMilliSec();
    }
    return;
  }
  start = GetCurrentTimeInMilliSec();
  */
  if (haveToUpdate == 0) {
    if (!fullydisplayed && !mousemoving && !keypressed) {
      glDrawBuffer(buffermode);
      //Call the display function
      DisplayItFunc(GL_RENDER, true);
    }
    return;
  }
  

  // case: display is invalid - update it
  if (haveToUpdate == 1) {
    paint();
    haveToUpdate = 0;
    return;
  }
  // case: display is invalid - update it with all points
/*if (haveToUpdate == 7) {
    showall = true;
    paint();
    haveToUpdate = 0;
    return;
  }
  */

  // case: camera angle is changed - instead of repeating code call Reshape,
  // since these OpenGL commands are the same
  if (haveToUpdate == 2) {
    int viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    CallBackReshapeFunc(viewport[2],viewport[3]);
    paint();
    haveToUpdate = 0;
    return;
  }
  
  // case: animation
  if(haveToUpdate == 3 ){
    frameNr += 1;
    if(!(MetaMatrix.size() > 1 &&  frameNr < (int) MetaMatrix[1].size())){
      frameNr = 0;
      haveToUpdate = 4;
      return;
    }
    paint();

    if(save_animation){
      string filename = scan_dir + "animframe" + to_string(frameNr,5) + ".ppm";
      cout << "write " << filename << endl;
      glDumpWindowPPM(filename.c_str(),0);

   }
    
  }
#ifdef _MSC_VER
  Sleep(300);
  Sleep(anim_delay);
#else
  usleep(anim_delay * 10000);
#endif

  if (haveToUpdate == 4) { // stop animation
    frameNr = 0;  // delete these lines if you want a 'continue' functionality.
    haveToUpdate = 1;
  }

  // case: path animation
  if(haveToUpdate == 6){

    if (path_iterator == 0) {
      oldcamNavMode = cameraNavMouseMode;  // remember state of old mousenav
      cameraNavMouseMode = 0;
    }



    //check if the user wants to animate both
    //scan matching and the path at the same
    //time

    //cout << "path_iterator: " << path_iterator << endl;
    if(path_iterator < path_vectorX.size()){   // standard animation case

      //call the path animation function
      //hide both the cameras and the path
      show_cameras = 0;
      show_path = 0;
      //increase the iteration count

      path_iterator += 1;
      //cout << "i am here" << endl;
      //repaint the screen
      paint();

      //save the animation
      if(save_animation){
        string filename = scan_dir + "animframe" + to_string(path_iterator,5) + ".ppm";
        string jpgname = scan_dir + "animframe" + to_string(path_iterator,5) + ".jpg";
        cout << "written " << filename << " of " << path_vectorX.size() << " files" << endl;
        glWriteImagePPM(filename.c_str(), factor, 0);
        haveToUpdate = 6;

      }
    }else{                             // animation has just ended
      cameraNavMouseMode = oldcamNavMode;
      show_cameras = 1;
      show_path = 1;
      //cout << "i am here instead" << endl;
      haveToUpdate = 0;
    }
  }
}

void BasicGLPane::idle_event(wxIdleEvent& event)
{
  idle();
  //event.RequestMore();
}

void BasicGLPane::mouseMoved(wxMouseEvent& event) 
{
  int x = event.GetX();
  int y = event.GetY();

  CallBackMouseMotionFunc(x, y);
  //idle();
}
void BasicGLPane::mouseDown(wxMouseEvent& event) {}
void BasicGLPane::mouseWheelMoved(wxMouseEvent& event) {}
void BasicGLPane::mouseReleased(wxMouseEvent& event) {}
void BasicGLPane::rightClick(wxMouseEvent& event) {}
void BasicGLPane::mouseLeftWindow(wxMouseEvent& event) {}


void BasicGLPane::keyPressed(wxKeyEvent& event) {
  KeyboardFunc(event.GetKeyCode(), event.CmdDown(), event.AltDown(), event.ShiftDown());
}



void BasicGLPane::keyReleased(wxKeyEvent& event) {}
 
// Vertices and faces of a simple cube to demonstrate 3D render
// source: http://www.opengl.org/resources/code/samples/glut_examples/examples/cube.c
GLfloat v[8][3];
GLint faces[6][4] = {  /* Vertex indices for the 6 faces of a cube. */
    {0, 1, 2, 3}, {3, 2, 6, 7}, {7, 6, 5, 4},
    {4, 5, 1, 0}, {5, 6, 2, 1}, {7, 4, 0, 3} };
 
 
 
BasicGLPane::BasicGLPane(wxFrame* parent, int* args) :
    wxGLCanvas(parent, wxID_ANY, args, wxDefaultPosition, wxDefaultSize, wxFULL_REPAINT_ON_RESIZE||wxWANTS_CHARS)
{

  m_context = new wxGLContext(this);
    // prepare a simple cube to demonstrate 3D render
    // source: http://www.opengl.org/resources/code/samples/glut_examples/examples/cube.c
    v[0][0] = v[1][0] = v[2][0] = v[3][0] = -1;
    v[4][0] = v[5][0] = v[6][0] = v[7][0] = 1;
    v[0][1] = v[1][1] = v[4][1] = v[5][1] = -1;
    v[2][1] = v[3][1] = v[6][1] = v[7][1] = 1;
    v[0][2] = v[3][2] = v[4][2] = v[7][2] = 1;
    v[1][2] = v[2][2] = v[5][2] = v[6][2] = -1;    
 
    // To avoid flashing on MSW
    SetBackgroundStyle(wxBG_STYLE_CUSTOM);
}
 
BasicGLPane::~BasicGLPane()
{
  delete m_context;
}
 
void BasicGLPane::resized(wxSizeEvent& evt)
{
//  wxGLCanvas::OnSize(evt);
    wxSize s = evt.GetSize();
    CallBackReshapeFunc(s.GetWidth(), s.GetHeight());
  
    Refresh();
}
 
/** Inits the OpenGL viewport for drawing in 3D. */
void BasicGLPane::prepare3DViewport(int topleft_x, int topleft_y, int bottomrigth_x, int bottomrigth_y)
{
  
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Black Background
    glClearDepth(1.0f); // Depth Buffer Setup
    glEnable(GL_DEPTH_TEST); // Enables Depth Testing
    glDepthFunc(GL_LEQUAL); // The Type Of Depth Testing To Do
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
  
    glEnable(GL_COLOR_MATERIAL);
  
    glViewport(topleft_x, topleft_y, bottomrigth_x-topleft_x, bottomrigth_y-topleft_y);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
  
    float ratio_w_h = (float)(bottomrigth_x-topleft_x)/(float)(bottomrigth_y-topleft_y);
    gluPerspective(45 /*view angle*/, ratio_w_h, 0.1 /*clip close*/, 200 /*clip far*/);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
  
}
 
int BasicGLPane::getWidth()
{
    return GetSize().x;
}
 
int BasicGLPane::getHeight()
{
    return GetSize().y;
}
 
 
void BasicGLPane::render( wxPaintEvent& evt )
{
  paint();
}


void BasicGLPane::paint(bool interruptable)
{
    if(!IsShown()) return;
    wxGLCanvas::SetCurrent(*m_context);
    wxPaintDC(this); // only to be used in paint events. use wxClientDC to paint outside the paint event
    
    glDrawBuffer(buffermode);
    // delete framebuffer and z-buffer

    //Call the display function
    DisplayItFunc(GL_RENDER, interruptable);

    // show the rednered scene
    if (!interruptable) { // interruptible draw is single buffered
      SwapBuffers();
    }
//    ProcessPendingEvents();
}


void updateControls() {
  globalGUI->updateControls();
}
void updatePointModeControls() {}
void updateTopViewControls() {}
void resetRotationButton() {}
void updateCamControls() {}

static bool interrupted;

void interruptDrawing() {
  interrupted = true;
}
void checkForInterrupt() {
  interrupted = false;
}
bool isInterrupted() {
  globalGUI->Dispatch();
  globalGUI->ProcessPendingEvents();
  return interrupted;
}
