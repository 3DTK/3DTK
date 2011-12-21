#ifndef __INCLUDE_SELECTIONFRAME__
#define __INCLUDE_SELECTIONFRAME__

#include <wx/collpane.h>
#include <wx/string.h>
#include <wx/checkbox.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/stattext.h>
#include <wx/spinctrl.h>
#include <wx/sizer.h>
#include <wx/panel.h>
#include <wx/choice.h>
#include <wx/button.h>
#include <wx/statbox.h>
#include <wx/statline.h>
#include <wx/textctrl.h>
#include <wx/frame.h>

#include "wx/things/spinctld.h"
///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
/// Class Selection
///////////////////////////////////////////////////////////////////////////////
class Selection : public wxFrame 
{
	private:
	
	protected:
		wxCheckBox* m_checkBox1;
		wxCheckBox* m_checkBox2;
		wxCheckBox* m_checkBox3;
		wxPanel* m_panel2;
		wxStaticText* m_staticText2;
		
		wxSpinCtrl* pointsize_spinner;
		wxPanel* m_panel21;
		wxStaticText* m_staticText21;
		
		wxChoice* m_choice1;
		wxPanel* m_panel3;
		wxStaticText* m_staticText4;
		
//		wxSpinCtrl* fogdens_spinner;
		wxSpinCtrlDbl* fogdens_spinner;
		wxPanel* m_panel211;
		wxStaticText* m_staticText211;
		
		wxChoice* m_choice11;
		wxPanel* m_panel212;
		wxStaticText* m_staticText212;
		
		wxChoice* m_choice12;
		wxPanel* m_panel213;
		wxStaticText* m_staticText213;
		
		wxChoice* m_choice13;
		wxPanel* m_panel411;
		wxStaticText* m_staticText411;
		
//		wxSpinCtrl* m_spinCtrl61;
		wxSpinCtrlDbl* m_spinCtrl61;
		wxPanel* m_panel41;
		wxStaticText* m_staticText41;
		
		wxSpinCtrlDbl* m_spinCtrl6;
		wxButton* m_button4;
		wxStaticLine* m_staticline1;
		wxButton* m_button3;
		wxPanel* m_panel4;
		wxStaticText* m_staticText6;
		wxSpinCtrl* m_spinCtrl3;
		wxButton* m_button5;
		wxStaticLine* m_staticline2;
		wxPanel* m_panel412;
		wxStaticText* m_staticText412;
		wxTextCtrl* m_textCtrl1;
		wxButton* m_button51;
		wxButton* m_button6;
		wxButton* m_button14;
		wxStaticLine* m_staticline4;
		wxCheckBox* m_checkBox4;
		wxButton* m_button7;
		wxPanel* m_panel4121;
		wxStaticText* m_staticText4121;
		wxTextCtrl* m_textCtrl11;
		wxButton* m_button511;
		wxButton* m_button61;
		wxPanel* m_panel4131;
		wxStaticText* m_staticText4131;
		
		wxSpinCtrl* m_spinCtrl621;
		wxButton* m_button71;
		wxStaticLine* m_staticline3;
		wxPanel* m_panel41211;
		wxStaticText* m_staticText41211;
		wxTextCtrl* m_textCtrl111;
		wxButton* m_button5111;
		wxButton* m_button611;
		wxCheckBox* m_checkBox5;
		wxCheckBox* m_checkBox6;
		wxPanel* m_panel41311;
		wxStaticText* m_staticText41311;
		
		wxSpinCtrl* m_spinCtrl6211;
		wxPanel* m_panel413111;
		wxStaticText* m_staticText413111;
		
		wxSpinCtrl* m_spinCtrl62111;
		wxPanel* m_panel413112;
		wxStaticText* m_staticText413112;
		
		wxSpinCtrl* frame_spin;
		wxPanel* m_panel4131111;
		wxStaticText* m_staticText4131111;
		
		wxSpinCtrl* m_spinCtrl621111;
		wxPanel* farplane_panel;
		wxStaticText* farplane_text;
		
		wxSpinCtrl* farplane_spinner;
		wxPanel* nearplane_panel;
		wxStaticText* nearplane_text;
		
		wxSpinCtrl* nearplane_spinner;
		wxPanel* lod_panel;
		wxStaticText* lod_text;
		
		wxSpinCtrlDbl* lod_spinner;
		wxButton* cycleLOD;
		wxButton* m_button31;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnClose( wxCloseEvent& event ) { exit(0); event.Skip(); }
		virtual void OnDrawPoints( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnDrawCameras( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnDrawPaths( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnPointSize( wxSpinEvent& event ) { event.Skip(); }
		virtual void OnFogChoice( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnFogDensity( wxSpinEvent& event ) { event.Skip(); }
		virtual void OnColorValue( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnColorMap( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnColorType( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnColorMinVal( wxSpinEvent& event ) { event.Skip(); }
		virtual void OnColorMaxVal( wxSpinEvent& event ) { event.Skip(); }
		virtual void OnColorResetMinMax( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnInvert( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnAnimDelay( wxSpinEvent& event ) { event.Skip(); }
		virtual void OnAnimate( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnCameraFile( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnCameraSavePath( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnCameraLoadPath( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnCameraLoadRobotPath( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnSaveAnimation( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnAnimatePath( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnPositionFile( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnPositionSave( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnPositionLoad( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnFactor( wxSpinEvent& event ) { event.Skip(); }
		virtual void OnSaveImage( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnSelectionFile( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnSelectionSave( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnSelectionClear( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnSelectionSU( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnSelectionSV( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnSelectionDepth( wxSpinEvent& event ) { event.Skip(); }
		virtual void OnSelectionBrushsize( wxSpinEvent& event ) { event.Skip(); }
		virtual void OnFrameSpinner( wxSpinEvent& event ) { event.Skip(); }
		virtual void OnFramerateSpinner( wxSpinEvent& event ) { event.Skip(); }
		virtual void OnFarplaneSpinner( wxSpinEvent& event ) { event.Skip(); }
		virtual void OnNearplaneSpinner( wxSpinEvent& event ) { event.Skip(); }
		virtual void OnLODAdaption( wxSpinEvent& event ) { event.Skip(); }
		virtual void OnCycleLOD( wxCommandEvent& event ) { event.Skip(); }
		
	
	public:
		wxBoxSizer* bSizer1;
		
		Selection( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Selection"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( -1,-1 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL, bool advanced_controls = false );
		~Selection();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class Controls
///////////////////////////////////////////////////////////////////////////////
class Controls : public wxFrame 
{
	private:
	
	protected:
		wxStaticText* m_staticText21;
		
		wxSpinCtrlDbl* apex_spinner;
		wxStaticText* m_staticText211;
		wxSpinCtrlDbl* parallel_spinner;
		wxButton* m_button15;
		wxButton* m_button16;
		wxStaticText* m_staticText212;
		
		wxSpinCtrl* camera_spinner;
		wxButton* m_button19;
		wxButton* m_button20;
		wxCheckBox* m_checkBox7;
		wxCheckBox* always_box;
		wxCheckBox* alwaysred_box;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnClose( wxCloseEvent& event ) { exit(0); event.Skip(); }
		virtual void OnApexAngle( wxSpinEvent& event ) { event.Skip(); }
		virtual void OnParallelZoom( wxSpinEvent& event ) { event.Skip(); }
		virtual void OnTopView( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnResetPosition( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnChooseCamera( wxSpinEvent& event ) { event.Skip(); }
		virtual void OnAddCamera( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnDeleteCamera( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnMouseNav( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnAlwaysAllPoints( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnAlwaysReducePoints( wxCommandEvent& event ) { event.Skip(); }
		
	
	public:
		
		Controls( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Controls"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( -1,-1 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL );
		~Controls();
	
};

#endif
