///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Dec 21 2009)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "show/selectionframe.h"
#include "float.h"

///////////////////////////////////////////////////////////////////////////

Selection::Selection( wxWindow* parent, wxWindowID id, const wxString& title,
    const wxPoint& pos, const wxSize& size, long style, bool advanced_controls ) : wxFrame( parent, id,
      title, pos, size, style ) {
	//this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	bSizer1 = new wxBoxSizer( wxVERTICAL );
	
	m_checkBox1 = new wxCheckBox( this, wxID_ANY, wxT("Draw Points"), wxDefaultPosition, wxDefaultSize, 0 );
	m_checkBox1->SetValue(true); 
	bSizer1->Add( m_checkBox1, 0, wxRIGHT|wxLEFT, 5 );
	
	m_checkBox2 = new wxCheckBox( this, wxID_ANY, wxT("Draw Cameras"), wxDefaultPosition, wxDefaultSize, 0 );
	m_checkBox2->SetValue(true); 
	bSizer1->Add( m_checkBox2, 0, wxRIGHT|wxLEFT, 5 );
	
	m_checkBox3 = new wxCheckBox( this, wxID_ANY, wxT("Draw Paths"), wxDefaultPosition, wxDefaultSize, 0 );
	m_checkBox3->SetValue(true); 
	bSizer1->Add( m_checkBox3, 0, wxRIGHT|wxLEFT, 5 );
	
	m_panel2 = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer3;
	bSizer3 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText2 = new wxStaticText( m_panel2, wxID_ANY, wxT("Point Size:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText2->Wrap( -1 );
	bSizer3->Add( m_staticText2, 0, wxALL, 5 );
	
	
	bSizer3->Add( 0, 0, 1, wxEXPAND, 5 );
	
	pointsize_spinner = new wxSpinCtrl( m_panel2, wxID_ANY, wxT("1"), wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 10, 0 );
	bSizer3->Add( pointsize_spinner, 0, wxRIGHT|wxLEFT, 5 );
	
	m_panel2->SetSizer( bSizer3 );
	m_panel2->Layout();
	bSizer3->Fit( m_panel2 );
	bSizer1->Add( m_panel2, 0, wxALIGN_CENTER_HORIZONTAL|wxRIGHT|wxLEFT|wxEXPAND, 5 );
	
	m_panel21 = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer31;
	bSizer31 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText21 = new wxStaticText( m_panel21, wxID_ANY, wxT("Fog:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText21->Wrap( -1 );
	bSizer31->Add( m_staticText21, 0, wxALL, 5 );
	
	
	bSizer31->Add( 0, 0, 1, wxEXPAND, 5 );
	
	wxString m_choice1Choices[] = { wxT("None"), wxT("Exp"), wxT("Exp2"), wxT("Linear"), wxT("inverted, Exp"), wxT("inverted, Exp2"), wxT("inverted, Linear") };
	int m_choice1NChoices = sizeof( m_choice1Choices ) / sizeof( wxString );
	m_choice1 = new wxChoice( m_panel21, wxID_ANY, wxDefaultPosition, wxDefaultSize, m_choice1NChoices, m_choice1Choices, 0 );
	m_choice1->SetSelection( 1 );
	bSizer31->Add( m_choice1, 0, wxRIGHT|wxLEFT, 5 );
	
	m_panel21->SetSizer( bSizer31 );
	m_panel21->Layout();
	bSizer31->Fit( m_panel21 );
	bSizer1->Add( m_panel21, 0, wxALIGN_CENTER_HORIZONTAL|wxRIGHT|wxLEFT|wxEXPAND, 5 );
	
	m_panel3 = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer6;
	bSizer6 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText4 = new wxStaticText( m_panel3, wxID_ANY, wxT("Fog Density:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText4->Wrap( -1 );
	bSizer6->Add( m_staticText4, 0, wxALL, 5 );
	
	
	bSizer6->Add( 0, 0, 1, wxEXPAND, 5 );
	
	fogdens_spinner = new wxSpinCtrlDbl( *m_panel3, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 
      0, 1.0, 0.001, 0.001 );
	bSizer6->Add( fogdens_spinner, 0, wxRIGHT|wxLEFT, 5 );
	
	m_panel3->SetSizer( bSizer6 );
	m_panel3->Layout();
	bSizer6->Fit( m_panel3 );
	bSizer1->Add( m_panel3, 0, wxALIGN_CENTER_HORIZONTAL|wxRIGHT|wxLEFT|wxEXPAND, 5 );
	/*
	wxStaticBoxSizer* sbSizer3;
	sbSizer3 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Color:") ), wxVERTICAL );
  */
  wxCollapsiblePane *colorpane = new wxCollapsiblePane(this, wxID_ANY, wxT("Colors:"));
  wxWindow *colorwin = colorpane->GetPane();
	wxBoxSizer* sbSizer3 = new wxBoxSizer( wxVERTICAL );
  colorwin->SetSizer(sbSizer3); 

	
	m_panel211 = new wxPanel( colorwin, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer311;
	bSizer311 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText211 = new wxStaticText( m_panel211, wxID_ANY, wxT("value:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText211->Wrap( -1 );
	bSizer311->Add( m_staticText211, 0, wxALL, 5 );
	
	
	bSizer311->Add( 0, 0, 1, wxEXPAND, 5 );
	
	wxString m_choice11Choices[] = { wxT("height") };

  int m_choice11NChoices = sizeof( m_choice11Choices ) / sizeof( wxString );
	m_choice11 = new wxChoice( m_panel211, wxID_ANY, wxDefaultPosition, wxDefaultSize, m_choice11NChoices, m_choice11Choices, 0 );
	m_choice11->SetSelection( 4 );
	bSizer311->Add( m_choice11, 0, wxRIGHT|wxLEFT, 5 );
	
	m_panel211->SetSizer( bSizer311 );
	m_panel211->Layout();
	bSizer311->Fit( m_panel211 );
	sbSizer3->Add( m_panel211, 0, wxRIGHT|wxLEFT|wxALIGN_RIGHT|wxEXPAND, 5 );
	
	m_panel212 = new wxPanel( colorwin, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer312;
	bSizer312 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText212 = new wxStaticText( m_panel212, wxID_ANY, wxT("map:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText212->Wrap( -1 );
	bSizer312->Add( m_staticText212, 0, wxALL, 5 );
	
	
	bSizer312->Add( 0, 0, 1, wxEXPAND, 5 );
	
	wxString m_choice12Choices[] = { wxT("Solid"), wxT("Grey"), wxT("HSV"), wxT("Jet"), wxT("Hot"), wxT("Rand"), wxT("SHSV") };
	int m_choice12NChoices = sizeof( m_choice12Choices ) / sizeof( wxString );
	m_choice12 = new wxChoice( m_panel212, wxID_ANY, wxDefaultPosition, wxDefaultSize, m_choice12NChoices, m_choice12Choices, 0 );
	m_choice12->SetSelection( 0 );
	bSizer312->Add( m_choice12, 0, wxRIGHT|wxLEFT, 5 );
	
	m_panel212->SetSizer( bSizer312 );
	m_panel212->Layout();
	bSizer312->Fit( m_panel212 );
	sbSizer3->Add( m_panel212, 0, wxRIGHT|wxLEFT|wxALIGN_RIGHT|wxEXPAND, 5 );
	
	m_panel213 = new wxPanel( colorwin, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer313;
	bSizer313 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText213 = new wxStaticText( m_panel213, wxID_ANY, wxT("type:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText213->Wrap( -1 );
	bSizer313->Add( m_staticText213, 0, wxALL, 5 );
	
	
	bSizer313->Add( 0, 0, 1, wxEXPAND, 5 );
	
	wxString m_choice13Choices[] = { wxT("None"), wxT("Id Scans by Color"), wxT("Color by Points") };
	int m_choice13NChoices = sizeof( m_choice13Choices ) / sizeof( wxString );
	m_choice13 = new wxChoice( m_panel213, wxID_ANY, wxDefaultPosition, wxDefaultSize, m_choice13NChoices, m_choice13Choices, 0 );
	m_choice13->SetSelection( 0 );
	bSizer313->Add( m_choice13, 0, wxRIGHT|wxLEFT, 5 );
	
	m_panel213->SetSizer( bSizer313 );
	m_panel213->Layout();
	bSizer313->Fit( m_panel213 );
	sbSizer3->Add( m_panel213, 0, wxRIGHT|wxLEFT|wxALIGN_RIGHT|wxEXPAND, 5 );
	
	m_panel411 = new wxPanel( colorwin, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer51;
	bSizer51 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText411 = new wxStaticText( m_panel411, wxID_ANY, wxT("Min Val:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText411->Wrap( -1 );
	bSizer51->Add( m_staticText411, 0, wxALL, 5 );
	
	
	bSizer51->Add( 0, 0, 1, wxEXPAND, 5 );
	
	m_spinCtrl61 = new wxSpinCtrlDbl( *m_panel411, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -DBL_MAX, DBL_MAX, 0 );
	bSizer51->Add( m_spinCtrl61, 0, wxRIGHT|wxLEFT, 5 );
	
	m_panel411->SetSizer( bSizer51 );
	m_panel411->Layout();
	bSizer51->Fit( m_panel411 );
	sbSizer3->Add( m_panel411, 0, wxRIGHT|wxLEFT|wxALIGN_RIGHT|wxEXPAND, 5 );
	
	m_panel41 = new wxPanel( colorwin, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer5;
	bSizer5 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText41 = new wxStaticText( m_panel41, wxID_ANY, wxT("Max Val:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText41->Wrap( -1 );
	bSizer5->Add( m_staticText41, 0, wxALL, 5 );
	
	
	bSizer5->Add( 0, 0, 1, wxEXPAND, 5 );
	
	m_spinCtrl6 = new wxSpinCtrlDbl( *m_panel41, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -DBL_MAX, DBL_MAX, 6 );
	bSizer5->Add( m_spinCtrl6, 0, wxRIGHT|wxLEFT, 5 );
	
	m_panel41->SetSizer( bSizer5 );
	m_panel41->Layout();
	bSizer5->Fit( m_panel41 );
	sbSizer3->Add( m_panel41, 0, wxRIGHT|wxLEFT|wxALIGN_RIGHT|wxEXPAND, 5 );
	
	m_button4 = new wxButton( colorwin, wxID_ANY, wxT("Reset Min/Max"), wxDefaultPosition, wxDefaultSize, 0 );
	sbSizer3->Add( m_button4, 0, wxALIGN_CENTER_HORIZONTAL|wxRIGHT|wxLEFT, 5 );
	
	bSizer1->Add( colorpane, 0, wxALIGN_CENTER_HORIZONTAL, 5 );
	
	m_staticline1 = new wxStaticLine( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL );
	bSizer1->Add( m_staticline1, 0, wxEXPAND | wxALL, 5 );
	
	m_button3 = new wxButton( this, wxID_ANY, wxT("Invert"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer1->Add( m_button3, 0, wxALIGN_CENTER_HORIZONTAL|wxRIGHT|wxLEFT, 5 );
	
	m_panel4 = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer7;
	bSizer7 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText6 = new wxStaticText( m_panel4, wxID_ANY, wxT("Anim delay:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText6->Wrap( -1 );
	bSizer7->Add( m_staticText6, 0, wxALL, 5 );
	
	m_spinCtrl3 = new wxSpinCtrl( m_panel4, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 10, 0 );
	bSizer7->Add( m_spinCtrl3, 0, wxRIGHT|wxLEFT, 5 );
	
	m_panel4->SetSizer( bSizer7 );
	m_panel4->Layout();
	bSizer7->Fit( m_panel4 );
	bSizer1->Add( m_panel4, 0, wxALIGN_CENTER_HORIZONTAL|wxRIGHT|wxLEFT, 5 );
	
	m_button5 = new wxButton( this, wxID_ANY, wxT("Animate"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer1->Add( m_button5, 0, wxALIGN_CENTER_HORIZONTAL|wxRIGHT|wxLEFT, 5 );
	
	m_staticline2 = new wxStaticLine( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL );
	bSizer1->Add( m_staticline2, 0, wxEXPAND | wxALL, 5 );
	
	/*wxStaticBoxSizer* sbSizer2;
	sbSizer2 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Camera Path:") ), wxVERTICAL );
  */
  wxCollapsiblePane *camerapane = new wxCollapsiblePane(this, wxID_ANY, wxT("Camera Path:"));
  wxWindow *camerawin = camerapane->GetPane();
	wxBoxSizer* sbSizer2 = new wxBoxSizer( wxVERTICAL );
  camerawin->SetSizer(sbSizer2); 
	
	m_panel412 = new wxPanel( camerawin, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer52;
	bSizer52 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText412 = new wxStaticText( m_panel412, wxID_ANY, wxT("File:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText412->Wrap( -1 );
	bSizer52->Add( m_staticText412, 0, wxALL, 5 );
	
	m_textCtrl1 = new wxTextCtrl( m_panel412, wxID_ANY, wxT("path.dat"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer52->Add( m_textCtrl1, 0, wxRIGHT|wxLEFT, 5 );
	
	m_panel412->SetSizer( bSizer52 );
	m_panel412->Layout();
	bSizer52->Fit( m_panel412 );
	sbSizer2->Add( m_panel412, 0, wxALIGN_CENTER_HORIZONTAL|wxRIGHT|wxLEFT, 5 );
	
	m_button51 = new wxButton( camerawin, wxID_ANY, wxT("Save Path"), wxDefaultPosition, wxDefaultSize, 0 );
	sbSizer2->Add( m_button51, 0, wxALIGN_CENTER_HORIZONTAL|wxRIGHT|wxLEFT, 5 );
	
	m_button6 = new wxButton( camerawin, wxID_ANY, wxT("Load Path"), wxDefaultPosition, wxDefaultSize, 0 );
	sbSizer2->Add( m_button6, 0, wxALIGN_CENTER_HORIZONTAL|wxRIGHT|wxLEFT, 5 );
	
	m_button14 = new wxButton( camerawin, wxID_ANY, wxT("Load Robot Path"), wxDefaultPosition, wxDefaultSize, 0 );
	sbSizer2->Add( m_button14, 0, wxALIGN_CENTER_HORIZONTAL|wxRIGHT|wxLEFT, 5 );
	
	m_staticline4 = new wxStaticLine( camerawin, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL );
	sbSizer2->Add( m_staticline4, 0, wxEXPAND|wxALL, 5 );
	
	m_checkBox4 = new wxCheckBox( camerawin, wxID_ANY, wxT("Save Animation"), wxDefaultPosition, wxDefaultSize, 0 );
	sbSizer2->Add( m_checkBox4, 0, wxRIGHT|wxLEFT, 5 );
	
	m_button7 = new wxButton( camerawin, wxID_ANY, wxT("Animate path"), wxDefaultPosition, wxDefaultSize, 0 );
	sbSizer2->Add( m_button7, 0, wxALIGN_CENTER_HORIZONTAL|wxRIGHT|wxLEFT, 5 );
	
	bSizer1->Add( camerapane, 0, wxALIGN_CENTER_HORIZONTAL, 5 );
	
	/*wxStaticBoxSizer* sbSizer21;
	sbSizer21 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Position:") ), wxVERTICAL );
  */
  wxCollapsiblePane *positionpane = new wxCollapsiblePane(this, wxID_ANY, wxT("Position:"));
  wxWindow *positionwin = positionpane->GetPane();
	wxBoxSizer* sbSizer21 = new wxBoxSizer( wxVERTICAL );
  positionwin->SetSizer(sbSizer21); 
	
	m_panel4121 = new wxPanel( positionwin, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer521;
	bSizer521 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText4121 = new wxStaticText( m_panel4121, wxID_ANY, wxT("File:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText4121->Wrap( -1 );
	bSizer521->Add( m_staticText4121, 0, wxALL, 5 );
	
	m_textCtrl11 = new wxTextCtrl( m_panel4121, wxID_ANY, wxT("pose.dat"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer521->Add( m_textCtrl11, 0, wxRIGHT|wxLEFT, 5 );
	
	m_panel4121->SetSizer( bSizer521 );
	m_panel4121->Layout();
	bSizer521->Fit( m_panel4121 );
	sbSizer21->Add( m_panel4121, 0, wxALIGN_CENTER_HORIZONTAL|wxRIGHT|wxLEFT, 5 );
	
	m_button511 = new wxButton( positionwin, wxID_ANY, wxT("Save Pose"), wxDefaultPosition, wxDefaultSize, 0 );
	sbSizer21->Add( m_button511, 0, wxALIGN_CENTER_HORIZONTAL|wxRIGHT|wxLEFT, 5 );
	
	m_button61 = new wxButton( positionwin, wxID_ANY, wxT("Load Pose"), wxDefaultPosition, wxDefaultSize, 0 );
	sbSizer21->Add( m_button61, 0, wxALIGN_CENTER_HORIZONTAL|wxRIGHT|wxLEFT, 5 );
	
	m_panel4131 = new wxPanel( positionwin, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer531;
	bSizer531 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText4131 = new wxStaticText( m_panel4131, wxID_ANY, wxT("Factor"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText4131->Wrap( -1 );
	bSizer531->Add( m_staticText4131, 0, wxALL, 5 );
	
	
	bSizer531->Add( 0, 0, 1, wxEXPAND, 5 );
	
	m_spinCtrl621 = new wxSpinCtrl( m_panel4131, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 10, 0 );
	bSizer531->Add( m_spinCtrl621, 0, wxRIGHT|wxLEFT, 5 );
	
	m_panel4131->SetSizer( bSizer531 );
	m_panel4131->Layout();
	bSizer531->Fit( m_panel4131 );
	sbSizer21->Add( m_panel4131, 0, wxEXPAND|wxRIGHT|wxLEFT, 5 );
	
	m_button71 = new wxButton( positionwin, wxID_ANY, wxT("Save image"), wxDefaultPosition, wxDefaultSize, 0 );
	sbSizer21->Add( m_button71, 0, wxALIGN_CENTER_HORIZONTAL|wxRIGHT|wxLEFT, 5 );
	
	bSizer1->Add( positionpane, 0, wxALIGN_CENTER_HORIZONTAL, 5 );
	
	m_staticline3 = new wxStaticLine( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL );
	bSizer1->Add( m_staticline3, 0, wxALL|wxEXPAND, 5 );
	/*
	wxStaticBoxSizer* sbSizer211;
	sbSizer211 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Selection:") ), wxVERTICAL );
  */
  wxCollapsiblePane *selectionpane = new wxCollapsiblePane(this, wxID_ANY, wxT("Selection:"));
  wxWindow *selectionwin = selectionpane->GetPane();
	wxBoxSizer* sbSizer211 = new wxBoxSizer( wxVERTICAL );
  selectionwin->SetSizer(sbSizer211); 
	
	m_panel41211 = new wxPanel( selectionwin, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer5211;
	bSizer5211 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText41211 = new wxStaticText( m_panel41211, wxID_ANY, wxT("File:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText41211->Wrap( -1 );
	bSizer5211->Add( m_staticText41211, 0, wxALL, 5 );
	
	m_textCtrl111 = new wxTextCtrl( m_panel41211, wxID_ANY, wxT("selected.3d"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer5211->Add( m_textCtrl111, 0, wxRIGHT|wxLEFT, 5 );
	
	m_panel41211->SetSizer( bSizer5211 );
	m_panel41211->Layout();
	bSizer5211->Fit( m_panel41211 );
	sbSizer211->Add( m_panel41211, 0, wxALIGN_CENTER_HORIZONTAL|wxRIGHT|wxLEFT, 5 );
	
	m_button5111 = new wxButton( selectionwin, wxID_ANY, wxT("Save selected points"), wxDefaultPosition, wxDefaultSize, 0 );
	sbSizer211->Add( m_button5111, 0, wxALIGN_CENTER_HORIZONTAL|wxRIGHT|wxLEFT, 5 );
	
	m_button611 = new wxButton( selectionwin, wxID_ANY, wxT("Clear selected points"), wxDefaultPosition, wxDefaultSize, 0 );
	sbSizer211->Add( m_button611, 0, wxALIGN_CENTER_HORIZONTAL|wxRIGHT|wxLEFT, 5 );
	
	m_checkBox5 = new wxCheckBox( selectionwin, wxID_ANY, wxT("Select/Unselect"), wxDefaultPosition, wxDefaultSize, 0 );
	m_checkBox5->SetValue(true); 
	sbSizer211->Add( m_checkBox5, 0, wxALIGN_CENTER_HORIZONTAL|wxRIGHT|wxLEFT|wxEXPAND, 5 );
	
	m_checkBox6 = new wxCheckBox( selectionwin, wxID_ANY, wxT("Select Voxels"), wxDefaultPosition, wxDefaultSize, 0 );
	sbSizer211->Add( m_checkBox6, 0, wxALIGN_CENTER_HORIZONTAL|wxRIGHT|wxLEFT|wxEXPAND, 5 );
	
	m_panel41311 = new wxPanel( selectionwin, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer5311;
	bSizer5311 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText41311 = new wxStaticText( m_panel41311, wxID_ANY, wxT("Depth:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText41311->Wrap( -1 );
	bSizer5311->Add( m_staticText41311, 0, wxALL, 5 );
	
	
	bSizer5311->Add( 0, 0, 1, wxEXPAND, 5 );
	
	m_spinCtrl6211 = new wxSpinCtrl( m_panel41311, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 10, 0 );
	bSizer5311->Add( m_spinCtrl6211, 0, wxRIGHT|wxLEFT, 5 );
	
	m_panel41311->SetSizer( bSizer5311 );
	m_panel41311->Layout();
	bSizer5311->Fit( m_panel41311 );
	sbSizer211->Add( m_panel41311, 0, wxEXPAND|wxRIGHT|wxLEFT, 5 );
	
	m_panel413111 = new wxPanel( selectionwin, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	wxBoxSizer* bSizer53111;
	bSizer53111 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText413111 = new wxStaticText( m_panel413111, wxID_ANY, wxT("Brushsize:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText413111->Wrap( -1 );
	bSizer53111->Add( m_staticText413111, 0, wxALL, 5 );
	
	
	bSizer53111->Add( 0, 0, 1, wxEXPAND, 5 );
	
	m_spinCtrl62111 = new wxSpinCtrl( m_panel413111, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 10, 0 );
	bSizer53111->Add( m_spinCtrl62111, 0, wxRIGHT|wxLEFT, 5 );
	
	m_panel413111->SetSizer( bSizer53111 );
	m_panel413111->Layout();
	bSizer53111->Fit( m_panel413111 );
	sbSizer211->Add( m_panel413111, 1, wxEXPAND|wxRIGHT|wxLEFT, 5 );
	
	bSizer1->Add( selectionpane, 0, wxALIGN_CENTER_HORIZONTAL, 5 );

  if (advanced_controls) {
    wxCollapsiblePane *advancedpane = new wxCollapsiblePane(this, wxID_ANY, wxT("Advanced:"));
    wxWindow *advancedwin = advancedpane->GetPane();
    wxBoxSizer* sbSizer2111 = new wxBoxSizer( wxVERTICAL );
    advancedwin->SetSizer(sbSizer2111); 

    //  ------------------------
    m_panel413112 = new wxPanel( advancedwin, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
    wxBoxSizer* bSizer53112;
    bSizer53112 = new wxBoxSizer( wxHORIZONTAL );

    m_staticText413112 = new wxStaticText( m_panel413112, wxID_ANY, wxT("Frame #:"), wxDefaultPosition, wxDefaultSize, 0 );
    m_staticText413112->Wrap( -1 );
    bSizer53112->Add( m_staticText413112, 0, wxALL, 5 );


    bSizer53112->Add( 0, 0, 1, wxEXPAND, 5 );

    frame_spin = new wxSpinCtrl( m_panel413112, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 10, 0 );
    bSizer53112->Add( frame_spin, 0, wxRIGHT|wxLEFT, 5 );

    m_panel413112->SetSizer( bSizer53112 );
    m_panel413112->Layout();
    bSizer53112->Fit( m_panel413112 );
    sbSizer2111->Add( m_panel413112, 0, wxEXPAND|wxRIGHT|wxLEFT, 5 );

    //  -----------------------
    m_panel4131111 = new wxPanel( advancedwin, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
    wxBoxSizer* bSizer531111;
    bSizer531111 = new wxBoxSizer( wxHORIZONTAL );

    m_staticText4131111 = new wxStaticText( m_panel4131111, wxID_ANY, wxT("FPS:"), wxDefaultPosition, wxDefaultSize, 0 );
    m_staticText4131111->Wrap( -1 );
    bSizer531111->Add( m_staticText4131111, 0, wxALL, 5 );


    bSizer531111->Add( 0, 0, 1, wxEXPAND, 5 );

    m_spinCtrl621111 = new wxSpinCtrl( m_panel4131111, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 1, 100, 20 );
    bSizer531111->Add( m_spinCtrl621111, 0, wxRIGHT|wxLEFT, 5 );

    m_panel4131111->SetSizer( bSizer531111 );
    m_panel4131111->Layout();
    bSizer531111->Fit( m_panel4131111 );
    sbSizer2111->Add( m_panel4131111, 0, wxEXPAND|wxRIGHT|wxLEFT, 5 );
    //////////////////
    farplane_panel = new wxPanel( advancedwin, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
    wxBoxSizer* farplane_sizer;
    farplane_sizer = new wxBoxSizer( wxHORIZONTAL );

    farplane_text = new wxStaticText( farplane_panel, wxID_ANY, wxT("farplane:"), wxDefaultPosition, wxDefaultSize, 0 );
    farplane_text->Wrap( -1 );
    farplane_sizer->Add( farplane_text, 0, wxALL, 5 );


    farplane_sizer->Add( 0, 0, 1, wxEXPAND, 5 );

    farplane_spinner = new wxSpinCtrl( farplane_panel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 1, 100000, 40000 );
    farplane_sizer->Add( farplane_spinner, 0, wxRIGHT|wxLEFT, 5 );

    farplane_panel->SetSizer( farplane_sizer );
    farplane_panel->Layout();
    farplane_sizer->Fit( farplane_panel );
    sbSizer2111->Add( farplane_panel, 0, wxEXPAND |wxRIGHT|wxLEFT, 5 );
    ////////////////////////////////
    nearplane_panel = new wxPanel( advancedwin, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
    wxBoxSizer* nearplane_sizer;
    nearplane_sizer = new wxBoxSizer( wxHORIZONTAL );

    nearplane_text = new wxStaticText( nearplane_panel, wxID_ANY, wxT("nearplane:"), wxDefaultPosition, wxDefaultSize, 0 );
    nearplane_text->Wrap( -1 );
    nearplane_sizer->Add( nearplane_text, 0, wxALL, 5 );


    nearplane_sizer->Add( 0, 0, 1, wxEXPAND, 5 );

    nearplane_spinner = new wxSpinCtrl( nearplane_panel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 1, 100000, 10 );
    nearplane_sizer->Add( nearplane_spinner, 0, wxRIGHT|wxLEFT, 5 );

    nearplane_panel->SetSizer( nearplane_sizer );
    nearplane_panel->Layout();
    nearplane_sizer->Fit( nearplane_panel );

    sbSizer2111->Add( nearplane_panel, 0, wxEXPAND | wxRIGHT|wxLEFT, 5 );
    //	bSizer1->Add( sbSizer2111, 0, wxALIGN_CENTER_HORIZONTAL, 5 );
    ///////////////////
    cycleLOD = new wxButton( advancedwin, wxID_ANY, wxT("Cycle LOD"), wxDefaultPosition, wxDefaultSize, 0 );
    sbSizer2111->Add( cycleLOD, 0, wxALIGN_CENTER_HORIZONTAL|wxRIGHT|wxLEFT, 5 );

    //////////
    lod_panel = new wxPanel( advancedwin, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
    wxBoxSizer* lod_sizer;
    lod_sizer = new wxBoxSizer( wxHORIZONTAL );
	
    lod_text = new wxStaticText( lod_panel, wxID_ANY, wxT("lod speed:"), wxDefaultPosition, wxDefaultSize, 0 );
    lod_text->Wrap( -1 );
    lod_sizer->Add( lod_text, 0, wxALL, 5 );
	
	
    lod_sizer->Add( 0, 0, 1, wxEXPAND, 5 );
	
    //lod_spinner = new wxSpinCtrl( lod_panel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 2, 1 );
    lod_spinner = new wxSpinCtrlDbl( *lod_panel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0.0, 2.0, 1.0, 0.01 );
    lod_sizer->Add( lod_spinner, 0, wxRIGHT|wxLEFT, 5 );
	
    lod_panel->SetSizer( lod_sizer );
    lod_panel->Layout();
    lod_sizer->Fit( lod_panel );
    sbSizer2111->Add( lod_panel, 0, wxEXPAND | wxRIGHT| wxLEFT, 5 );




    bSizer1->Add( advancedpane, 0, wxALIGN_CENTER_HORIZONTAL, 5 );
  } else {
    frame_spin = new wxSpinCtrl( 0, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 10, 0 );
  }
	
	m_button31 = new wxButton( this, wxID_ANY, wxT("Quit"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer1->Add( m_button31, 0, wxALIGN_CENTER_HORIZONTAL|wxRIGHT|wxLEFT, 5 );
	
	this->SetSizer( bSizer1 );
	this->Layout();
	
	// Connect Events
	this->Connect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( Selection::OnClose ) );
	m_checkBox1->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( Selection::OnDrawPoints ), NULL, this );
	m_checkBox2->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( Selection::OnDrawCameras ), NULL, this );
	m_checkBox3->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( Selection::OnDrawPaths ), NULL, this );
	pointsize_spinner->Connect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Selection::OnPointSize ), NULL, this );
	m_choice1->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( Selection::OnFogChoice ), NULL, this );
	fogdens_spinner->Connect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Selection::OnFogDensity ), NULL, this );
	m_choice11->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( Selection::OnColorValue ), NULL, this );
	m_choice12->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( Selection::OnColorMap ), NULL, this );
	m_choice13->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( Selection::OnColorType ), NULL, this );
	m_spinCtrl61->Connect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Selection::OnColorMinVal ), NULL, this );
	m_spinCtrl6->Connect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Selection::OnColorMaxVal ), NULL, this );
	m_button4->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Selection::OnColorResetMinMax ), NULL, this );
	m_button3->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Selection::OnInvert ), NULL, this );
	m_spinCtrl3->Connect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Selection::OnAnimDelay ), NULL, this );
	m_button5->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Selection::OnAnimate ), NULL, this );
	m_textCtrl1->Connect( wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler( Selection::OnCameraFile ), NULL, this );
	m_button51->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Selection::OnCameraSavePath ), NULL, this );
	m_button6->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Selection::OnCameraLoadPath ), NULL, this );
	m_button14->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Selection::OnCameraLoadRobotPath ), NULL, this );
	m_checkBox4->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( Selection::OnSaveAnimation ), NULL, this );
	m_button7->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Selection::OnAnimatePath ), NULL, this );
	m_textCtrl11->Connect( wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler( Selection::OnPositionFile ), NULL, this );
	m_button511->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Selection::OnPositionSave ), NULL, this );
	m_button61->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Selection::OnPositionLoad ), NULL, this );
	m_spinCtrl621->Connect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Selection::OnFactor ), NULL, this );
	m_button71->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Selection::OnSaveImage ), NULL, this );
	m_textCtrl111->Connect( wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler( Selection::OnSelectionFile ), NULL, this );
	m_button5111->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Selection::OnSelectionSave ), NULL, this );
	m_button611->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Selection::OnSelectionClear ), NULL, this );
	m_checkBox5->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( Selection::OnSelectionSU ), NULL, this );
	m_checkBox6->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( Selection::OnSelectionSV ), NULL, this );
	m_spinCtrl6211->Connect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Selection::OnSelectionDepth ), NULL, this );
	m_spinCtrl62111->Connect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Selection::OnSelectionBrushsize ), NULL, this );
  if (advanced_controls) {
    frame_spin->Connect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Selection::OnFrameSpinner ), NULL, this );
    m_spinCtrl621111->Connect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Selection::OnFramerateSpinner ), NULL, this );
    farplane_spinner->Connect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Selection::OnFarplaneSpinner ), NULL, this );
    nearplane_spinner->Connect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Selection::OnNearplaneSpinner ), NULL, this );
    lod_spinner->Connect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Selection::OnLODAdaption ), NULL, this );
    cycleLOD->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Selection::OnCycleLOD ), NULL, this );
  }
}

Selection::~Selection()
{
  // Disconnect Events
	this->Disconnect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( Selection::OnClose ) );
	m_checkBox1->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( Selection::OnDrawPoints ), NULL, this );
	m_checkBox2->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( Selection::OnDrawCameras ), NULL, this );
	m_checkBox3->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( Selection::OnDrawPaths ), NULL, this );
	pointsize_spinner->Disconnect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Selection::OnPointSize ), NULL, this );
	m_choice1->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( Selection::OnFogChoice ), NULL, this );
	fogdens_spinner->Disconnect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Selection::OnFogDensity ), NULL, this );
	m_choice11->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( Selection::OnColorValue ), NULL, this );
	m_choice12->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( Selection::OnColorMap ), NULL, this );
	m_choice13->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( Selection::OnColorType ), NULL, this );
	m_spinCtrl61->Disconnect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Selection::OnColorMinVal ), NULL, this );
	m_spinCtrl6->Disconnect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Selection::OnColorMaxVal ), NULL, this );
	m_button4->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Selection::OnColorResetMinMax ), NULL, this );
	m_button3->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Selection::OnInvert ), NULL, this );
	m_spinCtrl3->Disconnect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Selection::OnAnimDelay ), NULL, this );
	m_button5->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Selection::OnAnimate ), NULL, this );
	m_textCtrl1->Disconnect( wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler( Selection::OnCameraFile ), NULL, this );
	m_button51->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Selection::OnCameraSavePath ), NULL, this );
	m_button6->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Selection::OnCameraLoadPath ), NULL, this );
	m_button14->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Selection::OnCameraLoadRobotPath ), NULL, this );
	m_checkBox4->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( Selection::OnSaveAnimation ), NULL, this );
	m_button7->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Selection::OnAnimatePath ), NULL, this );
	m_textCtrl11->Disconnect( wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler( Selection::OnPositionFile ), NULL, this );
	m_button511->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Selection::OnPositionSave ), NULL, this );
	m_button61->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Selection::OnPositionLoad ), NULL, this );
	m_spinCtrl621->Disconnect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Selection::OnFactor ), NULL, this );
	m_button71->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Selection::OnSaveImage ), NULL, this );
	m_textCtrl111->Disconnect( wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler( Selection::OnSelectionFile ), NULL, this );
	m_button5111->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Selection::OnSelectionSave ), NULL, this );
	m_button611->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Selection::OnSelectionClear ), NULL, this );
	m_checkBox5->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( Selection::OnSelectionSU ), NULL, this );
	m_checkBox6->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( Selection::OnSelectionSV ), NULL, this );
	m_spinCtrl6211->Disconnect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Selection::OnSelectionDepth ), NULL, this );
	m_spinCtrl62111->Disconnect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Selection::OnSelectionBrushsize ), NULL, this );
	frame_spin->Disconnect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Selection::OnFrameSpinner ), NULL, this );
	m_spinCtrl621111->Disconnect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Selection::OnFramerateSpinner ), NULL, this );
	farplane_spinner->Disconnect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Selection::OnFarplaneSpinner ), NULL, this );
	nearplane_spinner->Disconnect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Selection::OnNearplaneSpinner ), NULL, this );
	lod_spinner->Disconnect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Selection::OnLODAdaption ), NULL, this );
	cycleLOD->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Selection::OnCycleLOD ), NULL, this );
}

Controls::Controls( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxFrame( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxBoxSizer* bSizer17;
	bSizer17 = new wxBoxSizer( wxHORIZONTAL );
	
	wxStaticBoxSizer* sbSizer5;
	sbSizer5 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Settings:") ), wxVERTICAL );
	
	wxBoxSizer* bSizer26;
	bSizer26 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText21 = new wxStaticText( this, wxID_ANY, wxT("Apex Angle"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText21->Wrap( -1 );
	bSizer26->Add( m_staticText21, 0, wxALL, 5 );
	
	
	bSizer26->Add( 0, 0, 1, wxEXPAND, 5 );
	
	apex_spinner = new wxSpinCtrlDbl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 1, 180, 60 );
	bSizer26->Add( apex_spinner, 0, wxALIGN_RIGHT|wxLEFT|wxRIGHT, 5 );
	
	sbSizer5->Add( bSizer26, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer261;
	bSizer261 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText211 = new wxStaticText( this, wxID_ANY, wxT("Parallel Zoom"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText211->Wrap( -1 );
	bSizer261->Add( m_staticText211, 0, wxALL, 5 );
	
	parallel_spinner = new wxSpinCtrlDbl( *this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 10, 100000, 2000 );
	bSizer261->Add( parallel_spinner, 0, wxRIGHT|wxLEFT, 5 );
	
	sbSizer5->Add( bSizer261, 1, wxEXPAND, 5 );
	
	bSizer17->Add( sbSizer5, 0, 0, 5 );
	
	wxStaticBoxSizer* sbSizer51;
	sbSizer51 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Mode:") ), wxVERTICAL );
	
	m_button15 = new wxButton( this, wxID_ANY, wxT("Top View"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT );
	sbSizer51->Add( m_button15, 0, wxRIGHT|wxLEFT|wxEXPAND, 5 );
	
	m_button16 = new wxButton( this, wxID_ANY, wxT("Reset Position"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT );
	sbSizer51->Add( m_button16, 0, wxRIGHT|wxLEFT|wxEXPAND, 5 );
	
	bSizer17->Add( sbSizer51, 0, 0, 5 );
	
	wxStaticBoxSizer* sbSizer52;
	sbSizer52 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Camera:") ), wxVERTICAL );
	
	wxBoxSizer* bSizer262;
	bSizer262 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText212 = new wxStaticText( this, wxID_ANY, wxT("Choose Camera"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText212->Wrap( -1 );
	bSizer262->Add( m_staticText212, 0, wxALL, 5 );
	
	
	bSizer262->Add( 0, 0, 1, wxEXPAND, 5 );
	
	camera_spinner = new wxSpinCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 10, 0 );
	bSizer262->Add( camera_spinner, 0, wxALIGN_RIGHT|wxLEFT|wxRIGHT, 5 );
	
	sbSizer52->Add( bSizer262, 1, wxEXPAND, 5 );
	
	m_button19 = new wxButton( this, wxID_ANY, wxT("Add Camera"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT );
	sbSizer52->Add( m_button19, 0, wxRIGHT|wxLEFT|wxALIGN_CENTER_HORIZONTAL, 5 );
	
	m_button20 = new wxButton( this, wxID_ANY, wxT("Delete Camera"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT );
	sbSizer52->Add( m_button20, 0, wxRIGHT|wxLEFT|wxALIGN_CENTER_HORIZONTAL, 5 );
	
	bSizer17->Add( sbSizer52, 0, 0, 5 );
	
	wxStaticBoxSizer* sbSizer521;
	sbSizer521 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Settings:") ), wxVERTICAL );
	
	m_checkBox7 = new wxCheckBox( this, wxID_ANY, wxT("MouseNav"), wxDefaultPosition, wxDefaultSize, 0 );
	m_checkBox7->SetValue(true); 
	sbSizer521->Add( m_checkBox7, 0, wxRIGHT|wxLEFT, 5 );
	
	always_box = new wxCheckBox( this, wxID_ANY, wxT("Always all Points"), wxDefaultPosition, wxDefaultSize, 0 );
	sbSizer521->Add( always_box, 0, wxRIGHT|wxLEFT, 5 );
	
	alwaysred_box = new wxCheckBox( this, wxID_ANY, wxT("Always reduce Points"), wxDefaultPosition, wxDefaultSize, 0 );
	alwaysred_box->SetValue(true); 
	sbSizer521->Add( alwaysred_box, 0, wxRIGHT|wxLEFT, 5 );
	
	bSizer17->Add( sbSizer521, 0, 0, 5 );
	
	this->SetSizer( bSizer17 );
	this->Layout();
	bSizer17->Fit( this );
	
	// Connect Events
	this->Connect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( Controls::OnClose ) );
	apex_spinner->Connect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Controls::OnApexAngle ), NULL, this );
	parallel_spinner->Connect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Controls::OnParallelZoom ), NULL, this );
	m_button15->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Controls::OnTopView ), NULL, this );
	m_button16->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Controls::OnResetPosition ), NULL, this );
	camera_spinner->Connect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Controls::OnChooseCamera ), NULL, this );
	m_button19->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Controls::OnAddCamera ), NULL, this );
	m_button20->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Controls::OnDeleteCamera ), NULL, this );
	m_checkBox7->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( Controls::OnMouseNav ), NULL, this );
	always_box->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( Controls::OnAlwaysAllPoints ), NULL, this );
	alwaysred_box->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( Controls::OnAlwaysReducePoints ), NULL, this );
}

Controls::~Controls()
{
	// Disconnect Events
	this->Disconnect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( Controls::OnClose ) );
	apex_spinner->Disconnect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Controls::OnApexAngle ), NULL, this );
	parallel_spinner->Disconnect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Controls::OnParallelZoom ), NULL, this );
	m_button15->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Controls::OnTopView ), NULL, this );
	m_button16->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Controls::OnResetPosition ), NULL, this );
	camera_spinner->Disconnect( wxEVT_COMMAND_SPINCTRL_UPDATED, wxSpinEventHandler( Controls::OnChooseCamera ), NULL, this );
	m_button19->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Controls::OnAddCamera ), NULL, this );
	m_button20->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( Controls::OnDeleteCamera ), NULL, this );
	m_checkBox7->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( Controls::OnMouseNav ), NULL, this );
	always_box->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( Controls::OnAlwaysAllPoints ), NULL, this );
	alwaysred_box->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( Controls::OnAlwaysReducePoints ), NULL, this );
}
