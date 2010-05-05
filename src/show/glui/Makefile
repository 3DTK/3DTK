include ../../../Makefile.options

OBJ = ../../../obj/

.SUFFIXES: .cpp

INCS= 


libs =     ${GLLIBS}  -lXmu -lXext -lX11 -lXi -lm #-lmalloc 

LIBGLUI= -Llib -lglui

All: $(OBJ)libglui.a

GLUI_OBJS = $(OBJ)glui_add_controls.o $(OBJ)glui.o $(OBJ)glui_bitmap_img_data.o $(OBJ)glui_bitmaps.o $(OBJ)glui_button.o $(OBJ)glui_edittext.o $(OBJ)glui_checkbox.o $(OBJ)glui_node.o $(OBJ)glui_radio.o $(OBJ)glui_statictext.o $(OBJ)glui_panel.o $(OBJ)glui_separator.o $(OBJ)glui_spinner.o $(OBJ)glui_control.o $(OBJ)glui_column.o $(OBJ)glui_translation.o $(OBJ)glui_rotation.o $(OBJ)glui_mouse_iaction.o $(OBJ)glui_listbox.o $(OBJ)glui_rollout.o $(OBJ)arcball.o $(OBJ)algebra3.o $(OBJ)quaternion.o

$(OBJ)libglui.a:	$(GLUI_OBJS)
	ar -cr $(OBJ)libglui.a $(GLUI_OBJS)
	ranlib $(OBJ)libglui.a

$(OBJ)%.o:  %.cpp
	$(GPP) -c $(CFLAGS) $(CPPFLAGS) -Wno-deprecated $(INCS) $*.cpp -o $(OBJ)$*.o

clean:
	/bin/rm -f *.o


$(OBJ)algebra3.o: algebra3.h
$(OBJ)arcball.o: arcball.h algebra3.h quaternion.h stdinc.h
$(OBJ)glui.o: glui.h  stdinc.h 
$(OBJ)glui_add_controls.o: glui.h  stdinc.h 
$(OBJ)glui_bitmap_img_data.o: glui_img_checkbox_0.c glui_img_checkbox_1.c
$(OBJ)glui_bitmap_img_data.o: glui_img_radiobutton_0.c glui_img_radiobutton_1.c
$(OBJ)glui_bitmap_img_data.o: glui_img_uparrow.c glui_img_downarrow.c
$(OBJ)glui_bitmap_img_data.o: glui_img_leftarrow.c glui_img_rightarrow.c
$(OBJ)glui_bitmap_img_data.o: glui_img_spinup_1.c glui_img_spinup_0.c
$(OBJ)glui_bitmap_img_data.o: glui_img_spindown_1.c glui_img_spindown_0.c
$(OBJ)glui_bitmap_img_data.o: glui_img_checkbox_0_dis.c glui_img_checkbox_1_dis.c
$(OBJ)glui_bitmap_img_data.o: glui_img_radiobutton_0_dis.c
$(OBJ)glui_bitmap_img_data.o: glui_img_radiobutton_1_dis.c glui_img_spinup_dis.c
$(OBJ)glui_bitmap_img_data.o: glui_img_spindown_dis.c
$(OBJ)glui_bitmaps.o: glui.h stdinc.h 
$(OBJ)glui_button.o: glui.h stdinc.h 
$(OBJ)glui_checkbox.o: glui.h stdinc.h 
$(OBJ)glui_column.o: glui.h stdinc.h 
$(OBJ)glui_control.o: glui.h stdinc.h 
$(OBJ)glui_edittext.o: glui.h stdinc.h 
$(OBJ)glui_node.o: glui.h stdinc.h 
$(OBJ)glui_panel.o: glui.h stdinc.h 
$(OBJ)glui_radio.o: glui.h stdinc.h 
$(OBJ)glui_separator.o: glui.h stdinc.h 
$(OBJ)glui_spinner.o: glui.h stdinc.h 
$(OBJ)glui_statictext.o: glui.h stdinc.h 
$(OBJ)quaternion.o: quaternion.h algebra3.h stdinc.h
$(OBJ)glui_translation.o: glui.h stdinc.h  
$(OBJ)glui_rotation.o: glui.h stdinc.h  
$(OBJ)glui_mouse_iaction.o: glui.h stdinc.h  
$(OBJ)glui_listbox.o: glui.h stdinc.h  
$(OBJ)glui_rollout.o: glui.h stdinc.h 
